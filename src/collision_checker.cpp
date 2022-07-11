#include "collision_checker.hpp"
/*****************************************************************
 
 * This file is part of robot-planner.
 *
 * Copyright 2022 Dolayer 
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * robot-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robot-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 * 
 * \file:   collision_checker.cpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/12

 *********************************************************************/


namespace rpp
{
	template<typename Geometry>
	Geometry rotation(const Geometry& geometry, double angle)
	{
		Geometry g;
		bg::strategy::transform::rotate_transformer<BoostGeometry::degree, double, 2, 2> r(angle);
		bg::transform(geometry, g, r);
		return g;
	}

	template<class Geometry>
	Geometry translate(const Geometry& geometry, double x, double y)
	{
		Geometry g;
		bg::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
		bg::transform(geometry, g, translate);
		return g;
	}

	class collision_checker::impl;
	collision_checker::collision_checker(const robot_model& g, const polygon2d& boundary, const multi_polygon2d& obstacles):
		m_pimpl(std::make_unique<impl>(g,boundary,obstacles))
		
	{
	}

	collision_checker::~collision_checker() = default;

	bool collision_checker::safe_position(point2d point, double angle)
	{
		return m_pimpl->safe_position(point, angle);
	}

	bool collision_checker::safe_rotation(point2d point, double start_angle, double end_angle)
	{
		return m_pimpl->safe_rotation(point, start_angle, end_angle);
	}

	bool collision_checker::safe_path(point2d start_point, double start_angle, point2d end_point, double end_angle)
	{
		return m_pimpl->safe_path(start_point, start_angle, end_point, end_angle);
	}


	struct collision_checker::impl
	{
		impl(const robot_model& model, const polygon2d& boundary, const multi_polygon2d& obstacles);
		bool safe_position(point2d point, double angle);
		bool safe_rotation(point2d point, double start_angle, double end_angle);
		bool safe_path(point2d start_point, double start_angle, point2d end_point, double end_angle);
		
	private:
		polygon2d get_robot_safe_boundary(point2d point, double angle);
		multi_polygon2d get_robot_rotation_boundarys(point2d point, double start_angle, double end_angle);
		multi_polygon2d get_robot_path_boundarys(point2d start_point, double start_angle, point2d end_point, double end_angle);

		bool collision(const polygon2d& path_boundary);
		bool collision(const multi_polygon2d& path_boundarys);
		

	private:
		robot_model m_robot_model;
		polygon2d m_boundary;
		multi_polygon2d m_obstacles;
	};

	collision_checker::impl::impl(const robot_model& model, const polygon2d& boundary, const multi_polygon2d& obstacles)
	{
		m_boundary = boundary;
		for (const auto& obstacle : obstacles) {
			m_boundary.inners().emplace_back(obstacle.outer());
		}
	}

	bool collision_checker::impl::safe_position(point2d point, double angle)
	{
		auto robotBoundary = get_robot_safe_boundary(point,angle);
		return collision(robotBoundary);
	}

	bool collision_checker::impl::safe_rotation(point2d point, double start_angle, double end_angle)
	{
		multi_polygon2d rotation_boundarys = get_robot_rotation_boundarys(point,start_angle,end_angle);
		return collision(rotation_boundarys);
	}

	bool collision_checker::impl::safe_path(point2d start_point, double start_angle, point2d end_point, double end_angle)
	{
		multi_polygon2d path_boundarys = get_robot_path_boundarys(start_point,start_angle,end_point,end_angle);
		return collision(path_boundarys);
	}

	polygon2d collision_checker::impl::get_robot_safe_boundary(point2d point, double angle)
	{
		polygon2d robot_boundary;

		double left = -m_robot_model.robot_size.at(2) - m_robot_model.safe_distance.at(2);
		double right = m_robot_model.robot_size.at(3) + m_robot_model.safe_distance.at(3);
		double top = m_robot_model.robot_size.at(0) + m_robot_model.safe_distance.at(0);
		double bottom = -m_robot_model.robot_size.at(1) - m_robot_model.safe_distance.at(1);


		robot_boundary.outer().emplace_back(point2d(left, bottom));
		robot_boundary.outer().emplace_back(point2d(left, top));
		robot_boundary.outer().emplace_back(point2d(right, top));
		robot_boundary.outer().emplace_back(point2d(right, bottom));
		robot_boundary.outer().emplace_back(point2d(left, bottom));
		bg::correct(robot_boundary);

		robot_boundary = rotation(robot_boundary, angle);
		robot_boundary = translate(robot_boundary, point.get<0>(), point.get<1>());
		return robot_boundary;
	}

	multi_polygon2d collision_checker::impl::get_robot_rotation_boundarys(point2d point, double start_angle, double end_angle)
	{
		multi_polygon2d rotation_boundarys;

		double num = std::cos(start_angle) * std::sin(end_angle) - std::cos(end_angle) * std::sin(start_angle);
		if (num > 0) {
			for (int angle = start_angle; start_angle >= end_angle; start_angle -= 5) {
				auto boundary = get_robot_safe_boundary(point, angle);
				multi_polygon2d out;
				bg::union_(boundary, rotation_boundarys, out);
				rotation_boundarys = out;
			}
		}
		else {
			for (int angle = start_angle; start_angle <= end_angle; start_angle += 5) {
				auto boundary = get_robot_safe_boundary(point, angle);
				multi_polygon2d out;
				bg::union_(boundary, rotation_boundarys, out);
				rotation_boundarys = out;
			}
		}

		auto boundary = get_robot_safe_boundary(point, end_angle);
		multi_polygon2d out;
		bg::union_(boundary, rotation_boundarys, out);
		rotation_boundarys = out;
		return rotation_boundarys;
	}

	multi_polygon2d collision_checker::impl::get_robot_path_boundarys(point2d start_point, double start_angle, point2d end_point, double end_angle)
	{
		auto start_rotation_boundarys = get_robot_rotation_boundarys(start_point, start_angle, end_angle);
		double length = bg::distance(start_point, end_point);
		double left = -m_robot_model.robot_size.at(2) - m_robot_model.safe_distance.at(2);
		double right = m_robot_model.robot_size.at(3) + m_robot_model.safe_distance.at(3);
		double top = m_robot_model.robot_size.at(0) + m_robot_model.safe_distance.at(0)+ length;
		double bottom = -m_robot_model.robot_size.at(1) - m_robot_model.safe_distance.at(1);

		polygon2d boundary;
		boundary.outer().emplace_back(point2d(left, bottom));
		boundary.outer().emplace_back(point2d(left, top));
		boundary.outer().emplace_back(point2d(right, top));
		boundary.outer().emplace_back(point2d(right, bottom));
		boundary.outer().emplace_back(point2d(left, bottom));
		bg::correct(boundary);

		boundary = rotation(boundary, end_angle);
		boundary = translate(boundary, start_point.get<0>(), start_point.get<1>());
		start_rotation_boundarys.emplace_back(boundary);

		return start_rotation_boundarys;
	}

	bool collision_checker::impl::collision(const polygon2d& path_boundary)
	{
		return bg::covered_by(path_boundary, m_boundary);
	}
	
	bool collision_checker::impl::collision(const multi_polygon2d& path_boundarys)
	{
		return bg::covered_by(path_boundarys, m_boundary);
	}

}
