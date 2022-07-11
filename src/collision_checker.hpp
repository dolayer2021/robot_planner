#pragma once
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
 * \file:   collision_checker.hpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/12

 *********************************************************************/
#include "geometry.hpp"
#include "robot_model.hpp"
#include <memory>

namespace rpp
{
	using namespace geometry;

	class collision_checker
	{
	public:
		collision_checker(const robot_model& g,const polygon2d& boundary,const multi_polygon2d& obstacles);
		collision_checker(const collision_checker&) = delete;
		collision_checker& operator=(const collision_checker&) = delete;

		collision_checker(collision_checker&&) = default;
		collision_checker& operator =(collision_checker&&) = default;
		~collision_checker();

		bool safe_position(point2d point,double angle);
		bool safe_rotation(point2d point, double start_angle, double end_angle);
		bool safe_path(point2d start_point, double start_angle, point2d end_point, double end_angle);

	private:
		struct impl;
		std::unique_ptr<impl> m_pimpl;
	};

}
