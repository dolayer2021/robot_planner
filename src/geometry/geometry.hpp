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
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 * 
 * \file：   geometry.hpp
 * \brief：  
 * 
 * \author： Dolayer
 * \date：   2022/07/10

 *********************************************************************/
#pragma once

#include "boost/geometry/geometry.hpp"
#include "boost/config.hpp"
#include <vector>

namespace rpp {
	namespace geometry
	{
		namespace bg = boost::geometry;
		//点
		using point2d = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
		using point3d = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;
		using point2i = boost::geometry::model::point<int, 2, boost::geometry::cs::cartesian>;
		using point3i = boost::geometry::model::point<int, 3, boost::geometry::cs::cartesian>;

		//点集
		using multi_point2d = boost::geometry::model::multi_point<point2d>;
		using multi_point3d = boost::geometry::model::multi_point<point3d>;
		using multi_point2i = boost::geometry::model::multi_point<point2i>;
		using multi_point3i = boost::geometry::model::multi_point<point3i>;

		//线段集
		using linestring2d = boost::geometry::model::linestring<point2d>;
		using linestring3d = boost::geometry::model::linestring<point3d>;
		using linestring2i = boost::geometry::model::linestring<point2i>;
		using linestring3i = boost::geometry::model::linestring<point3i>;

		//线段集集
		using multi_linestring2d = boost::geometry::model::multi_linestring<linestring2d>;
		using multi_linestring3d = boost::geometry::model::multi_linestring<linestring3d>;
		using multi_linestring2i = boost::geometry::model::multi_linestring<linestring2i>;
		using multi_linestring3i = boost::geometry::model::multi_linestring<linestring3i>;

		//多边形
		using polygon2d = boost::geometry::model::polygon<point2d>;
		using polygon2i = boost::geometry::model::polygon<point2i>;

		//多边形集
		using multi_polygon2d = boost::geometry::model::multi_polygon<polygon2d>;
		using multi_polygon2i = boost::geometry::model::multi_polygon<polygon2i>;

		//线段
		using segment2d = boost::geometry::model::segment<point2d>;
		using segment2i = boost::geometry::model::segment<point2i>;

		using multi_segment2d = std::vector<segment2d>;
		using multi_segment2i = std::vector<segment2i>;

		//box
		using box2d = boost::geometry::model::box<point2d>;
		using box2i = boost::geometry::model::box<point2i>;
	}
}


