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
 * \file:   graph.hpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/11

 *********************************************************************/
#pragma once

#include "geometry.hpp"
#include <vector>

namespace rpp
{
	using namespace geometry;
	using edge_index = std::pair<int, int>;

	struct graph
	{
		multi_point2d vertices;
		std::vector<edge_index> edge_indexs;
	};

}