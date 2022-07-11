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
 * \file:   map_resolver.hpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/11

 *********************************************************************/
#include "geometry.hpp"
#include <string>
#include <optional>

namespace rpp
{
	using namespace geometry;

	struct map_t {
		polygon2d boundary;
		multi_polygon2d obstacles;
	};

	std::optional<map_t> map_resolver(const std::string& map_data);

}