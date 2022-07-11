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
 * \file:   planner_base.hpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/11

 *********************************************************************/
#include"../graph/graph.hpp"
#include "geometry.hpp"
#include <optional>
#include <memory>

namespace rpp 
{
	class planner_base
	{
	public:
		planner_base(const graph& g) {};
		virtual std::optional<linestring2d> search_path(point2d start_point, point2d end_point) {
			return std::nullopt;
		};
		virtual ~planner_base() {};
	};

	using planner_ptr_t = std::unique_ptr<planner_base>;
}
