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
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 * 
 * \file£º   dijkstra.hpp
 * \brief£º  
 * 
 * \author£º Dolayer
 * \date£º   2022/07/11

 *********************************************************************/

#include <optional>
#include "geometry.hpp"
#include "../graph/graph.hpp"
#include "planner_base.hpp"

namespace rpp
{
	using namespace geometry;

	class dijkstra : public planner_base
	{
	public:
		dijkstra(const graph& g);
		dijkstra(const dijkstra&) = delete;
		dijkstra& operator=(const dijkstra&) = delete;

		dijkstra(dijkstra&&) = default;
		dijkstra& operator =(dijkstra&&) = default;
		~dijkstra();
		
		virtual std::optional<linestring2d> search_path(point2d start_point, point2d end_point);

	private:
		struct impl;
		std::unique_ptr<impl> m_pimpl;
	};

}