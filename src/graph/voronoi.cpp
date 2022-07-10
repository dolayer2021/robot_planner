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
 * \file£º   voronoi.cpp
 * \brief£º  
 * 
 * \author£º Dolayer
 * \date£º   2022/07/11

 *********************************************************************/
#include "voronoi.hpp"
#include "boost/polygon/voronoi.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

namespace boost {
	namespace polygon {
		using rpp::geometry::point2i;
		using rpp::geometry::segment2i;
		template <>
		struct geometry_concept<point2i> {
			typedef point_concept type;
		};

		template <>
		struct point_traits<point2i> {
			typedef int coordinate_type;

			static inline coordinate_type get(const point2i& point,
				orientation_2d orient) {
				return (orient == HORIZONTAL) ? point.get<0>() : point.get<1>();
			}
		};

		template <>
		struct geometry_concept<segment2i> {
			typedef segment_concept type;
		};

		template <>
		struct segment_traits<segment2i> {
			typedef int coordinate_type;
			typedef point2i point_type;

			static inline point_type get(const segment2i& segment,
				direction_1d dir) {
				return dir.to_int() ? segment.second : segment.first;
			}
		};
	}  // namespace polygon
}  // namespace boost



namespace rpp 
{
	struct voronoi::impl
	{
		impl(const polygon2d& boundary, const multi_polygon2d& obstacles);
		graph build();

	private:
		multi_linestring2d buil_bpl_voronoi(const polygon2d& boundary, const multi_polygon2d& obstacles);
		multi_linestring2d simplify(const polygon2d& boundary, const multi_polygon2d& obstacles, multi_linestring2d&& edges);
		graph edges_to_graph(multi_linestring2d&& edges);
	private:
		polygon2d m_boundary;
		multi_polygon2d m_obstacles;
	};

	voronoi::impl::impl(const polygon2d& boundary, const multi_polygon2d& obstacles):
		m_boundary(boundary),
		m_obstacles(obstacles)
	{
	
	}

	/** build voronoi */
	graph voronoi::impl::build()
	{
		auto edges = buil_bpl_voronoi(m_boundary, m_obstacles);
		auto simplify_edges = simplify(m_boundary, m_obstacles, std::move(edges));
		return edges_to_graph(std::move(simplify_edges));
	}

	graph voronoi::impl::edges_to_graph(multi_linestring2d&& edges)
	{
		graph g;
		for (const auto& edge : edges) {
			if (edge.size() == 2) {
				int front_index{ -1 }, back_index{ -1 };
				auto front_iter = std::find_if(g.vertices.begin(), g.vertices.end(), [&](point2d p) {
					return bg::distance(p, edge.front()) < 2;
					});
				front_index = std::distance(g.vertices.begin(), front_iter);
				if (front_iter == g.vertices.end()) {
					g.vertices.emplace_back(edge.front());
				}

				auto back_iter = std::find_if(g.vertices.begin(), g.vertices.end(), [&](point2d p) {
					return bg::distance(p, edge.back()) < 2;
					});
				back_index = std::distance(g.vertices.begin(), back_iter);
				if (back_iter == g.vertices.end()) {
					g.vertices.emplace_back(edge.back());
				}

				g.edge_indexs.push_back(edge_index(front_index, back_index));
			}
		}
		return g;
	}


	/** create boost polygon voronoi */
	multi_linestring2d voronoi::impl::buil_bpl_voronoi(const polygon2d& boundary, const multi_polygon2d& obstacles)
	{
		multi_segment2i voronoi_segments;

		auto polygon_to_segments = [&](const polygon2d& polygon) {
			multi_segment2i segments;
			if (true) {
				auto outer = polygon.outer();
				for (size_t i = 0; i < outer.size() - 1; ++i) {
					int x0 = outer.at(i).get<0>();
					int y0 = outer.at(i).get<1>();
					int x1 = outer.at(i + 1).get<0>();
					int y1 = outer.at(i + 1).get<1>();
					segment2i segment{ point2i(x0,y0),point2i(x1,y1) };
					segments.emplace_back(segment);
				}
			}
			return segments;
		};

		auto boundary_segments = polygon_to_segments(boundary);
		voronoi_segments.insert(voronoi_segments.end(), boundary_segments.begin(), boundary_segments.end());

		for (const auto& obstacle : obstacles) {
			auto obstracle_segments = polygon_to_segments(obstacle);
			voronoi_segments.insert(voronoi_segments.end(), obstracle_segments.begin(), obstracle_segments.end());
		}
		voronoi_diagram<double> vd;
		boost::polygon::construct_voronoi(voronoi_segments.begin(), voronoi_segments.end(), &vd);

		multi_linestring2d edges;
		for (auto iter = vd.edges().begin(); iter != vd.edges().end(); iter++) {
			if (iter->is_finite()) {
				auto p0 = point2d(iter->vertex0()->x(), iter->vertex0()->y());
				auto p1 = point2d(iter->vertex1()->x(), iter->vertex1()->y());
				linestring2d segment{ p0,p1 };
				edges.emplace_back(segment);
			}
		}

		return edges;
	}

	/** simplify voronoi edeges */
	multi_linestring2d voronoi::impl::simplify(const polygon2d& boundary, const multi_polygon2d& obstacles,multi_linestring2d&& edges)
	{
		multi_linestring2d simplify_edges;
		for (const auto& edge : edges) {
			if (edge.size() == 2) {
				auto p0 = edge.front();
				auto p1 = edge.back();

				if (bg::within(p0, boundary) && bg::within(p1, boundary) && bg::distance(obstacles, edge) > 1000) {
					if (simplify_edges.size() > 0) {
						if (!bg::covered_by(edge, simplify_edges)) {
							simplify_edges.emplace_back(edge);
						}
					}
					else {
						simplify_edges.emplace_back(edge);
					}
				}
			}
		}
		return simplify_edges;
	}

	voronoi::voronoi(const polygon2d& boundary, const multi_polygon2d& obstacles):
		m_pimpl(std::make_unique<impl>(boundary, obstacles))
	{
		
	}

	voronoi::~voronoi() = default;

	graph voronoi::build()
	{
		return m_pimpl->build();
	}
}

