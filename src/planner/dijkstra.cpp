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
 * \file：   dijkstra.cpp
 * \brief：  
 * 
 * \author： Dolayer
 * \date：   2022/07/11

 *********************************************************************/
#include "dijkstra.hpp"
#include "geometry.hpp"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iostream>
#include <vector>

using namespace boost;
using namespace std;
namespace rpp
{
    using edge_weight_property = property<edge_weight_t, int>; //边的权重
    using bgl_graph = adjacency_list<vecS, vecS, undirectedS, no_property, edge_weight_property>;
    using vertex_descriptor = graph_traits < bgl_graph >::vertex_descriptor;

    struct dijkstra::impl
    {
        impl(const graph& g);
        std::optional<linestring2d> search_path(point2d start_point, point2d end_point);

    private:
        int get_nearest_index(const point2d& point);
        std::optional<vector<int>> search_index(int start_index, int end_index);
    private:
        graph m_graph;
        bgl_graph m_bgl_graph;
    };

    dijkstra::impl::impl(const graph& g):m_graph(g)
    {
        auto edge_indexs = g.edge_indexs;
        auto vertices = g.vertices;
        vector<int> weights;
        for (const auto& edge_index : edge_indexs) {
            int distance = (int)bg::distance(vertices.at(edge_index.first), vertices.at(edge_index.second));
            weights.push_back(distance);
        }

        auto size = edge_indexs.size();
        m_bgl_graph = bgl_graph(edge_indexs.begin(), edge_indexs.end(), &weights[0], size);
    }

    std::optional<vector<int>> dijkstra::impl::search_index(int start_index, int end_index) 
    {
        std::vector<vertex_descriptor> p(num_vertices(m_bgl_graph));
        std::vector<bgl_graph::edge_descriptor> q(num_vertices(m_bgl_graph));
        std::vector<int> d(num_vertices(m_bgl_graph));

        vertex_descriptor s = vertex(start_index, m_bgl_graph);

        dijkstra_shortest_paths(m_bgl_graph, s,
            predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, m_bgl_graph))).
            distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, m_bgl_graph))));

        vector<int> path_indexs;
        int t = end_index;
        for (; end_index != start_index; end_index = p[end_index])
            path_indexs.push_back(end_index);
        path_indexs.push_back(start_index);

        reverse(path_indexs.begin(), path_indexs.end());
        if (path_indexs.size() > 0) {
            return std::optional<vector<int>>(path_indexs);
        }
        else {
            std::nullopt;
        }
        
    }

    std::optional<linestring2d> dijkstra::impl::search_path(point2d start_point,point2d end_point)
    {
        int start_index = get_nearest_index(start_point);
        int end_index = get_nearest_index(end_point);
        auto res = search_index(start_index, end_index);

        if (res.has_value()) {
            linestring2d path;
            path.emplace_back(start_point);
            for (auto index : res.value()) {
                auto point = m_graph.vertices.at(index);
                path.emplace_back(point);
            }
            path.emplace_back(end_point);
            return std::optional<linestring2d>(path);
        }
        else {
            return std::nullopt;
        }
    }

    int dijkstra::impl::get_nearest_index(const point2d& point)
    {
        auto vertices = m_graph.vertices;
        int index = { -1 };
        double min_distance = 1000000;
        for (int i = 0; i < vertices.size(); i++) {
            double distance = boost::geometry::distance(vertices.at(i), point);
            if (distance < min_distance) {
                index = i;
                min_distance = distance;
            }
        }

        return index;
    };

    dijkstra::dijkstra(const graph& g):planner_base(g),
        m_pimpl(std::make_unique<impl>(g))
    {

    }

    dijkstra::~dijkstra() = default;

    std::optional<linestring2d> dijkstra::search_path(point2d start_point, point2d  end_point)
    {
        return m_pimpl->search_path(start_point, end_point);
    }

}
