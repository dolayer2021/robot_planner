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
 * \file：   astar.cpp
 * \brief：  
 * 
 * \author： Dolayer
 * \date：   2022/07/11

 *********************************************************************/
#include "astar.hpp"
#include "geometry.hpp"
#include <iostream>
#include <vector>
#include <map>

using namespace boost;
using namespace std;
namespace rpp
{
    struct astar::impl
    {
        impl(const graph& g);
        std::optional<linestring2d> search_path(point2d start_point, point2d end_point);

    private:
        int get_nearest_index(const point2d& point);
        std::optional<vector<int>> search_index(int start_index, int end_index);
        double cost(int start, int goal, int index);
   
    private:
        graph m_graph;
        std::map<int, vector<int>> m_astar_graph;
    };

    astar::impl::impl(const graph& g):m_graph(g)
    {
        for (int i = 0; i < g.vertices.size(); i++) {
            m_astar_graph[i] = vector<int>();
        }

        for (const auto& edge_index : g.edge_indexs) {
            m_astar_graph[edge_index.first].emplace_back(edge_index.second);
            m_astar_graph[edge_index.second].emplace_back(edge_index.first);
        }

        for (auto& edge : m_astar_graph) {
            std::sort(edge.second.begin(), edge.second.end(), [&](int index1,int index2) {
                double dis1 = bg::distance(m_graph.vertices.at(index1), m_graph.vertices.at(edge.first));
                double dis2 = bg::distance(m_graph.vertices.at(index2), m_graph.vertices.at(edge.first));
                return dis1 > dis2;
                });
        }
    }

    std::optional<vector<int>> astar::impl::search_index(int start, int goal)
    {
        auto get_neighbors_min_cost_index = [=](int cur_index, const vector<int>& neighbor_list, const const vector<int>& close_list) ->std::optional<int> {
            if (neighbor_list.empty()) {
                return std::nullopt;
            }
            double min_cost = 100000000;
            double min_cost_index = neighbor_list.front();
            for (const auto& neighbor : neighbor_list) {
                if (std::find(close_list.begin(), close_list.end(), neighbor) == close_list.end()) {
                    double cur_cost = cost(cur_index, goal, neighbor);
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        min_cost_index = neighbor;
                    }
                }
            }
            return std::optional<int>(min_cost_index);
        };

        vector<int> path_list{ start };
        vector<int> close_list{ start };

        while (!path_list.empty()) {
            int current = path_list.back();
            while (true) {
                if (current == goal) {
                    break;
                }
                auto neighbor_list = m_astar_graph[current];
                auto res = get_neighbors_min_cost_index(current, neighbor_list, close_list);
                if (res != std::nullopt) {
                    current =res.value();
                    path_list.emplace_back(current);
                    close_list.emplace_back(current);
                }
                else {
                    path_list.pop_back();
                    break;
                }
            }
            if (current == goal) {
                break;
            }
        }
        if (path_list.empty()) {
            return std::nullopt;
        }
        else {
            return std::optional<vector<int>>(path_list);
        }
    }

    double astar::impl::cost(int start, int goal, int index) 
    {
        double cost = bg::distance(m_graph.vertices.at(start), m_graph.vertices.at(index)) +
            bg::distance(m_graph.vertices.at(goal), m_graph.vertices.at(index));
        return cost;
    }

    std::optional<linestring2d> astar::impl::search_path(point2d start_point,point2d end_point)
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

    int astar::impl::get_nearest_index(const point2d& point)
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

    astar::astar(const graph& g):planner_base(g),
        m_pimpl(std::make_unique<impl>(g))
    {

    }

    astar::~astar() = default;

    std::optional<linestring2d> astar::search_path(point2d start_point, point2d  end_point)
    {
        return m_pimpl->search_path(start_point, end_point);
    }

}
