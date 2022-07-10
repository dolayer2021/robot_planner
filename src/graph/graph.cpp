/*****************************************************************//**
 * \file：   graph.cpp
 * \brief：  
 * 
 * \author： Dolayer
 * \date：   2022/07/06

 *********************************************************************/
#include "graph.hpp"


namespace rap
{
    graph::graph(const multi_linestring2d& segments)
    {
        for (const auto& segment : segments) {
            if (segment.size() == 2) {
                int front_index{ -1 }, back_index{ -1 };
                auto front_iter = std::find_if(m_vertices.begin(), m_vertices.end(), [&](point2d p) {
                    return bg::distance(p, segment.front()) < 2;
                    });
                front_index = std::distance(m_vertices.begin(), front_iter);
                if (front_iter == m_vertices.end()) {
                    m_vertices.emplace_back(segment.front());
                }

                auto back_iter = std::find_if(m_vertices.begin(), m_vertices.end(), [&](point2d p) {
                    return bg::distance(p, segment.back()) < 2;
                    });
                back_index = std::distance(m_vertices.begin(), back_iter);
                if (back_iter == m_vertices.end()) {
                    m_vertices.emplace_back(segment.back());
                }

                m_edge_indexs.push_back(edge_index(front_index, back_index));
            }
        }
    }
    multi_point2d graph::get_vertices() const
    {
        return m_vertices;
    }
    std::vector<edge_index> graph::get_edges() const
    {
        return m_edge_indexs;
    }

    point2d graph::get_vertex(int index) 
    {
        point2d p(0,0);
        if (index < m_vertices.size() && index >= 0 && m_vertices.size() > 0) {
            p = m_vertices.at(index);
        }
        return p;
    }
}