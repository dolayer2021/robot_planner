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
 * \file:   map_resolver.cpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/11

 *********************************************************************/
#include "map_resolver.hpp"
#include "json.hpp"
#include <string>
#include <iostream>

namespace rpp 
{
    using  std::cout;
    using  std::endl;

	std::optional<map_t> map_resolver(const std::string& map_data)
	{
        using namespace nlohmann;
        using std::string;
        try {
            json js = json::parse(map_data);
            string boundary_wkt= js.at("boundary").get<string>();
            auto obstaclesJs = js.at("obstacles");
            
            polygon2d boundary;
            bg::read_wkt(boundary_wkt, boundary);
            bg::correct(boundary);

            multi_polygon2d obstacles;
            for (auto obstacleJs : obstaclesJs) {
                auto obstacle_wkt = obstacleJs.get<string>();
                polygon2d obstacle;
                bg::read_wkt(obstacle_wkt, obstacle);
                bg::correct(obstacle);
                obstacles.emplace_back(obstacle);
            }
            
            if (bg::is_valid(boundary) || bg::is_valid(obstacles)) {
                return map_t{ boundary ,obstacles };
            }
            else {
                return std::nullopt;
            }
        }
        catch (std::exception& e) {
            cout << e.what() << endl;
            return std::nullopt;
        }
	}

}

