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
 * \file:   astar_test.cpp
 * \brief:  
 * 
 * \author: Dolayer
 * \date:   2022/07/11

 *********************************************************************/
#include "map_resolver.hpp"
#include "voronoi.hpp"
#include "graph.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include "astar.hpp"


using namespace std;
using namespace rpp;

std::string read_file(const std::string& url)
{
	std::string data;
	ifstream file;
	file.open(url);
	if (!file.is_open()){
		std::cout << "Open File Error:" << url << std::endl;
		return data;
	}

	string lineData;

	while (getline(file, lineData)){
		data += lineData;
	}
	return data;
}


int main(int argc, char* argv[])
{
	string map_url = R"(C:\Users\Administrator\Desktop\robot_planner\map\map2.json)";
	string map_data = read_file(map_url);

	auto map = map_resolver(map_data);
	if (!map.has_value()) {
		return -1;
	}
	
	polygon2d boundary = map->boundary;
	multi_polygon2d obstacles = map->obstacles;
	
	voronoi v(boundary, obstacles);
	graph g = v.build();


	planner_ptr_t ptr = std::make_unique<astar>(g);

	point2d start(30000, 13800);
	point2d end(51000, 31177);
	auto res = ptr->search_path(start, end);
	if (!res.has_value()) {
			return -1;
	}

	auto path = res.value();

	return 0;

}