#include "map_resolver.hpp"
#include "voronoi.hpp"
#include "graph.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include "dijkstra.hpp"


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
	string map_url = R"(C:\Users\Administrator\Desktop\RPPA\data\map1.json)";
	string map_data = read_file(map_url);

	auto map = map_resolver(map_data);
	if (!map.has_value()) {
		return -1;
	}
	
	polygon2d boundary = map->boundary;
	multi_polygon2d obstacles = map->obstacles;
	
	voronoi v(boundary, obstacles);
	graph g = v.build();

	planner_ptr_t ptr = std::make_unique<dijkstra>(g);

	point2d start(30000, 13800);
	point2d end(51000, 31177);
	auto res = ptr->search_path(start, end);
	if (!res.has_value()) {
			return -1;
	}

	auto path = res.value();

	return 0;

}