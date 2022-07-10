
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

