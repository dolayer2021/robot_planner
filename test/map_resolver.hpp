#pragma once

#include "geometry.hpp"
#include <string>
#include <optional>

namespace rpp
{
	using namespace geometry;

	struct map_t {
		polygon2d boundary;
		multi_polygon2d obstacles;
	};

	std::optional<map_t> map_resolver(const std::string& map_data);

}