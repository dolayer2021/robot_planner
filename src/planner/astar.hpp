#pragma once


#include <optional>
#include "geometry.hpp"
#include "../graph/graph.hpp"
#include "planner_base.hpp"

namespace rpp
{
	using namespace geometry;

	class astar : public planner_base
	{
	public:
		astar(const graph& g);
		astar(const astar&) = delete;
		astar& operator=(const astar&) = delete;

		astar(astar&&) = default;
		astar& operator =(astar&&) = default;
		~astar();
		
		virtual std::optional<linestring2d> search_path(point2d start_point, point2d end_point);

	private:
		struct impl;
		std::unique_ptr<impl> m_pimpl;
	};

}