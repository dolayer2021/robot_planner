add_library(dijkstra dijkstra.cpp dijkstra.hpp planner_base.hpp)
target_include_directories(dijkstra PUBLIC ./)
target_link_libraries(dijkstra  PRIVATE geometry)

set_target_properties(dijkstra PROPERTIES FOLDER planner)