add_library(voronoi  OBJECT voronoi.cpp voronoi.hpp graph.hpp)
target_include_directories(voronoi PUBLIC ./)
target_link_libraries(voronoi  PRIVATE geometry)

set_target_properties(voronoi PROPERTIES FOLDER graph)