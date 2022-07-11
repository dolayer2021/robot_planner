#map_resolver 
add_library(map_resolver OBJECT map_resolver.cpp map_resolver.hpp)
target_link_libraries(map_resolver PUBLIC Nlohmann::Json geometry)
target_include_directories(map_resolver PUBLIC ./)
set_target_properties(map_resolver PROPERTIES FOLDER test)	
#dijkstra_test
add_executable(dijkstra_test dijkstra_test.cpp)
target_link_libraries(dijkstra_test PRIVATE  voronoi map_resolver dijkstra)
set_target_properties(dijkstra_test PROPERTIES FOLDER test)	

#astar_test
add_executable(astar_test astar_test.cpp)
target_link_libraries(astar_test PRIVATE  voronoi map_resolver astar)
set_target_properties(astar_test PROPERTIES FOLDER test)	