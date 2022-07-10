#map_resolver 
add_library(map_resolver OBJECT map_resolver.cpp map_resolver.hpp)
target_link_libraries(map_resolver PUBLIC Nlohmann::Json geometry)
target_include_directories(map_resolver PUBLIC ./)

#voronoi test
add_executable(voronoi_test voronoi_test.cpp)
target_link_libraries(voronoi_test  PRIVATE voronoi map_resolver)
set_target_properties(voronoi_test PROPERTIES FOLDER test)	

#map_resolver_test
add_executable(map_resolver_test map_resolver_test.cpp)
target_link_libraries(map_resolver_test PRIVATE map_resolver)
set_target_properties(map_resolver_test PROPERTIES FOLDER test)	

#dijkstra_test
add_executable(dijkstra_test dijkstra_test.cpp)
target_link_libraries(dijkstra_test PRIVATE  voronoi map_resolver dijkstra)
set_target_properties(dijkstra_test PROPERTIES FOLDER test)	