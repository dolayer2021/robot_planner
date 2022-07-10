cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(robot_planner LANGUAGES  CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory( 3rdPart )
add_subdirectory( src )

option(BUILD_TEST "Build Test?" ON)

if(BUILD_TEST)
	enable_testing()
	add_subdirectory(test)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

