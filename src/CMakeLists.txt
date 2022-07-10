find_package(Boost 1.50 REQUIRED)

set(project_name service)

set(src_files
    service.cpp
    robot_model.cpp
    collision_checker.cpp
)

set(include_files
    service.hpp
    robot_model.hpp
    collision_checker.hpp
)
	
	
if(BUILD_STATIC_LIB)
    set(project_name ${project_name}-static)
	add_library(${project_name} STATIC ${src_files} ${include_files}) 
elseif(BUILD_SHARED_LIB)
    set(project_name ${project_name}-shared)
	add_library(${project_name} SHARED ${src_files} ${include_files}) 
    
else()
    set(project_name ${project_name}-object)
	add_library(${project_name} OBJECT ${src_files} ${include_files}) 
endif()

set_target_properties(${project_name} PROPERTIES FOLDER service)	
add_subdirectory(graph)
add_subdirectory(planner)
add_subdirectory(trajectory)
add_subdirectory(geometry)