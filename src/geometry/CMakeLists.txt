add_library(geometry INTERFACE geometry.hpp)
target_include_directories(geometry INTERFACE ./)
target_include_directories(geometry INTERFACE ./ ${Boost_INCLUDE_DIRS})


set_target_properties(geometry PROPERTIES FOLDER geometry)
