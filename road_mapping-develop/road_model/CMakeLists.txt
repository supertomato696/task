cmake_minimum_required(VERSION 2.5)
project(road_model)

add_definitions(-w)

include_directories(version)

# add_subdirectory(bev_road_model)
add_subdirectory(fast_road_model)
add_subdirectory(semantic_road_model/roadMapping)
