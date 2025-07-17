cmake_minimum_required(VERSION 2.5)
project(road_model)

add_definitions(-w)

add_subdirectory(bev_road_model)
add_subdirectory(semantic_road_model/roadMapping)
