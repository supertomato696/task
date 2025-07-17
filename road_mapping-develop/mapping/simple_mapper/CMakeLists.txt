cmake_minimum_required(VERSION 3.10)
project(simple_mapper_apps)
add_definitions(-w)

execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_SOURCE_DIR}/bin")
execute_process(COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_SOURCE_DIR}/cloud_split/demo.las" "${PROJECT_SOURCE_DIR}/bin")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(src)
# add_subdirectory(gicp_mapper)
# add_subdirectory(pgo)
# add_subdirectory(cloud_split)
