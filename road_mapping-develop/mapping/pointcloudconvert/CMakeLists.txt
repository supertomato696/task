cmake_minimum_required(VERSION 3.10)
project(pointcloudconvert)
add_definitions(-w)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -std=c++17 -Wno-deprecated-declarations")
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_SOURCE_DIR}/bin")
# execute_process(COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_SOURCE_DIR}/bin")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(wgs84_to_mars/lib/x86)
include_directories(wgs84_to_mars/include)
include_directories(${Boost_INCLUDE_DIRS})


# add_subdirectory(src)
add_executable(pointcloudconvert src/pointcloudconvert.cpp)
target_link_libraries(pointcloudconvert ${PCL_LIBRARIES}   ${OpenCV_LIBS}  ${Boost_LIBRARIES} libkcoords_plugin.so)