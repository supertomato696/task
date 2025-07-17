cmake_minimum_required(VERSION 3.0)
project(scan_rander)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -Wall -g -ggdb")

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -std=c++17 -Wno-deprecated-declarations")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# set global params of 3rd path and include path
set(THIRDPARTY_PATH ${PROJECT_SOURCE_DIR}/thirdparty)
set(PROJECT_INCLUDE_PATH ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_INCLUDE_PATH}
    ${THIRDPARTY_PATH})

FILE(GLOB PROJECT_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

# set .exe .lib output path and add library
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# add dependencies within project
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/opencv4)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP REQUIRED)

if(OpenMP_FOUND)
    message("OpenMP_FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

add_executable(scan_render src/scan_render.cpp)
target_link_libraries(scan_render ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(flat_scan_maker src/flat_scan_maker.cpp)
target_link_libraries(flat_scan_maker ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(label012_bev_id src/label012_bev_id.cpp)