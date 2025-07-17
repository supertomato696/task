cmake_minimum_required(VERSION 3.0)
project(simple_mapper)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -Wall -g -ggdb")

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -std=c++17 -Wno-deprecated-declarations")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# add dependencies within project
include_directories(/usr/include/eigen3)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(../include/proj4)
link_libraries(/usr/lib/x86_64-linux-gnu/libproj.so.15 proj)

find_package(OpenMP REQUIRED)

if(OpenMP_FOUND)
    message("OpenMP_FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# spdlog
find_package(spdlog REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../include)
include_directories(../)
include_directories(${Boost_INCLUDE_DIRS})

# add_executable(simple_mapper simple_mapper.cpp)
# target_link_libraries(simple_mapper ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(cloud_to_image cloud_to_image.cpp)
# target_link_libraries(cloud_to_image ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(ground_thinner ground_thinner.cpp)
# target_link_libraries(ground_thinner ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(full_field_mapper full_field_mapper.cpp)
# target_link_libraries(full_field_mapper ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

# add_executable(utm_frame_out utm_frame_out.cpp)
# target_link_libraries(utm_frame_out ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(flat_mapper flat_mapper.cpp)
target_link_libraries(flat_mapper ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} proj)

# find_package(osqp REQUIRED)
# add_executable(simple_line_fit simple_line_fit.cpp)
# target_link_libraries(simple_line_fit ${PCL_LIBRARIES} ${OpenCV_LIBS} osqp::osqp)

# add_executable(center_line_extract center_line_extract.cpp)
# target_link_libraries(center_line_extract ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(center_line_extract2 center_line_extract2.cpp)
# target_link_libraries(center_line_extract2 ${PCL_LIBRARIES} ${OpenCV_LIBS})