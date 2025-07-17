cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#工程名
project(Engine)
if(NOT CMAKE_BUILD_TYPE)
	set (CMAKE_BUILD_TYPE "Debug")
endif()

set (THIRDPARTY_INCLUDE ${PROJECT_SOURCE_DIR}/../ThirdParty)

if(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif(UNIX)

#编译标示
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	add_definitions(-D_DEBUG)
	add_definitions("-Wall -g")
endif()

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}/)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}/)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/lib/)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../ThirdParty/lib/)

#include directory
include_directories(${THIRDPARTY_INCLUDE}/include )
include_directories(${PROJECT_SOURCE_DIR}/Include )
include_directories(/usr/local/include)   

#centos boost
include_directories(${THIRDPARTY_INCLUDE}/include/geos)
link_libraries(z
		${THIRDPARTY_INCLUDE}/lib/libgeos-3.8.0.so)

link_directories(/usr/lib64)    
link_directories(/usr/lib/x86_64-linux-gnu /usr/local/lib  ${LIBRARY_OUTPUT_PATH})

#子目录
add_subdirectory(Source/Base)
add_subdirectory(Source/Geometries)
add_subdirectory(Source/Algorithm)
