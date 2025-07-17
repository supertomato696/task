# src CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
aux_source_directory(. srcs)

add_definitions(-DPCL_ONLY_CORE_POINT_TYPES -DPCLAPI_EXPORTS -DPCL_NO_PRECOMPILE)
add_definitions(-DPCPROCESS_EXPORTS)

include_directories(${PROJECT_SOURCE_DIR}/Include/Algorithm)

#boost
link_directories(/usr/lib64
		${PROJECT_SOURCE_DIR}/lib/Release)

add_library(Algorithm SHARED ${srcs} )
#${PROJECT_SOURCE_DIR}/Source/iVectorSemi/CWall.cpp)

#prompt:ubuntu need flann_cpp_s
if(THIRDPARTY_INCLUDE)

else()
    target_link_libraries(Algorithm
    Engine_Geometries
	log4cpp)
endif()

#target_link_libraries(Algorithm Engine_Geometries log4cpp)
set_target_properties(Algorithm PROPERTIES OUTPUT_NAME ${CMAKE_PROJECT_NAME}_Algorithm)
