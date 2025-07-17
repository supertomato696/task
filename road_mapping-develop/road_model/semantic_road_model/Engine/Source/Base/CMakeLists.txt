# src CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
set (CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
aux_source_directory(. srcs)
add_definitions(-DPCL_ONLY_CORE_POINT_TYPES -DPCLAPI_EXPORTS -DPCL_NO_PRECOMPILE)
add_definitions(-DPCPROCESS_EXPORTS)

include_directories(${PROJECT_SOURCE_DIR}/Include/Base)

#boost
link_directories(/usr/lib64)

add_library(Base SHARED ${srcs} )

#prompt:ubuntu need flann_cpp_s
if(THIRDPARTY_INCLUDE)
  
else()
    target_link_libraries(Base 
    sqlite3)
endif()

set_target_properties(Base PROPERTIES OUTPUT_NAME ${CMAKE_PROJECT_NAME}_Base)