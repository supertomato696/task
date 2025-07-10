cmake_minimum_required(VERSION 3.19)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(simple_msg CXX)

add_library(simple_msg INTERFACE)
target_include_directories(simple_msg INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include INTERFACE ${ASIO_INCLUDE_DIR})
target_link_libraries(simple_msg INTERFACE asio INTERFACE protobuf::libprotobuf)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include *.hpp)
target_sources(simple_msg PRIVATE ${SOURCE_FILES})

foreach(source IN LISTS SOURCE_FILES)
    get_filename_component(source_path "${source}" PATH)
    string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} "" relatie_source_path ${source_path})
    source_group("${relatie_source_path}" FILES "${source}")
endforeach()

add_subdirectory(src)
add_subdirectory(testing)


#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/simple_msg DESTINATION include/bos/framework)