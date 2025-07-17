# src CMakeLists.txt
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
aux_source_directory(. srcs)
aux_source_directory(./SCHAlgorithm srcs)
aux_source_directory(./SCHAlgorithm/LSF srcs)
add_definitions(-DPCL_ONLY_CORE_POINT_TYPES -DPCLAPI_EXPORTS -DPCL_NO_PRECOMPILE)
add_definitions(-DPCPROCESS_EXPORTS)

include_directories(${PROJECT_SOURCE_DIR}/Include/Geometries)

##geos
include_directories(${THIRDPARTY_INCLUDE}/include/geos)
link_libraries(z
        ${THIRDPARTY_INCLUDE}/lib/libgeos-3.8.0.so)

#boost
link_directories(/usr/lib64
${PROJECT_SOURCE_DIR}/lib/Release)

add_library(Geometries SHARED ${srcs} )

#prompt:ubuntu need flann_cpp_s
if(THIRDPARTY_INCLUDE)

else()
    target_link_libraries(Geometries
    Engine_Base)
endif()

#target_link_libraries(Geometries Engine_Base)
set_target_properties(Geometries PROPERTIES OUTPUT_NAME ${CMAKE_PROJECT_NAME}_Geometries)
