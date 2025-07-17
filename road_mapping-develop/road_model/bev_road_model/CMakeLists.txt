cmake_minimum_required(VERSION 2.8)

project(bev_road_model)
add_definitions(-DVERSION_NUM="0.1.0")
#aadd_compile_definitions(VERSION_NUM="0.1.0")

set(CMAKE_CXX_STANDARD 17)

add_definitions(-w)

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -Wall -g -ggdb")

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

find_package(Boost REQUIRED COMPONENTS system filesystem chrono thread)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_path(GFLAGS_INCLUDE_PATH gflags/gflags.h)
find_library(GFLAGS_LIBRARY NAMES gflags libgflags)
find_package(PCL 1.12.1 REQUIRED)
find_package(osqp REQUIRED)
find_package(libLAS REQUIRED)
include_directories(${libLAS_INCLUDE_DIRS})

set(THIRDPARTY_PATH ${PROJECT_SOURCE_DIR}/thirdparty)
include_directories(modules)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GFLAGS_INCLUDE_PATH})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${THIRDPARTY_PATH})
include_directories(${THIRDPARTY_PATH}/threadpool)
include_directories(${THIRDPARTY_PATH}/proj4)
include_directories(${PROJECT_SOURCE_DIR}/../version/)

link_libraries(${GFLAGS_LIBRARY})
link_libraries(${Boost_LIBRARIES})
link_libraries(${OpenCV_LIBS})
link_libraries(${PCL_LIBRARIES})
# link_libraries(${THIRDPARTY_PATH}/proj4/libproj.so.13 proj)
# link_libraries(${THIRDPARTY_PATH}/proj4/libproj.so.17 proj)
link_libraries(/usr/lib/x86_64-linux-gnu/libproj.so.15 proj)


aux_source_directory(modules/road_model ROAD_MODEL_SRC)
aux_source_directory(modules/accuracy_update ACCURACY_UPDATE_SRC)
aux_source_directory(modules/utils UTILS_SRC)
aux_source_directory(modules/dao DAO_SRC)

########################  上传 相关  #######################

include_directories(/usr/local/include)

add_definitions(-D ACCEPT_USE_OF_DEPRECATED_PROJ_API_H)

include(FindThreads)
include(FindProtobuf)

set(WITH_GLOG_VAL "1")
set(DATA_ENGINE_PATH ${THIRDPARTY_PATH}/hdmap_server/data-access-engine)
include_directories(${DATA_ENGINE_PATH})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# include_directories(${PROJECT_SOURCE_DIR}/modules/upload/proto/cpp_code)
include_directories(${THIRDPARTY_PATH}/hdmap_server/proto/cpp_code)

set(PROTO_PATH ${THIRDPARTY_PATH}/hdmap_server/proto)

set(PROTO_SRC ${PROTO_PATH}/cpp_code/common/common.pb.cc
        ${PROTO_PATH}/cpp_code/confidence/confidence.pb.cc
        ${PROTO_PATH}/cpp_code/dynamics/issue.pb.cc
        ${PROTO_PATH}/cpp_code/dynamics/odd.pb.cc
        ${PROTO_PATH}/cpp_code/lane/lane.pb.cc
        ${PROTO_PATH}/cpp_code/link/link.pb.cc
        ${PROTO_PATH}/cpp_code/metadata/metadata.pb.cc
        ${PROTO_PATH}/cpp_code/position/position.pb.cc
        ${PROTO_PATH}/cpp_code/traffic/traffic.pb.cc
        ${PROTO_PATH}/cpp_code/services/roadserver.pb.cc
    #    ${PROTO_PATH}/cpp_code/services/tileserver.pb.cc
        )
find_library(THRIFT_LIB NAMES thrift)
if (NOT THRIFT_LIB)
    set(THRIFT_LIB "")
endif()
find_library(THRIFTNB_LIB NAMES thriftnb)
if (NOT THRIFTNB_LIB)
    set(THRIFTNB_LIB "")
endif()

find_path(BRPC_INCLUDE_PATH NAMES brpc/server.h)
if(LINK_SO)
    find_library(BRPC_LIB NAMES brpc)
else()
    find_library(BRPC_LIB NAMES libbrpc.a brpc)
endif()
#if((NOT BRPC_INCLUDE_PATH) OR (NOT BRPC_LIB))
#    message(FATAL_ERROR "Fail to find brpc")
#endif()

include_directories(${BRPC_INCLUDE_PATH})

find_path(GFLAGS_INCLUDE_PATH gflags/gflags.h)
find_library(GFLAGS_LIBRARY NAMES gflags libgflags)
if((NOT GFLAGS_INCLUDE_PATH) OR (NOT GFLAGS_LIBRARY))
    message(FATAL_ERROR "Fail to find gflags")
endif()
include_directories(${GFLAGS_INCLUDE_PATH})

execute_process(
        COMMAND bash -c "grep \"namespace [_A-Za-z0-9]\\+ {\" ${GFLAGS_INCLUDE_PATH}/gflags/gflags_declare.h | head -1 | awk '{print $2}' | tr -d '\n'"
        OUTPUT_VARIABLE GFLAGS_NS
)
if(${GFLAGS_NS} STREQUAL "GFLAGS_NAMESPACE")
    execute_process(
            COMMAND bash -c "grep \"#define GFLAGS_NAMESPACE [_A-Za-z0-9]\\+\" ${GFLAGS_INCLUDE_PATH}/gflags/gflags_declare.h | head -1 | awk '{print $3}' | tr -d '\n'"
            OUTPUT_VARIABLE GFLAGS_NS
    )
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    include(CheckFunctionExists)
    CHECK_FUNCTION_EXISTS(clock_gettime HAVE_CLOCK_GETTIME)
    if(NOT HAVE_CLOCK_GETTIME)
        set(DEFINE_CLOCK_GETTIME "-DNO_CLOCK_GETTIME_IN_MAC")
    endif()
endif()

set(CMAKE_CPP_FLAGS "${DEFINE_CLOCK_GETTIME} -DGFLAGS_NS=${GFLAGS_NS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CPP_FLAGS} -DUSE_GLOG -DNDEBUG -D__const__=__unused____unused__ -pipe -W -Wall -Wno-unused-parameter -fPIC -fno-omit-frame-pointer")

find_path(LEVELDB_INCLUDE_PATH NAMES leveldb/db.h)
find_library(LEVELDB_LIB NAMES leveldb)
if ((NOT LEVELDB_INCLUDE_PATH) OR (NOT LEVELDB_LIB))
    message(FATAL_ERROR "Fail to find leveldb")
endif()
include_directories(${LEVELDB_INCLUDE_PATH})

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OPENSSL_ROOT_DIR
            "/usr/local/opt/openssl"    # Homebrew installed OpenSSL
            )
endif()

find_package(OpenSSL)
include_directories(${OPENSSL_INCLUDE_DIR})
find_path(GLOG_INCLUDE_PATH NAMES glog/logging.h)
find_library(GLOG_LIB NAMES glog)
if((NOT GLOG_INCLUDE_PATH) OR (NOT GLOG_LIB))
    message(FATAL_ERROR "Fail to find glog")
endif()
include_directories(${GLOG_INCLUDE_PATH})

# find_package(GDAL REQUIRED)
# if(NOT GDAL_FOUND)
#     message(FATAL_ERROR "GDAL library not found")
# endif()

find_package( GDAL )
if(GDAL_FOUND)
    # message(FATAL_ERROR "message(FATAL_ERROR "GDAL library found")")
    # target_include_directories(${PROJECT_NAME} PRIVATE ${GDAL_INCLUDE_DIR})
    # target_link_libraries(${PROJECT_NAME} ${GDAL_LIBRARY})
else()
    message( FATAL_ERROR "GDAL library not found" )
endif()

message(STATUS "GDAL include directories: ${GDAL_INCLUDE_DIRS}")
message(STATUS "GDAL library: ${GDAL_LIBRARY}")
include_directories(${GDAL_INCLUDE_DIRS})

set(DYNAMIC_LIB
        ${CMAKE_THREAD_LIBS_INIT}
        ${GFLAGS_LIBRARY}
        ${PROTOBUF_LIBRARIES}
        ${LEVELDB_LIB}
        ${OPENSSL_CRYPTO_LIBRARY}
        ${OPENSSL_SSL_LIBRARY}
        ${THRIFT_LIB}
        ${THRIFTNB_LIB}
        ${GLOG_LIB}
        dl
        pthread
        ${GDAL_LIBRARY}
        )

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
set(DYNAMIC_LIB ${DYNAMIC_LIB}
        pthread
        "-framework CoreFoundation"
        "-framework CoreGraphics"
        "-framework CoreData"
        "-framework CoreText"
        "-framework Security"
        "-framework Foundation"
        "-Wl,-U,_MallocExtension_ReleaseFreeMemory"
        "-Wl,-U,_ProfilerStart"
        "-Wl,-U,_ProfilerStop")
endif()
    
set(DATA_ENGINES ${DATA_ENGINE_PATH}/dao/config_address.cpp
        ${DATA_ENGINE_PATH}/dao/road_tile_dao.cpp
        ${DATA_ENGINE_PATH}/manager/road_geometry_mgr.cpp
        ${DATA_ENGINE_PATH}/proxy/common_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/confidence_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/dynamics_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/feature_id_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/feature_proxy_base.cpp
        ${DATA_ENGINE_PATH}/proxy/feature_with_id_proxy_base.cpp
        ${DATA_ENGINE_PATH}/proxy/lane_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/link_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/position_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/tile_proxy.cpp
        ${DATA_ENGINE_PATH}/proxy/traffic_proxy.cpp
        ${DATA_ENGINE_PATH}/data_access_engine.cpp
)

        
include_directories(
    ${THIRDPARTY_PATH}/include
    ${THIRDPARTY_PATH}/include/spdlog
    ${THIRDPARTY_PATH}/include/proj4
    ${THIRDPARTY_PATH}/include/geos
    #${THIRDPARTY_PATH}/include/pcl
    ${THIRDPARTY_PATH}/../Engine/Include
)

include_directories(/usr/local/include)

#centos boost
link_libraries(z
#        ${THIRDPARTY_INCLUDE}/lib/libboost_filesystem.so.1.53.0
        # ${THIRDPARTY_PATH}/lib/libboost_system.so.1.53.0
        # ${THIRDPARTY_PATH}/lib/libproj.so.13
        # ${THIRDPARTY_PATH}/lib/libEngine_Geometries.so
        # ${THIRDPARTY_PATH}/lib/libEngine_Base.so
        # ${THIRDPARTY_PATH}/lib/libEngine_Algorithm.so
        # ${THIRDPARTY_PATH}/lib/libgeos-3.8.0.so
        # ${THIRDPARTY_PATH}/lib/libjsoncpp.a
        #${THIRDPARTY_INCLUDE}/lib/liblz4.so
        #${THIRDPARTY_INCLUDE}/lib/libpcl_common.so.1.8
        #${THIRDPARTY_INCLUDE}/lib/libpcl_kdtree.so.1.8
        #/usr/local/lib/libsqlite3.so.0
        #/usr/lib/x86_64-linux-gnu/libspatialite.so.7
        #/usr/lib/x86_64-linux-gnu/libjsoncpp.a
        #/usr/lib/x86_64-linux-gnu/libgeos_c.so.1
        #/usr/lib/x86_64-linux-gnu/libgeos_c.so.1.11.1
        #/usr/lib/x86_64-linux-gnu/libgeos-3.7.1.so
        #/home/test/anaconda3/envs/HDMapBase3.6/lib/libgeos-3.8.0.so
        ${BRPC_LIB} ${DYNAMIC_LIB} "stdc++fs" proj
        )

link_directories(/usr/lib64)
link_directories(/usr/lib/x86_64-linux-gnu /usr/local/lib  ${LIBRARY_OUTPUT_PATH})       

list(REMOVE_ITEM ROAD_MODEL_SRC modules/road_model/main.cpp)
list(REMOVE_ITEM DAO_SRC modules/dao/json_to_obj.cpp)
list(REMOVE_ITEM DAO_SRC modules/dao/las_to_pcd.cpp)
###########################################################





add_executable(bev_road_model   ${DATA_ENGINES}
                                ${PROTO_SRC} 
                                ${ROAD_MODEL_SRC} 
                                ${UTILS_SRC} 
                                ${DAO_SRC}
                                ${PROJECT_SOURCE_DIR}/modules/road_model/main.cpp
                                )

target_link_libraries(bev_road_model ${BRPC_LIB} ${DYNAMIC_LIB} ${libLAS_LIBRARIES} -lpthread -lboost_thread -lboost_date_time osqp)

add_executable(json_to_obj      ${PROJECT_SOURCE_DIR}/modules/dao/json_to_obj.cpp
                                ${PROJECT_SOURCE_DIR}/modules/road_model/road_model_meta.h
                                ${PROJECT_SOURCE_DIR}/modules/road_model/road_model_meta.cpp
                                ${UTILS_SRC} 
                                ${DATA_ENGINES}
                                ${PROTO_SRC}
                                ${DAO_SRC}
                                )
target_link_libraries(json_to_obj ${BRPC_LIB} ${DYNAMIC_LIB} ${libLAS_LIBRARIES} -lpthread -lboost_thread -lboost_date_time )

add_executable(las_to_pcd      ${PROJECT_SOURCE_DIR}/modules/dao/las_to_pcd.cpp
                                ${PROJECT_SOURCE_DIR}/modules/road_model/road_model_meta.h
                                ${PROJECT_SOURCE_DIR}/modules/road_model/road_model_meta.cpp
                                ${UTILS_SRC} 
                                ${DATA_ENGINES}
                                ${PROTO_SRC}
                                ${DAO_SRC}
                                )
target_link_libraries(las_to_pcd ${BRPC_LIB} ${DYNAMIC_LIB} ${libLAS_LIBRARIES} -lpthread -lboost_thread -lboost_date_time )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
