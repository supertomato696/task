cmake_minimum_required(VERSION 2.8)

project(roadMapping)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-w)

find_package(PCL 1.10)
message("...."${PCL_INCLUDE_DIRS}"...")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    message("linux")
endif(UNIX)

# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall -g -ggdb")

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -Wall -g -ggdb")

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")


set (THIRDPARTY_INCLUDE ${PROJECT_SOURCE_DIR}/../../bev_road_model/thirdparty)

add_definitions(-D ACCEPT_USE_OF_DEPRECATED_PROJ_API_H)
set(WITH_GLOG_VAL "1")


#########################
include(FindThreads)
include(FindProtobuf)

set(DATA_ENGINE_PATH ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine)
#message(STATUS ${PROJECT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${DATA_ENGINE_PATH})
include_directories(./proto/cpp_code)

# TODO:qzc
include_directories(${THIRDPARTY_INCLUDE}/../modules)
aux_source_directory(${THIRDPARTY_INCLUDE}/../modules/utils UTILS_SRC)
include_directories(${THIRDPARTY_INCLUDE}/threadpool)

set(PROTO_SRC ./proto/cpp_code/common/common.pb.cc
        ./proto/cpp_code/confidence/confidence.pb.cc
        ./proto/cpp_code/dynamics/issue.pb.cc
        ./proto/cpp_code/dynamics/odd.pb.cc
        ./proto/cpp_code/lane/lane.pb.cc
        ./proto/cpp_code/link/link.pb.cc
        ./proto/cpp_code/metadata/metadata.pb.cc
        ./proto/cpp_code/position/position.pb.cc
        ./proto/cpp_code/traffic/traffic.pb.cc
        ./proto/cpp_code/services/roadserver.pb.cc
#        ./proto/cpp_code/services/tileserver.pb.cc
        )

# Search for libthrift* by best effort. If it is not found and brpc is
# compiled with thrift protocol enabled, a link error would be reported.
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
set(CMAKE_CXX_FLAGS "${CMAKE_CPP_FLAGS} -DUSE_GLOG -DNDEBUG -D__const__=__unused__ -pipe -W -Wall -Wno-unused-parameter -fPIC -fno-omit-frame-pointer")

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

find_package(Boost REQUIRED COMPONENTS system filesystem chrono thread)
link_libraries(${Boost_LIBRARIES})

set(DATA_ENGINES         
        # TODO:qzc
        ${UTILS_SRC} 
        
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/dao/config_address.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/dao/road_tile_dao.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/manager/road_geometry_mgr.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/common_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/confidence_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/dynamics_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/feature_id_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/feature_proxy_base.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/feature_with_id_proxy_base.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/lane_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/link_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/position_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/tile_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/proxy/traffic_proxy.cpp
        ${THIRDPARTY_INCLUDE}/hdmap_server/data-access-engine/data_access_engine.cpp

        )

#########################

include_directories(
        ${THIRDPARTY_INCLUDE}/include
        # ${THIRDPARTY_INCLUDE}/include/spdlog
        ${THIRDPARTY_INCLUDE}/proj4
        ${THIRDPARTY_INCLUDE}/include/geos
        #${THIRDPARTY_INCLUDE}/include/pcl
        ../Engine/Include
)
include_directories(${PROJECT_SOURCE_DIR}/include)
#include directory
include_directories(/usr/local/include)

#centos boost
link_libraries(z
#        ${THIRDPARTY_INCLUDE}/lib/libboost_filesystem.so.1.53.0
#        ${THIRDPARTY_INCLUDE}/lib/libboost_system.so.1.65.1
        # ${THIRDPARTY_INCLUDE}/lib/libboost_system.so.1.53.0
        -lboost_thread -lboost_date_time
        # ${THIRDPARTY_INCLUDE}/proj4/libproj.so.13
        # ${THIRDPARTY_INCLUDE}/proj4/libproj.so.17
        /usr/lib/x86_64-linux-gnu/libproj.so.15
        ${THIRDPARTY_INCLUDE}/lib/libEngine_Geometries.so
        ${THIRDPARTY_INCLUDE}/lib/libEngine_Base.so
        ${THIRDPARTY_INCLUDE}/lib/libEngine_Algorithm.so
        ${THIRDPARTY_INCLUDE}/lib/libgeos-3.8.0.so
        ${THIRDPARTY_INCLUDE}/lib/libjsoncpp.a
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

#geos
#include_directories(/usr/local/include/geos)
#link_directories(/usr/local/lib/libgeos.so.3.8.0)

link_directories(/usr/lib64)
link_directories(/usr/lib/x86_64-linux-gnu /usr/local/lib  ${LIBRARY_OUTPUT_PATH})

aux_source_directory(./source SRCS)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(osqp REQUIRED)
find_package(Ceres REQUIRED)

add_executable(skeleton_app skeleton_test.cpp cluster.cpp skeleton_cluster.cpp pclFilter.cpp)
target_link_libraries(skeleton_app ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test test.cpp Utils.cpp earth.cpp)
#target_link_libraries(test ${PCL_LIBRARIES})
#target_link_libraries(test ${OpenCV_LIBS})

#add_executable(clusterTest clusterTest.cpp Utils.cpp earth.cpp cluster.cpp qpSpline.cpp)
#target_link_libraries(clusterTest ${PCL_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(clusterTest osqp::osqp)

#add_executable(osqpTest osqpTest.cpp)
#target_link_libraries(osqpTest osqp::osqp)

add_executable(roadBoundaryModel roadBoundaryModel.cpp processRoadBoundary.cpp cluster.cpp qpSpline.cpp Utils.cpp earth.cpp pclFilter.cpp skeleton_cluster.cpp)
target_link_libraries(roadBoundaryModel z ${OpenCV_LIBS} ${BRPC_LIB} ${DYNAMIC_LIB} ${PCL_LIBRARIES} "stdc++fs")
target_link_libraries(roadBoundaryModel osqp::osqp)

#add_executable(roadModel roadModel.cpp Utils.cpp earth.cpp cluster.cpp qpSpline.cpp processBoundary.cpp processTrafficArrow.cpp processRoadMark.cpp convexHull.cpp)
#target_link_libraries(roadModel ${PCL_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(roadModel osqp::osqp)

#add_executable(trafficArrowTest trafficArrowTest.cpp Utils.cpp)
#target_link_libraries(trafficArrowTest ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(roadMarkTest roadMarkTest.cpp Utils.cpp cluster.cpp convexHull.cpp)
#target_link_libraries(roadMarkTest ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(convexHullTest convexHullTest.cpp convexHull.cpp)

#add_executable(multipleLanesTest multipleLanesTest.cpp Utils.cpp earth.cpp)
#target_link_libraries(multipleLanesTest ${PCL_LIBRARIES})

# add_executable(postRoadModel postRoadModel.cpp postProcessLaneBoundary.cpp postProcessRoadMark.cpp cluster.cpp qpSpline.cpp Utils.cpp pclFilter.cpp)
add_executable(postRoadModel postRoadModel.cpp processRoadBoundary.cpp postProcessLaneBoundary.cpp postProcessRoadMark.cpp cluster.cpp qpSpline.cpp Utils.cpp pclFilter.cpp skeleton_cluster.cpp)
target_link_libraries(postRoadModel ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(postRoadModel osqp::osqp)

add_executable(postRoadMark postRoadMark.cpp postProcessRoadMark.cpp cluster.cpp Utils.cpp earth.cpp pclFilter.cpp ${SRCS} ${DATA_ENGINES} ${PROTO_SRC})
target_link_libraries(postRoadMark z ${OpenCV_LIBS} ${BRPC_LIB} ${DYNAMIC_LIB} ${PCL_LIBRARIES} "stdc++fs")

add_executable(postLuKou postLuKou.cpp processCrosswalk.cpp processStopline.cpp processLuKouPly.cpp processImpassableArea.cpp cluster.cpp Utils.cpp pclFilter.cpp TriangleLib.cpp TinBuild.cpp ${SRCS} ${DATA_ENGINES} ${PROTO_SRC} MathUtils.cpp)
target_link_libraries(postLuKou z ${OpenCV_LIBS} ${BRPC_LIB} ${DYNAMIC_LIB} ${PCL_LIBRARIES} ${CERES_LIBRARIES} "stdc++fs" )

add_executable(pclcloudprocess pclcloudprocess.cpp Utils.cpp earth.cpp)
target_link_libraries(pclcloudprocess ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(uploadLane uploadLane.cpp earth.cpp processCrosswalk.cpp afterProcess.cpp afterProcessLG.cpp afterProcessCommon.cpp upload2RoadServer.cpp  pclFilter.cpp Utils.cpp  AfterProcessStopLine.cpp AfterProcessJunctionBoundary.cpp bindrelation.cpp cluster.cpp ${SRCS} ${DATA_ENGINES} ${PROTO_SRC} MathUtils.h MathUtils.cpp)
target_link_libraries(uploadLane z ${OpenCV_LIBS} ${BRPC_LIB} ${DYNAMIC_LIB}  ${PCL_LIBRARIES} ${CERES_LIBRARIES} "stdc++fs")

add_executable(processPerception processPerception.cpp processPreceptionLine.cpp)
target_link_libraries(processPerception ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(combinePcdFiles combinePcdFiles.cpp)
target_link_libraries(combinePcdFiles ${PCL_LIBRARIES})

add_executable(joinKeyPoints joinKeyPoints.cpp)
target_link_libraries(joinKeyPoints ${PCL_LIBRARIES})

add_executable(processSiteBoundary processSiteBoundary.cpp Utils.cpp pclFilter.cpp cluster.cpp skeleton_cluster.cpp)
target_link_libraries(processSiteBoundary ${OpenCV_LIBS} ${PCL_LIBRARIES})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
