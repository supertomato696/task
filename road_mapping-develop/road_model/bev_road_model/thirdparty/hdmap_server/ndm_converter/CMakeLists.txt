# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

cmake_minimum_required(VERSION 2.8.10)
project(ndm_converter C CXX)

option(WITH_GLOG "With glog" ON)
set(WITH_GLOG_VAL "1")
set(BRPC_WITH_GLOG 1)

#execute_process(
#    COMMAND bash -c "find ${PROJECT_SOURCE_DIR}/../.. -type d -regex \".*output/include$\" | head -n1 | xargs dirname | tr -d '\n'"
#    OUTPUT_VARIABLE OUTPUT_PATH
#)

set(CMAKE_PREFIX_PATH ${OUTPUT_PATH})

include(FindThreads)
include(FindProtobuf)
#protobuf_generate_cpp(PROTO_SRC PROTO_HEADER tileserver.proto)
# include PROTO_HEADER
include_directories(../proto/cpp_code)
include_directories(../data-access-engine)
include_directories(../GeometricTools/GTE/Mathematics)
include_directories(../GeometricTool/GTE)
set(PROTO_SRC ../proto/cpp_code/common/common.pb.cc 
              ../proto/cpp_code/confidence/confidence.pb.cc
              ../proto/cpp_code/dynamics/issue.pb.cc
              ../proto/cpp_code/dynamics/odd.pb.cc
              ../proto/cpp_code/lane/lane.pb.cc
              ../proto/cpp_code/link/link.pb.cc
              ../proto/cpp_code/metadata/metadata.pb.cc
              ../proto/cpp_code/position/position.pb.cc
              ../proto/cpp_code/traffic/traffic.pb.cc
              ../proto/cpp_code/services/roadserver.pb.cc
              ../proto/cpp_code/services/tileserver.pb.cc)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_path(BRPC_INCLUDE_PATH NAMES brpc/server.h)
find_library(BRPC_LIB NAMES libbrpc.a brpc)
if((NOT BRPC_INCLUDE_PATH) OR (NOT BRPC_LIB))
    message(FATAL_ERROR "Fail to find brpc")
endif()

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
set(CMAKE_CXX_FLAGS "${CMAKE_CPP_FLAGS} -DUSE_GLOG -DNDEBUG -O0 -D__const__=__unused__ -pipe -W -Wall -Wno-unused-parameter -Wno-reorder -Wno-invalid-offsetof -fPIC -fno-omit-frame-pointer")

if(CMAKE_VERSION VERSION_LESS "3.1.3")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
else()
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

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
    ${GLOG_LIB}
    dl
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

include_directories(../ndm_engine/include)
include_directories(../ndm_engine/proto)
include_directories(./)

set(DATA_ENGINES ../data-access-engine/dao/config_address.cpp
            ../data-access-engine/dao/road_tile_dao.cpp
            ../data-access-engine/manager/road_geometry_mgr.cpp
            ../data-access-engine/proxy/common_proxy.cpp
            ../data-access-engine/proxy/confidence_proxy.cpp
            ../data-access-engine/proxy/dynamics_proxy.cpp
            ../data-access-engine/proxy/feature_id_proxy.cpp
            ../data-access-engine/proxy/feature_proxy_base.cpp
            ../data-access-engine/proxy/feature_with_id_proxy_base.cpp
            ../data-access-engine/proxy/lane_proxy.cpp
            ../data-access-engine/proxy/link_proxy.cpp
            ../data-access-engine/proxy/position_proxy.cpp
            ../data-access-engine/proxy/tile_proxy.cpp
            ../data-access-engine/proxy/traffic_proxy.cpp
            ../data-access-engine/data_access_engine.cpp)
add_executable(ndm_converter ${DATA_ENGINES} ${PROTO_SRC} client.cpp jsoncpp.cpp utility.cpp)

target_link_libraries(ndm_converter ${PROJECT_SOURCE_DIR}/../ndm_engine/lib/libndm_proto.a)
target_link_libraries(ndm_converter ${BRPC_LIB} ${DYNAMIC_LIB} "stdc++fs" proj)
