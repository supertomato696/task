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
project(data_access_engine C CXX)

option(WITH_GLOG "With glog" ON)
set(WITH_GLOG_VAL "0")
if(WITH_GLOG)
    set(WITH_GLOG_VAL "1")
    set(BRPC_WITH_GLOG 1)
endif()

set(CMAKE_PREFIX_PATH ${OUTPUT_PATH})

include(FindThreads)
include(FindProtobuf)
#protobuf_generate_cpp(PROTO_SRC PROTO_HEADER tileserver.proto)
# include PROTO_HEADER
include_directories(../proto/cpp_code)
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
include_directories(${BRPC_INCLUDE_PATH})

find_path(GFLAGS_INCLUDE_PATH gflags/gflags.h)
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
set(CMAKE_CXX_FLAGS "${CMAKE_CPP_FLAGS} -DUSE_GLOG -DNDEBUG -O0 -D__const__=__unused____unused__ -pipe -W -Wall -Wno-unused-parameter -Wno-reorder -Wno-invalid-offsetof -fPIC -fno-omit-frame-pointer")

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

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OPENSSL_ROOT_DIR
        "/usr/local/opt/openssl"    # Homebrew installed OpenSSL
        )
endif()

find_package(OpenSSL)
include_directories(${OPENSSL_INCLUDE_DIR})

find_path(GLOG_INCLUDE_PATH NAMES glog/logging.h)
include_directories(${GLOG_INCLUDE_PATH})

set(DYNAMIC_LIB
    ${CMAKE_THREAD_LIBS_INIT}
    ${PROTOBUF_LIBRARIES}
    ${OPENSSL_CRYPTO_LIBRARY}
    ${OPENSSL_SSL_LIBRARY}
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

include_directories(./)
set(SOURCES ./dao/config_address.cpp
            ./dao/road_tile_dao.cpp
            ./manager/road_geometry_mgr.cpp
            ./proxy/common_proxy.cpp
            ./proxy/confidence_proxy.cpp
            ./proxy/dynamics_proxy.cpp
            ./proxy/feature_id_proxy.cpp
            ./proxy/feature_proxy_base.cpp
            ./proxy/feature_with_id_proxy_base.cpp
            ./proxy/lane_proxy.cpp
            ./proxy/link_proxy.cpp
            ./proxy/position_proxy.cpp
            ./proxy/tile_proxy.cpp
            ./proxy/traffic_proxy.cpp
            ./data_access_engine.cpp)
add_library(data_access_engine ${SOURCES} ${PROTO_SRC} ${PROTO_HEADER})
target_link_libraries(data_access_engine ${BRPC_LIB} ${DYNAMIC_LIB} proj)
