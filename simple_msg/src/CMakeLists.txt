set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(examples CXX)


include_directories(${PROTOBUF_SOURCE_DIR}/src)

add_executable(ipc_client ${CMAKE_CURRENT_SOURCE_DIR}/ipc_client.cpp)
target_link_libraries(ipc_client PRIVATE simple_msg)
set_target_properties(ipc_client PROPERTIES FOLDER "examples")

add_executable(ipc_server ${CMAKE_CURRENT_SOURCE_DIR}/ipc_server.cpp)
target_link_libraries(ipc_server PRIVATE simple_msg)
set_target_properties(ipc_server PROPERTIES FOLDER "examples")

add_executable(rpc_client ${CMAKE_CURRENT_SOURCE_DIR}/rpc_client.cpp example.pb.cc example.pb.h)
target_link_libraries(rpc_client PRIVATE simple_msg)
set_target_properties(rpc_client PROPERTIES FOLDER "examples")

add_executable(rpc_server ${CMAKE_CURRENT_SOURCE_DIR}/rpc_server.cpp example.pb.cc example.pb.h)
target_link_libraries(rpc_server PRIVATE simple_msg)
set_target_properties(rpc_server PROPERTIES FOLDER "examples")
