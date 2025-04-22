cmake_minimum_required(VERSION 3.20)
project(mcp_prompt_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------
# third‑party header‑only libs作为 INTERFACE 目标
# ---------------------------------------------------------------------
function(add_header_only name include_dir)
  add_library(${name} INTERFACE)
  target_include_directories(${name} INTERFACE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/${include_dir})
endfunction()

add_header_only(asio                  asio/asio/include)
add_header_only(nlohmann_json         json/single_include)   # 或 include
add_header_only(inja                  inja/include)
add_header_only(httplib               httplib)
add_header_only(sqlite_modern_cpp     sqlite_modern_cpp/hdr)
add_header_only(json_validator        json-schema-validator/single_include) # 可选

# ---------------------------------------------------------------------
# 系统/已安装依赖（需要链接）
# ---------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)        # cpp‑httplib https
find_package(SQLite3 REQUIRED)

# ---------------------------------------------------------------------
# 核心库
# ---------------------------------------------------------------------
add_library(mcp_core
        src/transport/AsioTcpTransport.cpp
        src/prompt/MultiModalAssembler.cpp
        src/prompt/PromptService.cpp
        src/prompt/SqlitePromptStore.cpp
        src/protocol/InitializeService.cpp
        src/resource/ResourceManager.cpp
        src/resource/ResourceService.cpp
        src/resource/resolvers/LocalFileResolver.cpp
        src/resource/resolvers/HttpResolver.cpp
        src/tools/ToolRegistry.cpp
        src/tools/ToolService.cpp
        src/tools/DemoTools.cpp


)

target_include_directories(mcp_core PUBLIC include)

target_link_libraries(mcp_core
    PUBLIC
        asio
        nlohmann_json
        inja
        httplib          # header‑only；仍需 link OpenSSL
        sqlite_modern_cpp
        json_validator   # 如果用到了
        Threads::Threads
        OpenSSL::SSL OpenSSL::Crypto
        SQLite::SQLite3
)

# ---------------------------------------------------------------------
# Server executable
# ---------------------------------------------------------------------
add_executable(mcp_promptd src/server_main.cpp)
target_link_libraries(mcp_promptd PRIVATE mcp_core)

# ---------------------------------------------------------------------
# Client SDK
# ---------------------------------------------------------------------
add_library(mcp_client src/client/McpClient.cpp)
target_include_directories(mcp_client PUBLIC include)
target_link_libraries(mcp_client
    PUBLIC
        asio
        nlohmann_json
        Threads::Threads
)

#--------------------------client_main
add_executable(mcp_client_demo src/client_main.cpp)
target_link_libraries(mcp_client_demo PRIVATE mcp_client)


# ---------------------------------------------------------------------
# Tests (可选)
# ---------------------------------------------------------------------
# enable_testing()
# add_subdirectory(tests)





# cmake_minimum_required(VERSION 3.20)
# project(mcp_prompt_cpp LANGUAGES CXX)

# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# # Dependencies -----------------------------------------------------------
# include(FetchContent)
# FetchContent_Declare(
#         asio
#         GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
#         GIT_TAG        asio-1-28-0
# )
# FetchContent_MakeAvailable(asio)

# FetchContent_Declare(
#         nlohmann_json
#         GIT_REPOSITORY https://github.com/nlohmann/json.git
#         GIT_TAG        v3.11.2
# )
# FetchContent_MakeAvailable(nlohmann_json)



# FetchContent_Declare(
#         inja
#         GIT_REPOSITORY https://github.com/pantor/inja.git
#         GIT_TAG        v3.4.0
# )
# FetchContent_MakeAvailable(inja)


# FetchContent_Declare(
#         sqlite_modern_cpp
#         GIT_REPOSITORY https://github.com/SqliteModernCpp/sqlite_modern_cpp.git
#         GIT_TAG        v3.2
# )
# FetchContent_MakeAvailable(sqlite_modern_cpp)

# find_package(SQLite3 REQUIRED)

# target_link_libraries(mcp_core PUBLIC
#         sqlite_modern_cpp
#         SQLite::SQLite3
# )

# FetchContent_Declare(
#         httplib
#         GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
#         GIT_TAG        v0.15.3
# )
# FetchContent_MakeAvailable(httplib)

# target_link_libraries(mcp_core PUBLIC httplib::httplib)

# # Targets ---------------------------------------------------------------
# add_library(mcp_core
#         src/transport/AsioTcpTransport.cpp
# #        src/protocol/JsonRpc.cpp
# #        src/protocol/McpTypes.cpp
# #        src/prompt/TemplateEngine.cpp
# #        src/prompt/PromptStore.cpp
#         src/prompt/MultiModalAssembler.cpp
#         src/prompt/PromptService.cpp
# #        src/resource/ResourceManager.cpp
# #        src/resource/resolvers/LocalFileResolver.cpp
# #        src/resource/resolvers/HttpResolver.cpp
# )

# target_sources(mcp_core PRIVATE
#     src/tools/ToolRegistry.cpp
#     src/tools/ToolService.cpp
#     src/tools/DemoTools.cpp
# )

# find_package(Threads REQUIRED)   # Asio 用
# target_link_libraries(mcp_core PUBLIC Threads::Threads)

# target_include_directories(mcp_core PUBLIC include)
# target_link_libraries(mcp_core PUBLIC asio nlohmann_json::nlohmann_json inja)



# add_executable(mcp_promptd src/server_main.cpp)
# target_link_libraries(mcp_promptd PRIVATE mcp_core)

# add_library(mcp_client
#         src/client/McpClient.cpp
# )
# target_include_directories(mcp_client PUBLIC include)
# target_link_libraries(mcp_client PUBLIC asio nlohmann_json::nlohmann_json Threads::Threads)

# # Tests (optional) -----------------------------------------------
# enable_testing()
# #add_subdirectory(tests)