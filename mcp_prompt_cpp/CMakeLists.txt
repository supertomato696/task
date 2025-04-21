cmake_minimum_required(VERSION 3.20)
project(mcp_prompt_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dependencies -----------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
        asio
        GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
        GIT_TAG        asio-1-28-0
)
FetchContent_MakeAvailable(asio)

FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.2
)
FetchContent_MakeAvailable(nlohmann_json)



FetchContent_Declare(
        inja
        GIT_REPOSITORY https://github.com/pantor/inja.git
        GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(inja)


FetchContent_Declare(
        sqlite_modern_cpp
        GIT_REPOSITORY https://github.com/SqliteModernCpp/sqlite_modern_cpp.git
        GIT_TAG        v3.2
)
FetchContent_MakeAvailable(sqlite_modern_cpp)

find_package(SQLite3 REQUIRED)

target_link_libraries(mcp_core PUBLIC
        sqlite_modern_cpp
        SQLite::SQLite3
)

FetchContent_Declare(
        httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG        v0.15.3
)
FetchContent_MakeAvailable(httplib)

target_link_libraries(mcp_core PUBLIC httplib::httplib)

# Targets ---------------------------------------------------------------
add_library(mcp_core
        src/transport/AsioTcpTransport.cpp
#        src/protocol/JsonRpc.cpp
#        src/protocol/McpTypes.cpp
#        src/prompt/TemplateEngine.cpp
#        src/prompt/PromptStore.cpp
        src/prompt/MultiModalAssembler.cpp
        src/prompt/PromptService.cpp
#        src/resource/ResourceManager.cpp
#        src/resource/resolvers/LocalFileResolver.cpp
#        src/resource/resolvers/HttpResolver.cpp
)

find_package(Threads REQUIRED)   # Asio 用
target_link_libraries(mcp_core PUBLIC Threads::Threads)

target_include_directories(mcp_core PUBLIC include)
target_link_libraries(mcp_core PUBLIC asio nlohmann_json::nlohmann_json inja)

add_executable(mcp_promptd src/server_main.cpp)
target_link_libraries(mcp_promptd PRIVATE mcp_core)

add_library(mcp_client
        src/client/McpClient.cpp
)
target_include_directories(mcp_client PUBLIC include)
target_link_libraries(mcp_client PUBLIC asio nlohmann_json::nlohmann_json Threads::Threads)

# Tests (optional) -----------------------------------------------
enable_testing()
#add_subdirectory(tests)