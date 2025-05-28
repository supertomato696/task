cmake_minimum_required(VERSION 3.20)
project(aios-app-manager LANGUAGES CXX)

# -------------------------------------------------------------
# 全局编译选项
# -------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 可以在命令行 -DUSE_SYSTEM_ASIO=ON / -DUSE_SYSTEM_GTEST=ON
option(USE_SYSTEM_ASIO   "Use system-installed Asio" OFF)
option(USE_SYSTEM_GTEST  "Use system-installed GTest" OFF)

# -------------------------------------------------------------
# 依赖：Asio (header-only)
# -------------------------------------------------------------
if (USE_SYSTEM_ASIO)
    find_path(ASIO_INCLUDE_DIR asio.hpp)
    if (NOT ASIO_INCLUDE_DIR)
        message(FATAL_ERROR "Asio headers not found; set USE_SYSTEM_ASIO=OFF to fetch automatically")
    endif()
else()
    include(FetchContent)
    FetchContent_Declare(
        asio
        GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
        GIT_TAG        asio-1-30-2            # 任意较新的 tag
    )
    FetchContent_MakeAvailable(asio)
    set(ASIO_INCLUDE_DIR ${asio_SOURCE_DIR}/asio/include)
endif()

# -------------------------------------------------------------
# 依赖：GoogleTest
# -------------------------------------------------------------
if (BUILD_TESTING)
    if (USE_SYSTEM_GTEST)
        find_package(GTest REQUIRED)
    else()
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        release-1.14.0
        )
        # 为避免与用户项目的 -std=c++20 冲突，禁用 googletest 自己的安装脚本
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()
endif()

# -------------------------------------------------------------
# aios-app-manager 静态库
# -------------------------------------------------------------
add_library(aios-app-manager STATIC)
target_sources(aios-app-manager PRIVATE
    src/LinuxAppProcessManager.cpp
        src/SignalHandler.cpp
    # 如果你把 ProcessMonitor.cpp / Launcher.cpp 拆 cpp，请在此追加
)
target_include_directories(aios-app-manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ASIO_INCLUDE_DIR}
)
target_compile_definitions(aios-app-manager PUBLIC
    ASIO_STANDALONE           # 使用 header-only Asio
)
target_link_libraries(aios-app-manager
    PRIVATE
        pthread               # Asio on Linux 需要
)

# -------------------------------------------------------------
# ----------- DEMO 可执行文件 ------------------------------
# -------------------------------------------------------------
#function(add_demo name)
#    add_executable(${name} demo/${name}.cpp)
#    target_link_libraries(${name} PRIVATE aios-app-manager)
#    target_compile_definitions(${name} PRIVATE ASIO_STANDALONE)
#endfunction()
#
#add_demo(demo_sleep)
#add_demo(demo_stop)
#add_demo(demo_fail)
#add_demo(demo_concurrent)
#add_demo(demo_mass_concurrent)
#add_demo(demo_sigkill)
#add_demo(demo_signal)
#add_demo(demo_double_fork)
#add_demo(demo_duplicate)

# -------------------------------------------------------------
# -------------------- 单元测试 ------------------------------
# -------------------------------------------------------------
if (BUILD_TESTING)
    enable_testing()

    add_executable(unit_tests
        tests/test_process_manager.cpp
    )
    target_include_directories(unit_tests PRIVATE ${ASIO_INCLUDE_DIR})
    target_link_libraries(unit_tests
        PRIVATE
            aios-app-manager
            GTest::gtest_main
            pthread
    )
    target_compile_definitions(unit_tests PRIVATE ASIO_STANDALONE)

    include(GoogleTest)
    gtest_discover_tests(unit_tests)
endif()

# -------------------------------------------------------------
# 选项摘要
# -------------------------------------------------------------
message(STATUS "Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "Use system Asio:      ${USE_SYSTEM_ASIO}")
message(STATUS "Use system GTest:     ${USE_SYSTEM_GTEST}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")

# demo
#add_executable(demo_env demo/demo_env.cpp)
#target_link_libraries(demo_env PRIVATE aios-app-manager)
#target_compile_definitions(demo_env PRIVATE ASIO_STANDALONE)

# test
#target_sources(unit_tests PRIVATE tests/test_env_manager.cpp)


add_subdirectory(demo)

#///add_demo# 常规构建（自动拉 Asio & GTest）
#mkdir build && cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
#cmake --build . --config Release
#ctest --output-on-failure       # 运行单元测试
#./demo_sleep                    # 运行 demo
#
#
#cmake .. -DUSE_SYSTEM_ASIO=ON -DUSE_SYSTEM_GTEST=ON
