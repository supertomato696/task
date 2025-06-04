cmake_minimum_required(VERSION 3.20)
project(demo LANGUAGES CXX)

# 继承父级编译标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## 包含路径（相对路径调整）
#target_include_directories(${PROJECT_NAME}} INTERFACE
#        ${CMAKE_CURRENT_SOURCE_DIR}/include
#        ${CMAKE_CURRENT_SOURCE_DIR}/../include          # 原父级 include
#        ${CMAKE_CURRENT_SOURCE_DIR}/../../aios-app      # 原 aios-app 路径
#)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
#                        ${CMAKE_CURRENT_SOURCE_DIR}/../include          # 原父级 include

)

# 定义批量添加 demo 的函数
function(add_demo name)
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name}
            PRIVATE
            aios-app-manager
            pthread
    )
#    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../aios-app)
    target_compile_definitions(${name} PRIVATE ASIO_STANDALONE)
endfunction()

# 批量添加所有 demo
add_demo(demo_sleep)
add_demo(demo_env)
add_demo(demo_stop)
add_demo(demo_fail)
add_demo(demo_concurrent)
add_demo(demo_mass_concurrent)
add_demo(demo_sigkill)
add_demo(demo_signal)
add_demo(demo_multi_thread)
add_demo(demo_duplicate)



# test use for start
add_executable(signalHandler ${CMAKE_CURRENT_SOURCE_DIR}/../bin/signalHanldler.cpp)
target_include_directories(signalHandler PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include )


# 指定目标目录
set(CONFIG_DEST_DIR ${CMAKE_BINARY_DIR}/src/aios-process-manager/demo/environment-file)

# 创建目标目录
file(MAKE_DIRECTORY ${CONFIG_DEST_DIR})

# 复制配置文件到目标目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/environment-file/environment-config DESTINATION ${CONFIG_DEST_DIR})


message(STATUS "Operating System: ${CMAKE_SYSTEM}")
message(STATUS "CMake version: ${CMAKE_VERSION}")

message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "Use system Asio:      ${USE_SYSTEM_ASIO}")
message(STATUS "Use system GTest:     ${USE_SYSTEM_GTEST}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")


#CMake 的嵌套变量解析规则要求：
#
#第一层 ${var} 是为了获取变量的名称。
#第二层 ${${var}} 是为了根据变量名称动态获取其对应的值。
# 打印所有变量
get_cmake_property(all_vars VARIABLES)
foreach(var ${all_vars})
    message(STATUS "${var} = ${${var}}")
endforeach()



# 设置安装子目录  install CMAKE_INSTALL_PREFIX/
set(CMAKE_INSTALL_BINDIR "executables")
set(CMAKE_INSTALL_LIBDIR "libraries")
set(CMAKE_INSTALL_INCLUDEDIR "headers")
set(CMAKE_INSTALL_DATADIR "data")


# DESTINATION
#含义: 指定安装路径。
#格式: 可以是相对路径或绝对路径，通常是相对路径，最终会被安装到 CMAKE_INSTALL_PREFIX 指定的根目录下。
# 安装规则
#install(TARGETS MyLibrary MyExecutable
#        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
#
#install(FILES include/MyLibrary.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(FILES data/config.txt DESTINATION ${CMAKE_INSTALL_DATADIR})
#install(DIRECTORY include/ DESTINATION include)

