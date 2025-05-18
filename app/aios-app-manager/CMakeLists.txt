cmake_minimum_required(VERSION 3.10)
project(aios-app-manager)

add_library(${PROJECT_NAME} STATIC)
target_sources(${PROJECT_NAME} PRIVATE
        src/linux_app_process_manager.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)




target_link_libraries(${PROJECT_NAME} PRIVATE
        aios-app
)