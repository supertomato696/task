project(aios-app CXX)

add_library(${PROJECT_NAME} STATIC)

target_link_libraries(${PROJECT_NAME} asio)



target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})

#install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/bos/framework/base_error_code.h DESTINATION include/bos/framework)