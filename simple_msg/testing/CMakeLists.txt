project(simplemsg_testcase LANGUAGES CXX )

file(GLOB_RECURSE SOURCES *.cpp)
add_executable(simplemsg_testcase ${SOURCES})
target_link_libraries(simplemsg_testcase PRIVATE Catch2WithMain)
target_include_directories(simplemsg_testcase PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)