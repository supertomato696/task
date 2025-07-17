# CMakeList.txt: mapbuilding 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# 如果支持，请为 MSVC 编译器启用热重载。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("mapbuilding")
include_directories(${CMAKE_SOURCE_DIR}/3rdlib/eigen-3.3.9)
include_directories(${CMAKE_SOURCE_DIR}/3rdlib/geos-3.11.4/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdlib/gdal-2.4.4/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdlib/proj-4.9.2/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdlib/spatialindex-2.0.0/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdlib/json-c/include)
link_directories(${CMAKE_SOURCE_DIR}/3rdlib/geos-3.11.4/lib)
link_directories(${CMAKE_SOURCE_DIR}/3rdlib/gdal-2.4.4/lib)
link_directories(${CMAKE_SOURCE_DIR}/3rdlib/proj-4.9.2/lib)
link_directories(${CMAKE_SOURCE_DIR}/3rdlib/spatialindex-2.0.0/lib)
link_directories(${CMAKE_SOURCE_DIR}/3rdlib/json-c/lib)

# 将源代码添加到此项目的可执行文件。
add_executable (mapbuilding "src/mapbuilding.cpp" "src/mapbuilding.h" "src/interpolation.cpp" "src/interpolation.h" "src/main.cpp" "src/polygonenumerator.h" "src/polygonenumerator.cpp" "src/logUtil.h" "src/logUtil.cpp")
target_link_libraries(mapbuilding geos geos_c proj_i proj gdal_i spatialindex_c-64 spatialindex-64 json-c)
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET mapbuilding PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
