# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-src")
  file(MAKE_DIRECTORY "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-src")
endif()
file(MAKE_DIRECTORY
  "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-build"
  "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix"
  "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix/tmp"
  "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix/src/inja-populate-stamp"
  "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix/src"
  "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix/src/inja-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix/src/inja-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/tanbowen/Desktop/mcp_prompt_cpp/build/_deps/inja-subbuild/inja-populate-prefix/src/inja-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
