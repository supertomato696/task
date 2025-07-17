#!/bin/bash
#cpu_num=$(nproc)
#cmake_core=$(expr $cpu_num / 2)

if [ ! -d "build" ]; then
    mkdir build
fi

# if [ -d "build" ]; then
#     rm -rf build
# fi
# if [ -d "bin" ]; then
#     rm -rf bin
# fi
# mkdir build

pushd build
# eval "cmake .. -DCMAKE_BUILD_TYPE=debug"
eval "cmake .."
#make -j ${cmake_core}
make -j24
popd
