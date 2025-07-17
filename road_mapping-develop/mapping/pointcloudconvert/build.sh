#!/bin/bash
cpu_num=$(nproc)
cmake_core=`expr $cpu_num / 2`
release=on
while getopts ":j:r:" opt
do
    case $opt in
        j) cmake_core="$OPTARG" ;;
        r) release="$OPTARG" ;;
        ?)
        echo "error"
        exit 1;;
    esac
done

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
rm CMakeCache.txt
BUILD_CMD="cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

eval ${BUILD_CMD}
make -j ${cmake_core}
popd
