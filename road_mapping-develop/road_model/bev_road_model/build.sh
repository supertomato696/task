# !/bin/bash
cpu_num=$(nproc)
cmake_core=$(expr $cpu_num)



# if [ $1 = "clean" ];then
# echo  "build clean"
# fi

if [ ! -d "build" ]; then
    mkdir build
fi


pushd build
# eval "cmake .. -DCMAKE_BUILD_TYPE=Debug "
eval "cmake .. "
make -j ${cmake_core}
popd
