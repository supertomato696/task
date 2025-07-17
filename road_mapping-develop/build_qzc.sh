#!/bin/bash
cpu_num=$(nproc)
cmake_core=$(expr $cpu_num / 2)
while getopts ":j:" opt; do
    case $opt in
    j) cmake_core="$OPTARG" ;;
    ?)
        echo "error"
        exit 1
        ;;
    esac
done
echo -e "\033[33m cpu core used to release: ${cmake_core} \033[0m"

# echo "============================================"
# echo "                编译 common"
# echo "============================================"
# pushd common
# bash ./build_common.sh -j${cmake_core}
# popd

# echo "============================================"
# echo "                编译 data_io"
# echo "============================================"
# cd data_io
# ./build_data_io.sh
# cd ..

echo "============================================"
echo "                编译 preprocess"
echo "============================================"
cd preprocess
./build_preprocess.sh
cd ..

# echo "============================================"
# echo "                编译 lisam"
# echo "============================================"
# cd lisam
# ./scripts/build.sh
# cd ..

echo "============================================"
echo "                编译 mapping"
echo "============================================"
pushd mapping
bash ./build_mapping_qzc.sh -j${cmake_core}
popd

echo "============================================"
echo "                编译 road model"
echo "============================================"
pushd road_model
bash ./build_road_model.sh
popd

# echo "============================================"
# echo "                编译 zl lk2 road model"
# echo "============================================"
# pushd zl_lk2_road_model
# bash ./build_road_model.sh
# popd

# echo "============================================"
# echo "                编译 location"
# echo "============================================"
# pushd location
# bash ./build.sh -j${cmake_core}
# popd


# echo "============================================"
# echo "                编译 DataCheckAuto"
# echo "============================================"
# pushd DataCheckAuto
# bash ./build_datacheckauto.sh
# popd

echo "============================================"
echo "                程序编译结束"
echo "============================================"
