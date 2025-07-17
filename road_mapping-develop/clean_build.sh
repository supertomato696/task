#!/bin/bash
cpu_num=$(nproc)
cmake_core=`expr $cpu_num / 2`
while getopts ":j:" opt
do
    case $opt in
        j) cmake_core="$OPTARG" ;;
        ?)
        echo "error"
        exit 1;;
    esac
done
echo "============================================"
echo "               清除现存程序文件"
echo "============================================"
rm -rf build
rm -rf bin
rm -rf common/remap_lane/bin
rm -rf common/remap_lane/lib
rm -rf lisam/build
rm -rf lisam/bin
rm -rf DataCheckAuto/build
rm -rf DataCheckAuto/bin
rm -rf DataCheckAuto/lib
rm -rf fsdmap_location/build
rm -rf fsdmap_location/install
rm -rf location/lidar_reloc/bin
rm -rf location/lidar_reloc/lib
rm -rf location/lidar_reloc/build
rm -rf location/visual_reloc/build
rm -rf location/visual_reloc/install

pushd data_io
bash ./clean_data_io.sh
popd

pushd preprocess
bash ./clean_preprocess.sh
popd

pushd mapping
bash ./clean_mapping.sh
popd

pushd road_model
bash ./clean_road_model.sh
popd

pushd DataCheckAuto
bash ./clean_datacheckauto.sh
popd

pushd zl_lk2_road_model
bash ./clean_road_model.sh
popd

echo "============================================"
echo "          调用 build.sh 重新编译程序"
echo "============================================"
bash ./build.sh -j ${cmake_core}
