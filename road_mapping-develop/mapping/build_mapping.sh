#!/bin/bash
cpu_num=$(nproc)
cmake_core=$(expr $cpu_num / 2)
release=on
while getopts ":j:r:" opt; do
    case $opt in
    j) cmake_core="$OPTARG" ;;
    r) release="$OPTARG" ;;
    ?)
        echo "error"
        exit 1
        ;;
    esac
done
echo -e "\033[33m cpu core used to release: ${cmake_core} \033[0m"

function echoTitle() {
    echo -e "\n\033[41;33m    $@    \033[0m\n"
}

echoTitle "mapping 开始编译"
# ============================================ #
echoTitle "编译 mapping-simple_mapper"
pushd simple_mapper
bash ./build.sh -j ${cmake_core}
popd

# ============================================ #
echoTitle "编译 mapping-cloud_regis"
pushd cloud_regis
bash ./build.sh -j ${cmake_core}
popd

# ============================================ #
echoTitle "编译 mapping-BALM"
pushd BALM
./build_balm.sh
popd

# ============================================ #
echoTitle "编译 mapping-dataprocess"
pushd dataprocess
bash ./build.sh -j ${cmake_core}
popd

# ============================================ #
echoTitle "编译 ba"
pushd ba
bash ./build.sh
popd

# ============================================ #
echoTitle "编译 autocheck"
pushd autocheck
bash ./build.sh
popd

echoTitle "mapping 完成编译"

