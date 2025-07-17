#!/bin/bash

echo "请检查 version.md 是否已经更改版本, 请输出（是: y, 否: n):"
read -r has_change_version

# 1 修改版本
if [[ "$has_change_version" == "y" ]]; then
    echo "✔️ 已更改版本， 开发打包代码...."
    ./version.sh
else
    echo "❌ ！！！请先修改 version.md 后再进行打包代码，否则旧的评估结果可能被会覆盖！！！"
    exit 0
fi

# 2 编译代码
# ./build_qzc.sh

# 3 打包代码
python3 package_auto_model.py

LATEST_VERSION=$(grep "v3\." version.md | tail -n 1)  #取倒数第二行  #如V1.0.28 2024-10-10
# 获取版本号的主要、次要和修补版本
MAJOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 1 | tr -d 'v')
MINOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 2)
PATCH_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 3 | cut -d ' ' -f 1)
# echo "${MAJOR_VERSION}"
# echo "${MINOR_VERSION}"
# echo "${PATCH_VERSION}"

export version="v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}"
echo "$version"

echo "============================================"
echo "          打包 docker release 镜像"
echo "============================================"
echo "============================================"
echo "               构建运行环境"
echo "============================================"
# ./docker_build_dev.sh
echo "============================================"
echo "                拷贝程序文件"
echo "============================================"
sudo docker build -f ./docker/Dockerfile_release . -t roadmodel:$version  --no-cache
echo "============================================"
echo "        完成 docker release 镜像打包"
echo "============================================"
