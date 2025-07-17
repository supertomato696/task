#!/bin/sh

echo "============================================"
echo "          提交 docker release 镜像"
echo "============================================"

LATEST_VERSION=$(grep "v3\." version.md | tail -n 1)  #取倒数第二行  #如V1.0.28 2024-10-10
# 获取版本号的主要、次要和修补版本
MAJOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 1 | tr -d 'v')
MINOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 2)
PATCH_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 3 | cut -d ' ' -f 1)
# echo "${MAJOR_VERSION}"
# echo "${MINOR_VERSION}"
# echo "${PATCH_VERSION}"

local_version="v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}"
export version="V3.9.0_20250715_mapping_autolabeling_v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}"
echo "$version"
# export version=$(head -n +1 CHANGELOG.md)
sudo docker tag roadmodel:$local_version 10.0.117.68/byd_roadmodel/roadmodel:$version
sudo docker push 10.0.117.68/byd_roadmodel/roadmodel:$version

echo "docker release 版本号: ${version}"
echo "查看提交结果: http://10.0.117.68/harbor/projects/35/repositories"


sudo docker tag roadmodel:$local_version map-caas.denzacloud.com/byd_roadmodel/roadmodel:$version
sudo docker push map-caas.denzacloud.com/byd_roadmodel/roadmodel:$version
echo "docker release 专线版本号: ${version}"
# echo "查看提交结果: http://10.0.117.68/harbor/projects/35/repositories"

echo "============================================"
echo "        完成 docker release 镜像提交"
echo "============================================"
