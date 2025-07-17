#!/bin/sh

echo "============================================"
echo "          提交 docker debug 镜像"
echo "============================================"

LATEST_VERSION=$(grep "v3\." version.md | tail -n 1)  #取倒数第二行  #如V1.0.28 2024-10-10
# 获取版本号的主要、次要和修补版本
MAJOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 1 | tr -d 'v')
MINOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 2)
PATCH_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 3 | cut -d ' ' -f 1)
# echo "${MAJOR_VERSION}"
# echo "${MINOR_VERSION}"
# echo "${PATCH_VERSION}"

version="v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}"
username=$(whoami)
current_time=$(date "+%m-%d_%H_%M_%S") 


version_tag="${version}_${username}_${current_time}"
echo "$version_tag"
sudo docker tag roadmodel:$version 10.0.117.68/byd_roadmodel_test/roadmodel:$version_tag
sudo docker push 10.0.117.68/byd_roadmodel_test/roadmodel:$version_tag
echo "推送的docker debug 版本号: ${version_tag}"
echo "查看提交结果: http://10.0.117.68/harbor/projects/36/repositories"


sudo docker tag roadmodel:$version map-caas.denzacloud.com/byd_roadmodel_test/roadmodel:$version_tag
sudo docker push map-caas.denzacloud.com/byd_roadmodel_test/roadmodel:$version_tag
echo "推送的docker debug 专线版本号: ${version_tag}"
# echo "查看提交结果: http://10.0.117.68/harbor/projects/36/repositories"


echo "============================================"
echo "        完成 docker debug 镜像提交"
echo "============================================"
