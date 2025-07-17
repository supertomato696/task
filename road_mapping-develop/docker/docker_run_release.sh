#!/bin/sh

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

sudo docker run -it --rm \
    -v /mnt/c/02data/01zb_mapping/test/mnt:/data \
    roadmodel:$version \
    midwork \
    --workspace_folder="/data"

# docker run -it \
#             -v  /mnt/d/01_code/04_dataset/dilabel/crowd_source/new:/data \
#             --name run_exe \
#             roadmodel:v2 \
#             /bin/bash

