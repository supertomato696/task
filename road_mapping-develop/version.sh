#!/bin/bash

# 从 .md 获取最新版本号
# LATEST_VERSION=$(tail -n 2 version.md | head -n 1)  #取倒数第二行  #如V1.0.28 2024-10-10
LATEST_VERSION=$(grep "v3\." version.md | tail -n 1)  #取倒数第二行  #如V1.0.28 2024-10-10
# 获取版本号的主要、次要和修补版本
MAJOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 1 | tr -d 'v')
MINOR_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 2)
PATCH_VERSION=$(echo $LATEST_VERSION | cut -d '.' -f 3 | cut -d ' ' -f 1)


# 更新 Version.h
cat <<EOL > road_model/version/version.h
#ifndef _ROAD_MAPPING_VERSION_H__
#define _ROAD_MAPPING_VERSION_H__

#define ROAD_MAPPING_MAJOR_VERSION $MAJOR_VERSION
#define ROAD_MAPPING_MINOR_VERSION $MINOR_VERSION
#define ROAD_MAPPING_PATCH_VERSION $PATCH_VERSION

//版本号拼接
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define ROAD_MAPPING_VERSION_STRING TOSTRING(ROAD_MAPPING_MAJOR_VERSION) "." \
                                    TOSTRING(ROAD_MAPPING_MINOR_VERSION) "." \
                                    TOSTRING(ROAD_MAPPING_PATCH_VERSION)  

#endif
EOL


# 更新 script/road_model_pipeline_multi_qzc.py
sed -i "s/LATEST_VERSION=.*/LATEST_VERSION=\"$LATEST_VERSION\"/" script/road_model_pipeline_multi_qzc.py
sed -i 's/LATEST_VERSION="\(.*\) \(.*\)"/LATEST_VERSION="\1_\2"/' script/road_model_pipeline_multi_qzc.py  # V1.0.28 2024-10-10 改为V1.0.28_2024-10-10
sed -i "s/ROAD_MAPPING_TAG=.*/ROAD_MAPPING_TAG=\"v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}\"/" script/road_model_pipeline_multi_qzc.py
sed -i "s/ROAD_MAPPING_TAG=.*/ROAD_MAPPING_TAG=\"v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}\"/" script/road_model_pipeline_multi_batch_qzc.py

# 完成
echo "版本号更新完成:"
echo "Latest version: $LATEST_VERSION"
echo ""
echo "version.h:"
cat road_model/version/version.h | head -n 6
echo ""
echo "script/road_model_pipeline_multi_qzc.py:"
LATEST_VERSION=$(grep 'LATEST_VERSION=' script/road_model_pipeline_multi_qzc.py | cut -d'=' -f2 | tr -d ' "')
echo "Latest version: $LATEST_VERSION"
TAG=$(grep 'TAG=' script/road_model_pipeline_multi_qzc.py | cut -d'=' -f2 | tr -d ' "')
echo "TAG: $TAG"