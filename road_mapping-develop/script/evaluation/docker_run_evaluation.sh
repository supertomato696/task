#!/bin/bash

echo "cur file: $0"
echo "input: $1"
echo "output: $2"
echo "run_evaluation: $3"


docker run -it --rm \
    -v $1:/app/data/input \
    -v $2:/app/data/output \
    10.0.117.68/byd_geoeva/geoeva_mapping_autolabeling:v2.0.4 python main.py


docker run -it --rm \
    -v $1:/app/data/input \
    -v $2:/app/data/output \
    data-management.tencentcloudcr.com/byd_geoeva/geoeva_mapping_autolabeling:v2.0.4 python main.py