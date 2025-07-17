#!/bin/sh

#ldconfig

echo `pwd`
echo `ls`
if [ $1 = 'midwork' ]; then
    echo "预设指令 midwork，启动中业 scrip/road_model_pipeline_multi_qzc.py 脚本:"
    shift
    echo "====================================="
    echo python3 script/road_model_pipeline_multi_qzc.py "$@"
    echo "====================================="
    python3 script/road_model_pipeline_multi_qzc.py "$@"
elif [ $1 = 'debug' ]; then
    echo "预设指令 debug，启动中业 scrip/road_model_pipeline_multi_batch_qzc.py 脚本:"
    shift
    echo "====================================="
    echo python3 script/road_model_pipeline_multi_batch_qzc.py "$@"
    echo "====================================="
    python3 script/road_model_pipeline_multi_batch_qzc.py "$@"

else  # 非预设指令，则直接执行
    echo "==============未识别的指令============"
    echo $@
    echo "====================================="
    $@
fi
