<!-- ----------------道路建模模块环境搭建&程序运行------------------ -->
# 1.运行环境

  ubuntu 18.04或者20.04

# 2.安装编译依赖

  编译需要安装很多依赖库，安装流程在shell脚本中，可直接运行

  road_mapping/docker/setup.sh

  另外需要安装python库：

  pip install pyproj pymap3d opencv-python scipy requests requests_toolbelt geopandas pyransac3d open3d networkx psutil Pillow
  pip install --extra-index-url https://rospypi.github.io/simple/ rosbag sensor_msgs geometry_msgs

# 3.代码编译

  顶层目录下，编译脚本：build_qzc.sh

  或者进去模块目录下，进行单模块编译


# 4.运行配置

## 4.1 运行指南


编译后，理论上可以直接运行，顶层脚本，自动化处理所有流程

方式1： 直接运行

修改 road_model_pipeline_multi_batch_qzc.py 

1. 修改数据，这里可以是存放路口的文件夹（可以是单个路口的数据也可以是多个路口的数据）

```
default_workspace_folder = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross"
```

目录结构如下：

```
ten_cross
├── 11002424768485               <<<------- 路口名                                  ------->>>
│   ├── 10个路口的sdlink.geojson  <<<------- sdlink信息，没有的话，从上级目录拷贝到这里 ------->>>
│   ├── 10个路口的点位及范围.gpkg  <<<------- 路口信息，没有的话，从上级目录拷贝到这里   ------->>>
│   ├── 223126                   <<<------- 这个为运行过程中输出的结果路径            ------->>>
│   ├── PL0681_event_ld_split_merge_event_20240916-112447_0        <<<------- 包名  ------->>>
│   ├── PL6720_event_trafficlight_status_event_20240928-113418_0
├── 11002458078950
│   ├── 10个路口的sdlink.geojson
│   ├── 10个路口的点位及范围.gpkg
│   ├── 223126
│   ├── PL3073_event_trafficlight_status_event_20240903-175820_0
│   ├── PL0169_event_trafficlight_status_event_20240904-164033_0
```

2. 修改要运行的路 以及 路口里面的哪几个clip，不需要运行的注释掉就行 （注意：如果填了路口名字，但是 包 名为空，那默认运行里面所有的包）

```
lukou_ids = {
    ############ 氢 ############ 
    # "11002424768485": [   <<<------- 路口名 ------->>>
    #     # 双向直行（左右前后）
    #     'PL0681_event_ld_split_merge_event_20240916-112447_0', <<<------- 包名 ------->>>
    #     ## 左右转
    #     'PL6720_event_trafficlight_status_event_20240928-113418_0',
    #     # 掉头
    # ],
    ############ 氦 ############
    "11002458078950": [
        # 双向直行
        'PL3073_event_trafficlight_status_event_20240903-175820_0',
        # 左右转
        'PL0169_event_trafficlight_status_event_20240904-164033_0',
    ],
    ......
}
```

3. 运行：

```
python script/road_model_pipeline_multi_batch_qzc.py --run_stage=0
```

说明：

```
workspace_folder ： 数据路径根目录

run_stage ： 为了靠后的模块方便调试，可以分阶段运行
  - 0: 所有模块都运行
  - 1：拼图模块
  - 2：矢量化
  - 3：拓扑

ps： 第一次运行至少要设置为 --run_stage=0
```


方式2：编译后，建议分三步骤 或者 更多的流程进行处理，这里分为三个部分

```
1. 根据数据集，生成 global_cloud.pcd （带label的点云）

2. BEV_road_model

3. semantic_road_model
```

## 4.2 根据数据集，生成 global_cloud.pcd （带label的点云）

数据路径：

```
\\sjnas01\xjsy\16-XJSYGY\66-内部数据资料中转\crowd_source\to_qzc\xxx.zip
```


## 4.3 bev_road_model

### 4.3.1 运行 mmt test 数据：

  


## 4.4 运行 semantic_road_model



# 5.运行

  执行 road_mapping/road_model/bev_road_model/run.sh 

# 6.执行的中间可视化结果需要使用cloudcompare工具打开pcd点云文件查看

  中间可视化结果目录：road_mapping/road_model/bev_road_model/data_debug_log
