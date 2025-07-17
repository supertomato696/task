rm -rf bin/
rm -rf build/
mkdir build
cd build
cmake ..
make -j8
cd ../bin

# task_ws_floder=/home/ccj/data/lukou_test

# ./pcdConvert $task_ws_floder/clip_001_20240816_095926/pointcloud_compound_old.pcd \
# $task_ws_floder/clip_001_20240816_095926/pointcloud_compound.pcd

# ./postLuKou $task_ws_floder/fsd_mapbuild_out \
# $task_ws_floder/clip_001_20240816_095926/pointcloud_compound.pcd \
# /home/ccj/code/map/raw/road_mapping/data/lidar_mapping_ws/fsd_mapbuild_out/fsd_mapbuild_task.json \
# /home/ccj/code/map/raw/road_mapping/data/lidar_mapping_ws/fsd_mapbuild_out/LuKou


# ./postRoadMark $task_ws_floder/fsd_mapbuild_out \
# $task_ws_floder/clip_001_20240816_095926/pointcloud_compound.pcd \
# /home/ccj/code/map/raw/road_mapping/data/lidar_mapping_ws/fsd_mapbuild_out/fsd_mapbuild_task.json \
# /home/ccj/code/map/raw/road_mapping/data/lidar_mapping_ws/fsd_mapbuild_out/RoadMarkRes


# ./uploadLane /home/ccj/code/map/raw/road_mapping/data/lidar_mapping_ws/fsd_mapbuild_out \
# /home/ccj/code/map/raw/road_mapping/data/lidar_mapping_ws/fsd_mapbuild_out/fsd_mapbuild_task.json \
# $task_ws_floder/clip_001_20240816_095926/pointcloud_compound.pcd