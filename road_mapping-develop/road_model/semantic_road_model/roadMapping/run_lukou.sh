rm -rf bin/
rm -rf build/
mkdir build
cd build
cmake ..
make -j24
cd ../bin
task_id=29443
task_ws_floder=/home/lenovo/data/lidar_mapping_ws/$task_id
./postLuKou $task_ws_floder/fsd_mapbuild_out_cloud_pano_seg/ $task_ws_floder/model_pcd/global_cloud.pcd \
$task_ws_floder/fsd_mapbuild_out_cloud_pano_seg/fsd_mapbuild_task.json $task_ws_floder/fsd_mapbuild_out_cloud_pano_seg/LuKou
