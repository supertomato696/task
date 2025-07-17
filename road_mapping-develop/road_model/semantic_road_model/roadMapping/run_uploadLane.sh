rm -rf build
rm -rf bin 
mkdir build
mkdir bin
cd build

cmake ..
make -j24

cd ../bin
task_id=29443
./uploadLane /home/lenovo/data/lidar_mapping_ws/$task_id/fsd_mapbuild_out_cloud_pano_seg \
/home/lenovo/data/lidar_mapping_ws/$task_id/fsd_mapbuild_out_cloud_pano_seg/fsd_mapbuild_task.json \
/home/lenovo/data/lidar_mapping_ws/$task_id/model_pcd/global_cloud.pcd 1
