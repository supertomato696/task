task_floder=/home/ccj/data/lidar_mapping_ws
task_id=23675
road_branch=yxx_test
label=cloud_pano_seg
info_json_path=$task_floder/$task_id/task_info.json

cd roadMapping
./run.sh
cd bin
./pcdConvert /home/ccj/data/test/clip_001_20240816_095926/pointcloud_compound.pcd \
$task_floder/$task_id/model_pcd/global_cloud.pcd

cd ../..
python3 json_to_json.py /home/ccj/code/map/raw/road_mapping/data/task_info_demo.json /home/ccj/data/test/clip_001_20240816_095926/sd_link-clip_001_20240816_095926.geojson \
$task_floder/$task_id/task_info.json
python3 run_label.py --workspace_folder=$task_floder --task_id=$task_id --road_branch=$road_branch --info_json_path=$info_json_path --label=$label
