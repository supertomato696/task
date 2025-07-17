
base_path=$1
out_put_path=$base_path/norm_temp
code_path=/home/cxf/01code/road_mapping_sum/test/road_mapping
# mkdir -p $out_put_path/output/log

rm -r $out_put_path/*

# gdb --args\
 $code_path/road_model/fast_road_model/bin/fast_road_model \
 --flagfile=$code_path/road_model/fast_road_model/conf/road_model_local.ini \
 --base_dir=$base_path/auto_label/output/tile_download \
 --middle_json_path=$base_path/auto_label/task_info_demo.json \
 --debug_file_dir=$out_put_path/output/data_debug_log \
 --shp_file_dir=$out_put_path/output/export_to_shp \
 --base_log_dir=$out_put_path/output/log \
 --mapping_line_dir=fsd_mapbuild_out_cloud_bev_label \
 --mapping_line_dir_boundary=fsd_mapbuild_out_cloud_bev_label \
 --mapping_object_dir=fsd_mapbuild_out_cloud_bev_label \
 --custom_utm_enable=false \
 --build_intersection_enable=false \
 --valid_enable=true\
 --valid_x=102\
 --valid_y=129\
 --valid_dis=80 \
 --sample_line_link_repair_theta=60 \
 --polygon_use_task=true \
 --use_image_debug=true \
#  --use_pcd_debug=true

# ./debug_cxf_local.sh /mnt/c/02data/01zb_mapping/0605_10lukou/lidar_byd/503576