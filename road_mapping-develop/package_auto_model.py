import os, sys
import shutil

def copy_file(src_file: str, dst_file: str):
    if not os.path.exists(src_file):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    # if not os.path.exists(os.path.dirname(dst_file)):
    #     # shutil.rmtree(dst_dir, ignore_errors=True)
    #     os.makedirs(os.path.dirname(dst_file))

    shutil.copyfile(src_file, dst_file)

def copy_file_with_metadata(src_file: str, dst_file: str):
    if not os.path.exists(src_file):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    # if not os.path.exists(os.path.dirname(dst_file)):
    #     # shutil.rmtree(dst_dir, ignore_errors=True)
    #     os.makedirs(os.path.dirname(dst_file))

    shutil.copy2(src_file, dst_file)

def copy_folder(src_dir: str, dst_dir: str):
    if not os.path.exists(src_dir):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, ignore_errors=True)
    shutil.copytree(src_dir, dst_dir)


def add_folder_if_no_exist(folder_path: str):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def del_file_if_exist(file_path: str):
    if not os.path.exists(file_path):
        return
    os.remove(file_path)

def rm_folder(src_dir: str):
    if not os.path.exists(src_dir):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return

    shutil.rmtree(src_dir, ignore_errors=True)

if __name__ == "__main__":
    package_dir = "output"
    src_files = [
        # 顶层 script
        "script/road_model_pipeline_multi_batch_qzc.py",
        "script/road_model_pipeline_multi_qzc.py",
        "script/csv_to_info_json.py",
        "script/zip_file.sh",
        "script/unzip_file.sh",
        "script/evaluation/docker_run_evaluation.sh",
        "script/evaluation/change_evaluate_version.sh",
        "data/task_info_demo.json",
        "data/evaluate_config.yaml",
        "data/evaluate_config_v2.yaml",
        
        # common 脚本
        "common/data/error_code.py",
        "common/data/error_code.py",
        "common/util/log.py",
        "common/util/util.py",

        # data_io
        "data_io/script/util.py",
        "data_io/script/bev_label_map.py",
        "data_io/script/bev_converter.py",
        "data_io/script/data_download_qzc.py",

        # preprocess 
        "preprocess/script/data_preprocess_qzc.py",
        "preprocess/script/data_set_maker_qzc.py",
        "preprocess/script/bev_label_vis.py",
        "preprocess/script/trail_data.py",
        "preprocess/script/util.py",

        # mapping
        # lidarmapping
        "mapping/pointcloudconvert/bin/pointcloudconvert",
        "mapping/pointcloudconvert/wgs84_to_mars/lib/x86/libkcoords_plugin.so",
        "mapping/pointcloudconvert/wgs84_to_mars/include/Coord.h",
        "mapping/pointcloudconvert/wgs84_to_mars/include/util.h",

        # bevmapping
        "mapping/script/run_bev_mapping.py",
        "mapping/script/bev_line_track.py",
        "mapping/script/trail_data.py",
        "mapping/script/util.py",
        
        "preprocess/pointcloudrender/bin/flat_scan_maker",
        "mapping/simple_mapper/bin/flat_mapper",

        # run_mapbuild
        "road_model/script/run.py",
        "road_model/semantic_road_model/run_label.py",
        "road_model/semantic_road_model/scripts/process_link_node.py",
        "road_model/semantic_road_model/roadMapping/bin/processPerception",
        "road_model/semantic_road_model/roadMapping/bin/combinePcdFiles",
        "road_model/semantic_road_model/roadMapping/bin/pclcloudprocess",
        "road_model/semantic_road_model/roadMapping/bin/postRoadModel",
        "road_model/semantic_road_model/roadMapping/bin/roadBoundaryModel",
        "road_model/semantic_road_model/roadMapping/bin/joinKeyPoints",
        "road_model/semantic_road_model/roadMapping/bin/json_to_obj",
        "road_model/semantic_road_model/roadMapping/bin/postRoadMark",
        "road_model/semantic_road_model/roadMapping/bin/postLuKou",
        "road_model/semantic_road_model/roadMapping/bin/uploadLane",
        "road_model/semantic_road_model/roadMapping/bin/processSiteBoundary",

        # traffic
        "road_model/process_air/process_tl_merging.py",
        "road_model/process_air/process_tl_topology.py",
        "road_model/process_air/process_tl_tracking.py",
        "road_model/process_air/process_trafficlight.py",
        "road_model/process_air/util.py",
        "road_model/version/version.h",

        "road_model/bev_road_model/bin/bev_road_model",
        "road_model/fast_road_model/bin/fast_road_model",
        "road_model/script/tran_imgpos_coor.py",
        "road_model/bev_road_model/conf/road_model_local.ini",

        # "road_model/bev_road_model/thirdparty/proj4/libproj.so.13",
        # "road_model/bev_road_model/thirdparty/proj4/libproj.so.17",
        "road_model/bev_road_model/thirdparty/lib/libEngine_Algorithm.so",
        "road_model/bev_road_model/thirdparty/lib/libEngine_Base.so",
        "road_model/bev_road_model/thirdparty/lib/libEngine_Geometries.so",

        # fast_road_model
        "road_model/fast_road_model/bin/fast_road_model",
        "road_model/fast_road_model/conf/road_model_local.ini",
        "road_model/fast_road_model/thirdparty/lib/libEngine_Algorithm.so",
        "road_model/fast_road_model/thirdparty/lib/libEngine_Base.so",
        "road_model/fast_road_model/thirdparty/lib/libEngine_Geometries.so",

        # version
        "road_model/bev_road_model/version/version.h",
        "version.md",
    ]

    rm_folder(package_dir)
    
    for src_file in src_files:
        dst_file = os.path.join(package_dir, src_file)
        add_folder_if_no_exist(os.path.dirname(dst_file))
        copy_file_with_metadata(src_file, dst_file)

    print()
