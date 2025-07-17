'''
Description: 
Author: yxx
Date: 2022-08-26 14:36:17
Reference: 
'''
import argparse
import copy
import datetime
import os
import sys
import time
import subprocess
import json
from shapely.wkt import dumps, loads
from multiprocessing import cpu_count, Pool
import shutil
import shutil
import tran_imgpos_coor

fsd_mapbuild_root_folder = os.path.dirname(os.path.abspath(__file__+"/../"))  # 顶层目录加入到python的环境变量中
fsd_root_folder= os.path.dirname(os.path.abspath(__file__+"/../../")) 
sys.path.append(fsd_root_folder)
from common.util.util import *

sys.path.append(os.path.join(fsd_mapbuild_root_folder, "semantic_road_model"))
from  run_label import *

lidar_mapping_root_folder = os.path.dirname(os.path.abspath(__file__+"/../../"))  # 顶层目录加入到python的环境变量中
print(("fsd_mapbuild_root_folder is {}").format(fsd_mapbuild_root_folder))

def run_semantic_road_model(args):
    workspace_folder = str(args.workspace_folder)
    task_id = str(args.task_id)
    road_branch = str(args.road_branch)
    info_json_path = str(args.info_json_path)
    process_mode=str(args.process_mode)
    upload_lane = "1" if process_mode == "0" else "0"
    label = "cloud_bev_label"
    # label = "both"
    data_type = str(args.data_type)
    error_code = run_all_label(workspace_folder,task_id,road_branch,info_json_path,label,upload_lane,data_type)
    print(error_code)
    return error_code

def run_bev_road_model(args):
    """
    Arguments:bev_自动建模
    ---------
    Returns
    -------
    """
    workspace_folder = str(args.workspace_folder)
    task_id = str(args.task_id)
    road_branch = str(args.road_branch)
    info_json_path = str(args.info_json_path)
    utm_num = int(args.utm_num)
    t_utm_world = json.loads(args.t_utm_world)
    process_mode = str(args.process_mode)
    task_dir = os.path.join(workspace_folder, task_id)
    data_dir = os.path.join(task_dir, "tile_download")
    polygon_path = str(args.polygon_json_path)
    groud_dir = task_dir + "/mapping_output/"
    label = "fsd_mapbuild_out" + "_" + "cloud_bev_label"
    
    config_file = "/conf/road_model_online.ini"
    if process_mode == "0":
        print("==========离散任务跳过bev建模===========")
        return 0
    if process_mode == "2":
        config_file = "/conf/road_model_local.ini"

    bev_root_folder = ""
    upload_path = ""
    if args.default_topo_version == 0:
        bev_root_folder = fsd_mapbuild_root_folder + "/../bev_road_model"
        upload_path = bev_root_folder + "/bin/bev_road_model"
    elif args.default_topo_version == 1:
        bev_root_folder = fsd_mapbuild_root_folder + "/../fast_road_model"
        upload_path = bev_root_folder + "/bin/fast_road_model"

    upload_path += " --flagfile=" + bev_root_folder + config_file
    upload_path += " --base_dir=" + data_dir
    upload_path += " --middle_json_path=" + info_json_path
    upload_path += " --road_branch=" + road_branch
    upload_path += " --crosspoint_file_dir=" + task_dir + "/fsd_mapbuild_out_cloud_bev_label/crosspoint.pcd"
    upload_path += " --debug_file_dir=" + task_dir + "/bev_mapbuild_out/debug_data"
    upload_path += " --shp_file_dir=" + task_dir + "/bev_mapbuild_out/export_to_shp"
    upload_path += " --origin_shp_file_dir=" + task_dir + "/bev_mapbuild_out/export_to_origin_shp"
    upload_path += " --mid_shp_file_dir=" + task_dir + "/bev_mapbuild_out/export_to_mid_shp"
    upload_path += " --base_log_dir=" + task_dir + "/bev_mapbuild_out/log"
    upload_path += " --utm_zone=" + str(utm_num)
    upload_path += " --utm_center_x=" + str(t_utm_world[0])
    upload_path += " --utm_center_y=" + str(t_utm_world[1])
    upload_path += " --utm_center_z=" + str(t_utm_world[2])
    upload_path += " --polygon_custom_file=" + polygon_path
    upload_path += " --mapping_line_dir=" + label
    upload_path += " --mapping_line_dir_boundary=" + label    
    upload_path += " --mapping_object_dir=" + label
    cmd="\033[32m 建模运行开始 \033[0m\n "+upload_path
    print(cmd)
    # print("\033[33m write bev road model cmd to:"+task_dir+"\033[0m")
    # with open(task_dir+"/bev_mapbuild_out/run.sh", 'w', encoding='utf-8') as file:
    #      file.write(upload_path)

    if os.path.exists(task_dir + "/bev_mapbuild_out/export_to_origin_shp"):
        shutil.rmtree(task_dir + "/bev_mapbuild_out/export_to_origin_shp", ignore_errors=True)
    if os.path.exists(task_dir + "/bev_mapbuild_out/export_to_shp"):
        shutil.rmtree(task_dir + "/bev_mapbuild_out/export_to_shp", ignore_errors=True)
    if os.path.exists(task_dir + "/bev_mapbuild_out/export_to_mid_shp"):
        shutil.rmtree(task_dir + "/bev_mapbuild_out/export_to_mid_shp", ignore_errors=True)
    return os.system(upload_path)

def run_mano_link(args):
    workspace_folder = str(args.workspace_folder)
    task_id = str(args.task_id)
    road_branch = str(args.road_branch)
    info_json_path = str(args.info_json_path)
    utm_num = int(args.utm_num)
    t_utm_world = json.loads(args.t_utm_world)
    process_mode = str(args.process_mode)
    upload_lane = "1" if process_mode == "0" else "0"
    # upload_lane = "3"
    task_dir = os.path.join(workspace_folder, task_id)
    
    global_pcd_path = os.path.join(task_dir, "mapping_output/global_map_ground.pcd")
    if upload_lane == "0":
        task_arg = json_file_to_dict(os.path.join(task_dir, "data_meta.json"))
        for perline in task_arg["link_dict"]:
            global_pcd_path = os.path.join(task_dir, "mapping_output/global_map_ground.pcd")
            monolink_path = os.path.join(task_dir, "monolink")
            add_folder_if_no_exist(monolink_path)
            monolink_pcd_path = monolink_path + "/" + str(perline) + ".pcd"
            print(monolink_pcd_path)
            cp_cmd = "cp " + global_pcd_path + " " + monolink_pcd_path
            print(perline)
            os.system(cp_cmd)

    if upload_lane == "1":
        #util.write_lines_to_file_override(os.path.join(task_dir, "debug/current_stage.txt"), ["单link建图"])
        cmd = "{} --task_folder={} --info_json_path={}".format(
            os.path.join(lidar_mapping_root_folder, "mapping/simple_mapper/bin/mono_link_mapping"),
            task_dir,
            info_json_path)
        ret = run_a_process(cmd, 3600*3)
        if ret is not None:
            return 701  # 超时退出
        
    # if upload_lane == "3":
    #     task_arg = json_file_to_dict(os.path.join(task_dir, "data_meta.json"));
    #     for perline in task_arg["link_dict"]:
    #         global_pcd_path = os.path.join(task_dir, "mapping_output/global_map_ground.pcd")
    #         monolink_path = os.path.join(task_dir, "monolink")
    #         add_folder_if_no_exist(monolink_path)
    #         monolink_pcd_path = monolink_path + "/" + str(perline) + ".pcd"
    #         print(monolink_pcd_path)
    #         cp_cmd = "cp " + global_pcd_path + " " + monolink_pcd_path
    #         print(perline)
    #         os.system(cp_cmd)


    return 0

def run_trans_imgpos(args) -> int:
    task_dir = os.path.join(args.workspace_folder, args.task_id)
    data_dir = os.path.join(task_dir, "tile_download")
    opt_dir = os.path.join(task_dir, "mapping_output/veh_pose")
    print("img_pos path {} {}".format(task_dir, opt_dir))
    return tran_imgpos_coor.trans_imgpos(data_dir, opt_dir)


parser = argparse.ArgumentParser()
parser.add_argument('--workspace_folder', type=str, default=os.environ['HOME'] + "/lidar_mapping_ws")
parser.add_argument('--task_id', type=str, default="27544")
parser.add_argument('--road_branch', type=str, default='yxx2')
parser.add_argument('--info_json_path', type=str, default="/home/test/lidar_mapping_ws/27544/task_info.json")
parser.add_argument('--polygon_json_path', type=str, default="polygon.txt")
parser.add_argument('--utm_num', type=int, default=51)
parser.add_argument('--t_utm_world', type=str, default='[321554,3418826,0]')
# 0: 离散  1：众包 2：评测、线下
parser.add_argument('--process_mode', type=str, default='0')
parser.add_argument('--global_pcd_path', type=str, default="")
parser.add_argument('--run_stage', type=int, default=0)
parser.add_argument('--default_topo_version', type=int, default=0, help="topo 版本，临时")
parser.add_argument('--data_type', type=str, default="", help="数据类型")

if __name__ == "__main__":
    args = parser.parse_args()
    error_code = run_trans_imgpos(args)
    if error_code != 0:
        print("=======================img_pos_gcj生成失败==========================")
        exit(error_code)

    #  error_code = run_mano_link(args)
    #  if error_code != 0:
    #      print("=======================run_mano_link生成失败==========================")
    #      exit(error_code)


    task_dir = os.path.join(args.workspace_folder, args.task_id)
    
    if args.run_stage == 0 or args.run_stage == 2:
        del_folder_if_exist(os.path.join(task_dir, "fsd_mapbuild_out"))
        del_folder_if_exist(os.path.join(task_dir, "fsd_mapbuild_out_cloud_line_seg"))
        del_folder_if_exist(os.path.join(task_dir, "fsd_mapbuild_out_cloud_pano_seg"))
        del_folder_if_exist(os.path.join(task_dir, "fsd_mapbuild_out_cloud_bev_label"))
        
        error_code = run_semantic_road_model(args)
        if error_code != 0:
            print("=======================语义建模失败==========================")
            exit(error_code)

    if args.run_stage == 0 or args.run_stage == 3:
        error_code = run_bev_road_model(args)
        if error_code != 0:
            print("=======================bev建模失败==========================")
            exit(error_code)
    #return error_code