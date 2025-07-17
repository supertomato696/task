'''
Description: 
Author: biyadi1
Date: 2023-02-09 14:36:17
Reference: 
'''
import argparse
import copy
import datetime
import os
import time
import subprocess
import json
from shapely.wkt import dumps, loads
from multiprocessing import cpu_count, Pool
import shutil
import sys


fsd_mapbuild_root_folder = os.path.dirname(os.path.abspath(__file__))  # 顶层目录加入到python的环境变量中
print(("fsd_mapbuild_root_folder is {}").format(fsd_mapbuild_root_folder))

def run_bev_road_model(workspace_folder,task_id,utm_num,t_utm_world,road_branch,info_json_path)->int:
    """
    Arguments:所有link进行自动化建模
    ---------
    Returns
    -------
    """
    task_dir = os.path.join(workspace_folder, task_id)
    data_dir = os.path.join(task_dir, "tile_download")
    groud_dir = task_dir + "/mapping_output/"

    upload_path = fsd_mapbuild_root_folder + "/bin/fast_road_model"
    upload_path += " --flagfile=" + fsd_mapbuild_root_folder + "/conf/road_model_online.ini"
    upload_path += " --base_dir=" + data_dir
    upload_path += " --middle_json_path=" + info_json_path
    upload_path += " --road_branch=" + road_branch
    upload_path += " --debug_file_dir=" + task_dir + "/bev_mapbuild_out/debug_data"
    upload_path += " --base_log_dir=" + task_dir + "/bev_mapbuild_out/log"
    upload_path += " --utm_zone=" + str(utm_num)
    upload_path += " --utm_center_x=" + str(t_utm_world[0])
    upload_path += " --utm_center_y=" + str(t_utm_world[1])
    upload_path += " --utm_center_z=" + str(t_utm_world[2])
    print(upload_path)
    return os.system(upload_path)


parser = argparse.ArgumentParser()
parser.add_argument('--workspace_folder', type=str, default=os.environ['HOME'] + "/dataused")
parser.add_argument('--task_id', type=str, default="27515")
parser.add_argument('--road_branch', type=str, default='yxx2')
parser.add_argument('--info_json_path', type=str, default="/home/test/dataused/27515/task_info.json")
parser.add_argument('--utm_num', type=int, default=51)
parser.add_argument('--t_utm_world', type=str, default='[324704,3419015,0]')

if __name__ == "__main__":
    time_start = time.time()
    args = parser.parse_args()
    workspace_folder = str(args.workspace_folder)
    task_id = str(args.task_id)
    road_branch = str(args.road_branch)
    info_json_path = str(args.info_json_path)
    utm_num = int(args.utm_num)
    t_utm_world=json.loads(args.t_utm_world)
    error_code = run_bev_road_model(workspace_folder,task_id,utm_num,t_utm_world,road_branch,info_json_path)
    print(error_code)
    if  error_code == 0:
        result_json = os.path.join(workspace_folder, task_id,'bev_mapbuild_out','result.json')
        with open(result_json,'w',encoding='utf8')as fp:
           print(fp)
        fp.close()
