'''
Description:
Author: qzc
Date: 2025-01-08 09:00:00
Reference:
'''
import argparse
import copy
import datetime
import os, sys
import shutil
import time
import subprocess
import json
# from shapely.wkt import dumps, loads
from multiprocessing import cpu_count, Pool



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

def del_folder_if_exists(file_folder: str):
    if not os.path.exists(file_folder):
        return
    shutil.rmtree(file_folder)

def run_a_process(cmd: str, timeout_s=None):
    print("run " + cmd)
    try:
        subprocess.run(cmd.split(" "), timeout=timeout_s)
        return None
    # 超时会触发异常
    except Exception as e:
        print("🔴 Error in run_a_process "+e)
        return e
    
def run_one_zip(lukou_name: str, cp_src_path, cp_dst_path):
    zip_map_file = lukou_name+'.zip'
    cmd_str = "./unzip_one.sh " + os.path.join(cp_src_path, zip_map_file) + " "  + cp_dst_path
    # util.copy_folder(cp_src_path, cp_dst_path)
    if run_a_process(cmd_str, 3600*5) is not None:
        # return ErrorCode.激光建图_超时退出
        check_message = "run faile"

if __name__ == "__main__":
    raw_cross_dir = "/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0125/mmt_rc"

    #多线程处理
    lukou_name_array=["batch-10010", "batch-10013", "batch-10017", "batch-312", "batch-195", "batch-286"]

    # cpu_num = len(lukou_name_array)
    # if cpu_count() < cpu_num:
    #     cpu_num = cpu_count()        
    # p = Pool(cpu_num)

    for lukou_name in lukou_name_array:
        print("start process: ", lukou_name)
        input_dir_sd_link = os.path.join(raw_cross_dir, "match", "batch-10010", "tasks/data_engine/match/input/PL2032_event_dbw_disabled_takeover_20240904-153605_0")
        input_dir_tasks_json = os.path.join(raw_cross_dir, "../new_7lukou", lukou_name, "model/auto_label/input") 
        input_dir_parse = os.path.join(raw_cross_dir, "../new_7lukou", lukou_name, "model/source/data_engine/parse") 
        input_dir_match = os.path.join(raw_cross_dir, "match", lukou_name, "tasks/data_engine/match/output")
        input_dir_cover = os.path.join(raw_cross_dir, "cover", lukou_name)
        input_dir_bev_pose = os.path.join(raw_cross_dir, "refine_pose", lukou_name, "multi_mapping") 
        output_dir = os.path.join(raw_cross_dir, lukou_name) 
        del_folder_if_exists(output_dir)
        add_folder_if_no_exist(output_dir)

        src_files = {
            # 给自动建模
            # platform
            os.path.join(input_dir_tasks_json, "tasks.json") : os.path.join(output_dir, "model/auto_label/input/tasks.json"),

            os.path.join(input_dir_sd_link, "sd_link.geojson") : os.path.join(output_dir, "model/auto_label/input/sd_link.geojson"),
            os.path.join(input_dir_sd_link, "sd_node.geojson") : os.path.join(output_dir, "model/auto_label/input/sd_node.geojson"),

            # parse
            input_dir_parse : os.path.join(output_dir, "model/source/data_engine/parse"),
            
            # cover
            os.path.join(input_dir_cover, "output.json") : os.path.join(output_dir, "model/source/data_engine/cover/output.json"),
            
            # match
            input_dir_match : os.path.join(output_dir, "model/source/data_engine/match"),

            # bev_pose
            input_dir_bev_pose : os.path.join(output_dir, "model/source/bev_mapping/output/multi_mapping"),
                
        }

        for key, value in src_files.items():
            # print(key, value)
            if not os.path.exists(key):
                continue

            if os.path.isdir(key):
                del_folder_if_exists(value)
                # dst_dir = os.path.dirname(value)
                dst_dir = value
                add_folder_if_no_exist(dst_dir)
                copy_folder(key, dst_dir)
            else:
                dst_dir = os.path.dirname(value)
                add_folder_if_no_exist(dst_dir)

                copy_file_with_metadata(key, value)


    #     p.apply_async(run_one_zip, args=(src_files))
    # p.close()
    # p.join()
