# 高精地图：道路建模流程规划
import argparse
import os
import shutil

import json
import time
import subprocess
import sys


BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # 顶层目录加入到python的环境变量中
sys.path.append(BASE_DIR)
sys.path.append(os.path.abspath(os.path.join(BASE_DIR, 'python')))
sys.path.append(os.path.abspath(os.path.join(BASE_DIR, 'hdmapbase')))

from hdmapbase.Model.HDProject import run
from hdmapbase.Log.logger import *
# from hdmapbase_data_download import *
# from work.RoadBoundaryProcess import *

from python.tools import file_tools
# from python import info
# from python import config
# from link.link_cut_gps_process import* 
# from python.link.link_list_merage import *


def write_json(json_path: str, dict):
    folder = os.path.dirname(json_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    b = json.dumps(dict)
    with open(json_path, 'w') as f:
        f.write(b)
        f.close()


def process(data_path, road_branch):
    
    print("*************mapbuild process start now************")
    if not os.path.exists(data_path):
        print("data_path:" + data_path +" 数据路径不存在，请检查!!!!!!")
        return
    
    #读取预处理数据路径
    preprocess_data_path = os.path.join(data_path, 'preprocess')
    if not os.path.exists(preprocess_data_path):
        print("preprocess_data_path:" + preprocess_data_path +" 预处理数据路径不存在，请检查!!!!!!")
        return
    
    #读取重建数据路径
    reconstruction_data_path = os.path.join(data_path, 'reconstruction', 'data_output')
    if not os.path.exists(reconstruction_data_path):
        print("reconstruction_data_path:" + reconstruction_data_path +" 重建数据路径不存在，请检查!!!!!!")
        return

    #数据路径均存在，开始进行建模处理
    print("------准备就绪，开始进行建图任务了------")
    mapbuild_data_path = os.path.join(data_path, 'mapbuild')
    if not os.path.exists(mapbuild_data_path):
        os.mkdir(mapbuild_data_path)
    else:
        file_tools.empty_folder_if_exist(mapbuild_data_path)
    

    log_name: str = 'mapbuild.log'
    log_dir = os.path.join(mapbuild_data_path, log_name)
    print("log_dir:" + log_dir)
    logger = CreateLogger("map_build", log_dir).logger
   
    logger.info("******************自动化建模流程开始***************\n\n")
    start_time = time.time()
    
    logger.info("------road_branch:" + road_branch)
    logger.info("------data_path:" + mapbuild_data_path)
    
    #进行数据下载
    logger.info("--------------整理处理过程中所需数据--------------")      
    
    file_tools.add_folder_if_no_exist(os.path.join(mapbuild_data_path, 'camera'))
    file_tools.add_folder_if_no_exist(os.path.join(mapbuild_data_path, 'image_recog'))
    file_tools.add_folder_if_no_exist(os.path.join(mapbuild_data_path, 'image_seg'))
    file_tools.add_folder_if_no_exist(os.path.join(mapbuild_data_path, 'GPS'))
    file_tools.add_folder_if_no_exist(os.path.join(mapbuild_data_path, 'trafficsign'))
    
    #gps数据
    gps_names = file_tools.get_files_name_with_suffix_reomvesuffix(reconstruction_data_path, '.gps')
    if not gps_names:
         logger.info("处理数据中无车体位姿数据，处理中断!!!!!!")
         return
    
    gps_dir = os.path.join(mapbuild_data_path, 'GPS')
    for i in range(len(gps_names)):
        file_tools.add_folder_if_no_exist(os.path.join(gps_dir, str(i), "estimate_pose"))
        file_tools.copy_file(os.path.join(reconstruction_data_path, gps_names[i]+'.gps'), os.path.join(gps_dir, str(i), "estimate_pose", gps_names[i]+'.gps'))
        #感知数据
        file_tools.copy_files_with_suffix(os.path.join(preprocess_data_path, gps_names[i]), os.path.join(mapbuild_data_path, 'image_recog'), ".perception")
        #相机参数数据
        file_tools.copy_files_with_suffix(os.path.join(preprocess_data_path, gps_names[i]), os.path.join(mapbuild_data_path, 'camera'), ".camera")
        #语义分割数据
        img_seg_dir = os.path.join(mapbuild_data_path, 'image_seg', gps_names[i], 'image_seg')
        file_tools.copy_folder(os.path.join(preprocess_data_path, gps_names[i], 'image_seg'), img_seg_dir)
    
    #交通牌数据
    overground_path = os.path.join(reconstruction_data_path, 'overground')
    file_tools.copy_files_with_suffix(overground_path, os.path.join(mapbuild_data_path, 'trafficsign'), ".json")
        
   
    end_time1 = time.time()
    logger.info("--------------数据整理完毕，总共用时:" + str((end_time1 - start_time) / 60) + "分--------------")   

    #调用道路建模Python
    logger.info("--------------hdmapbase 处理开始--------------")
    run(gps_dir)
    end_time2 = time.time()
    logger.info("--------------hdmapbase 处理完毕，总共用时:" + str((end_time2 - end_time1) / 60) + "分--------------")

    # #laneGroup生成与上传
     # exe path
    logger.info("--------------hdmapbuild 处理开始--------------")
    project_folder = os.path.dirname(os.path.abspath(__file__))
    hdmap_build_server = project_folder + "/" + "hdmap_build/lib/hdmap_build"
    hdmap_build = hdmap_build_server + " -tilepath " + gps_dir + " -road_branch " + road_branch
    
    logger.info(hdmap_build)
    subprocess.call([hdmap_build], shell=True)
    end_time3 = time.time()
    logger.info("--------------hdmapbuild 处理完毕，总共用时:" + str((end_time3 - end_time2) / 60) + "分--------------")

    end_time = time.time()
    logger.info('Run all use time : %s min'%((end_time - start_time) / 60))

    error_code = 0
    result_json: dict = {"error_code": error_code}
    write_json(os.path.join(data_path, "mapbuild", "result.json"),
               result_json)
    return

def move_file(move_to_path, old_path, filetype):
    print('path:', move_to_path)
    
    if not os.path.exists(move_to_path):
        os.mkdir(move_to_path)
    
    filelist = os.listdir(old_path) #列出该目录下的所有文件,listdir返回的文件列表是不包含路径的。
    print(filelist)
    for file in filelist:
        src = os.path.join(old_path, file)#返回路径
        print('src:', src)
        img_recog_path = os.path.join(src, file+"."+filetype)
        print('img_recog_path:', img_recog_path)
        check_file = os.path.join(move_to_path, file+"."+filetype)
        print('check_path:', check_file)
        if os.path.exists(check_file):
            continue
        if os.path.exists(img_recog_path):
             shutil.copy(img_recog_path, move_to_path)
             shutil.rmtree(src)
        
       

parser = argparse.ArgumentParser(description='Example with long option names')

parser.add_argument('--data_path', type=str,
                    default='USE_TIME_TEST')

parser.add_argument('--road_branch', type=str,
                    default='USE_TIME_TEST')

if __name__ == '__main__':
    args = parser.parse_args()
    data_path = str(args.data_path)
    road_branch = str(args.road_branch)
    
    # data_path = "/home/test/data/1641452080537913"
    # road_branch = "test"
    process(data_path, road_branch)
   
