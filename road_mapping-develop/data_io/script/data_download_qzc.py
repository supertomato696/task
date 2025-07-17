import json
import os
import sys
import time
import argparse
from typing import List
import multiprocessing
from multiprocessing import cpu_count, Pool
import random

project_root_folder = os.path.abspath(__file__+"/../../../")
sys.path.append(project_root_folder)
from common.data.error_code import ErrorCode
from data_io.script.bev_label_map import DataType


def json_file_to_dict(json_path: str) -> dict:
    if not os.path.exists(json_path):
        print("🔴 error 路径不存在", json_path)
        return {}
    data = {}
    with open(json_path, 'r') as f:
        try:
            data = json.load(f)
        except Exception as e:
            print('Reason: ' + str(e))
            return {}
    return data


def dict_to_json_file(json_path: str, dict: dict):
    def convert(x):
        if hasattr(x, "tolist"):
            return x.tolist()
        raise TypeError(x)
    folder = os.path.dirname(json_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    b = json.dumps(dict, default=convert, ensure_ascii=False)
    with open(json_path, 'w') as f:
        f.write(b)
        f.close()

import shutil
def del_folder_if_exist(dir=''):
    if os.path.exists(dir):
        shutil.rmtree(dir, ignore_errors=True)

def del_file_if_exist(file_path: str):
    if not os.path.exists(file_path):
        return
    os.remove(file_path)


def run_a_process(cmd: str, timeout_s=None):
    import subprocess
    """
    运行一个子进程
    :param cmd: 命令行
    :param timeout_s: 超时
    :return:
    """
    print("run cmd:\n------------------------------------------------\n "
          + cmd
          + "\n------------------------------------------------")
    try:
        subprocess.run(cmd.split(" "), timeout=timeout_s)
        return None
    except Exception as e:
        print(e)
        return e


def thread_pool(cpu_num: int, func_name_param_list: list):
    from concurrent.futures import ThreadPoolExecutor
    from multiprocessing import cpu_count
    if cpu_num < 1:
        cpu_num = cpu_count()

    with ThreadPoolExecutor(cpu_num) as executor:
        future_list = []
        for param in func_name_param_list:
            while True:
                try:
                    future = executor.submit(*param)
                except Exception as err:
                    print("🔴 Error in thread_pool, wait to start thread."+err)
                    time.sleep(5)
                    continue
                break
            future_list.append(future)


def get_folder_name_in_path(path: str) -> str:
    if not os.path.exists(path):
        print('error, get_folder_name_in_path(), no {}'.format(path))
        return ''
    if os.path.isdir(path):
        return path.split('/')[-1]
    else:
        return path.split('/')[-2]


def get_files_path_recursion_with_prefix(root='', prefix='') -> List[str]:
    if not os.path.exists(root):
        print('get_files_path_recursion_with_prefix(), no {}'.format(root))
        return []
    files_path = []
    for main_dir, dirs, file_name_list in os.walk(root):
        for name in file_name_list:
            if name.startswith(prefix):
                file_path = os.path.join(main_dir, name)
                files_path.append(file_path)
    return sorted(files_path)


def get_folder_path_in_file_path(file_path=''):
    return os.path.dirname(file_path)


def get_files_path_recursion(root='') -> List[str]:
    if not os.path.exists(root):
        print('get_files_path_recursion(), no {}'.format(root))
        return []
    files_path = []
    for main_dir, dirs, file_name_list in os.walk(root):
        for name in file_name_list:
            files_path.append(os.path.join(main_dir, name))
    return sorted(files_path)


def multi_trail_download(download_info_json: str, download_folder: str, tile_server: str, specific_types: str):
    trail_download_info_list = json_file_to_dict(download_info_json)
    params = []
    for one_trail in trail_download_info_list:
        entry = os.path.join(project_root_folder, "data_io/script/trail_download.py")
        cmd = "python3 {} --server_host={} --tile_id={} --trail_id={} --tile_branch={} --down_folder={} --specific_types={}".format(
            entry,
            tile_server,
            one_trail["download_tile_id"],
            one_trail["trail_id"],
            one_trail["download_branch"],
            download_folder,
            specific_types)
        params.append([run_a_process, cmd, 3600])
    thread_pool(5, params)


def data_integrity_check(download_folder: str):
    total_trail_cnt = 0
    data_check_error_log = {}
    for one_flag_file in get_files_path_recursion_with_prefix(download_folder, "完成下载标记文件.txt"):
        trail_folder = get_folder_path_in_file_path(one_flag_file)
        trail_id = get_folder_name_in_path(trail_folder)
        error_log = []
        if len(get_files_path_recursion(os.path.join(trail_folder, "lidar"))) == 0:
            error_log.append("no_lidar")
        if len(get_files_path_recursion(os.path.join(trail_folder, "f_ca_120"))) == 0:
            error_log.append("no_f_ca_120")
        if (len(get_files_path_recursion(os.path.join(trail_folder, "perception_raw"))) == 0) and (len(get_files_path_recursion(os.path.join(trail_folder, "bevlane_semantic_mask_f_ca_120"))) == 0):
            error_log.append("no_perception_raw")
        if not os.path.exists(os.path.join(trail_folder, "calib_cam_params.json")):
            error_log.append("no_calib_cam")
        if not os.path.exists(os.path.join(trail_folder, "calib_lidar_params.json")):
            error_log.append("no_calib_lidar")
        if not os.path.exists(os.path.join(trail_folder, "nav_ins.txt")):
            error_log.append("no_nav_ins")
        if not os.path.exists(os.path.join(trail_folder, "odometry.txt")):
            error_log.append("no_odometry")
        if len(error_log) > 0:
            del_file_if_exist(one_flag_file)
            data_check_error_log[trail_id] = error_log
        total_trail_cnt += 1
    return total_trail_cnt, data_check_error_log


def zb_auto_data_integrity_check(download_folder: str):
    total_trail_cnt = 0
    data_check_error_log = {}
    for one_flag_file in get_files_path_recursion_with_prefix(download_folder, "完成下载标记文件.txt"):
        trail_folder = get_folder_path_in_file_path(one_flag_file)
        trail_id = get_folder_name_in_path(trail_folder)
        error_log = []
        if len(get_files_path_recursion(os.path.join(trail_folder, "lidar"))) == 0:
            error_log.append("no_lidar")
        if len(get_files_path_recursion(os.path.join(trail_folder, "f_ca_120"))) == 0:
            error_log.append("no_f_ca_120")
        if not os.path.exists(os.path.join(trail_folder, "calib_cam_params.json")):
            error_log.append("no_calib_cam")
        if not os.path.exists(os.path.join(trail_folder, "calib_lidar_params.json")):
            error_log.append("no_calib_lidar")
        if not os.path.exists(os.path.join(trail_folder, "nav_ins.txt")):
            error_log.append("no_nav_ins")
        if not os.path.exists(os.path.join(trail_folder, "odometry.txt")):
            error_log.append("no_odometry")
        if len(error_log) > 0:
            data_check_error_log[trail_id] = error_log
        total_trail_cnt += 1
    return total_trail_cnt, data_check_error_log


def data_download_and_check(download_info_json: str, download_folder: str,
                            tile_server: str, specific_types: str) -> ErrorCode:
    print("--------------------------------------------")
    print("             多进程 trail 数据下载            ")
    print("--------------------------------------------")
    multi_trail_download(download_info_json, download_folder, tile_server, specific_types)

    print("--------------------------------------------")
    print("                数据完整性检查                ")
    print("--------------------------------------------")
    total_trail_cnt, data_check_error_log = data_integrity_check(download_folder)

    error_trail_cnt = 0
    for v in data_check_error_log.values():
        if v != []:
            error_trail_cnt += 1

    if (error_trail_cnt > 0):
        print(data_check_error_log)
        print("--------------------------------------------")
        print("             下载有缺失，重试一次               ")
        print("--------------------------------------------")
        time.sleep(2)
        multi_trail_download(download_info_json, download_folder, tile_server, specific_types)
        total_trail_cnt, data_check_error_log = data_integrity_check(download_folder)

    error_trail_cnt = 0
    for v in data_check_error_log.values():
        if v != []:
            error_trail_cnt += 1

    error_code = ErrorCode.中业运行正常
    if (total_trail_cnt == 1 and error_trail_cnt == 1):
        error_code = ErrorCode.数据下载_缺失1趟
    if error_trail_cnt == 2:
        error_code = ErrorCode.数据下载_缺失2趟
    if error_trail_cnt == 3:
        error_code = ErrorCode.数据下载_缺失3趟
    if error_trail_cnt > 3:
        error_code = ErrorCode.数据下载_缺失超过3趟

    data_download_result = {"error_code": error_code.value, "data_check_error": data_check_error_log}
    print("data_download_result:", data_download_result)
    dict_to_json_file(os.path.join(download_folder, "download_result.json"), data_download_result)


def zb_auto_data_download_and_check(download_info_json: str, download_folder: str,
                                    tile_server: str, specific_types: str) -> ErrorCode:
    print("--------------------------------------------")
    print("             多进程 trail 数据下载            ")
    print("--------------------------------------------")
    multi_trail_download(download_info_json, download_folder, tile_server, specific_types)

    print("--------------------------------------------")
    print("                数据完整性检查                ")
    print("--------------------------------------------")
    total_trail_cnt, data_check_error_log = zb_auto_data_integrity_check(download_folder)

    error_trail_cnt = 0
    for v in data_check_error_log.values():
        if v != []:
            error_trail_cnt += 1
    error_code = ErrorCode.中业运行正常
    if error_trail_cnt == 1:
        error_code = 131

    data_download_result = {"error_code": error_code.value, "data_check_error": data_check_error_log}
    print("data_download_result:", data_download_result)
    dict_to_json_file(os.path.join(download_folder, "download_result.json"), data_download_result)

def add_folder_if_no_exist(folder_path: str):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)


from collections import defaultdict
from data_io.script.bev_converter import BevReaderMmtRc, BevReaderByd


def download_mmt_rc_one_trail(one_trail, download_folder: str, use_opt_pose: int, data_type: str, out_queue):

    tile_id = one_trail["download_tile_id"]
    trail_id = one_trail["trail_id"]
    trail_name = one_trail["trail_name"]
    save_folder = os.path.join(download_folder, str(tile_id), str(trail_id))
    # save_json = os.path.join(save_folder, "bev_label", str(bev_timestamp_us)+".json")
    del_folder_if_exist(save_folder+"/bev_label")
    add_folder_if_no_exist(save_folder)
    print("start download_mmt_rc_one_trail")

    # TODO:qzc 解析 mmt 数据
    bev_reader = BevReaderMmtRc()
    # 解析 轨迹
    traj_data = {}

    data_engine_dir = os.path.join(download_folder, "../../../source/data_engine")
    parse_dir = os.path.join(data_engine_dir, "parse")
    data_folder = os.path.join(parse_dir, trail_name)

    egopose_path = os.path.join(data_folder, "position/_mla_egopose.json")
    if use_opt_pose:
        bev_dir = os.path.join(download_folder, "../../../source/bev_mapping/output/multi_mapping", trail_name)
        egopose_path = os.path.join(bev_dir, "_mla_egopose_refine.json")
    # 先拷贝一份：
    dst_egopose_path = os.path.join(save_folder, "_mla_egopose.json")
    shutil.copyfile(egopose_path, dst_egopose_path)

    save_egopose_txt = os.path.join(save_folder, "nav_ins.txt")
    bev_reader.get_ins_dict_qzc(egopose_path, save_egopose_txt)

    save_egopose_txt = os.path.join(save_folder, "odometry.txt")
    bev_reader.get_egopose_dict_qzc(egopose_path, save_egopose_txt, use_opt_pose)

    # 直接下载到 mapping_output/veh_pose ： pose 需要转到 wgs84
    opt_egopose_path = os.path.join(data_folder, "position/_mla_egopose.json")
    if use_opt_pose:
        bev_dir = os.path.join(download_folder, "../../../source/bev_mapping/output/multi_mapping", trail_name)
        opt_egopose_path = os.path.join(bev_dir, "_mla_egopose_refine.json")

    save_egopose_txt = os.path.join(save_folder, "opt_pose_veh.txt")
    ret = bev_reader.get_egopose_dict_qzc(opt_egopose_path, save_egopose_txt, use_opt_pose)
    if ret == -1:
        out_queue.put(-1)
        return 

    # 解析到： save_folder/bev_label 目录中
    # 读取信号灯

    # print(bev_data_trafic_lights[0]['meta']['sensor_timestamp_us'])
    bev_data_trafic_lights = {}
    try:
        traffic_light_path = os.path.join(data_folder, "bev_features/_worldmodel_traffic_light.json")
        with open(traffic_light_path, 'r', encoding='utf-8') as file:
            bev_data_trafic_lights = json.load(file)
    except Exception  as e:
        # 捕获特定类型的异常并处理
        print(f"发生异常：{e}")
        print("文件不存在：", traffic_light_path)

    # 用于同步 traffic 和 其他要素
    m_time_to_traffic_id = {}
    traffic_light_size = len(bev_data_trafic_lights)
    for id in range(traffic_light_size):
        bev_frame_data = bev_data_trafic_lights[id]
        bev_timestamp_us = bev_frame_data['meta']['sensor_timestamp_us']
        m_time_to_traffic_id[bev_timestamp_us] = id

    # 读取其他要素
    ddld_landmark_path = os.path.join(data_folder, "bev_features/_ddld_landmark.json")
    bev_data_ddld_landmarks = {}
    with open(ddld_landmark_path, 'r', encoding='utf-8') as file:
        bev_data_ddld_landmarks = json.load(file)

    # 解析数据'
    for bev_frame_data in bev_data_ddld_landmarks:
        bev_timestamp_us = bev_frame_data['meta']['sensor_timestamp_us']

        out_frame_dict = {
            "cat_version": "MMT_RC",
            "sensor_timestamp_us": bev_timestamp_us,
            "detail":{
                "instance_list":[]
            }
        }

        # 车道中心线
        out_one_obj_list = bev_reader.get_lane_center_qzc(bev_frame_data)
        for obj in out_one_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)
        
        # 车道边界
        out_one_obj_list = bev_reader.get_lane_boundary_qzc(bev_frame_data)
        for obj in out_one_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)


        # 道路边界
        out_one_obj_list = bev_reader.get_road_boundary_qzc(bev_frame_data)
        for obj in out_one_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)


        # 停止位置
        out_one_obj_list = bev_reader.get_stopline_qzc(bev_frame_data)
        for obj in out_one_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)


        # arrow
        out_one_obj_list = bev_reader.get_arrow_qzc(bev_frame_data)
        for obj in out_one_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)


        # cross walk
        out_one_obj_list = bev_reader.get_cross_walk_qzc(bev_frame_data)
        for obj in out_one_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)

        # junction
        # out_one_obj_list = bev_reader.get_junction_qzc(bev_frame_data)
        # for obj in out_one_obj_list:
        #     out_frame_dict["detail"]["instance_list"].append(obj)                

        # TODO:qzc use file :_worldmodel_traffic_light.json
        # 交通灯
        if bev_timestamp_us in m_time_to_traffic_id:
            traffic_id = m_time_to_traffic_id[bev_timestamp_us]
            out_one_obj_list = bev_reader.get_traffic_light_qzc(bev_data_trafic_lights[traffic_id])
            if out_one_obj_list is not None:
                for obj in out_one_obj_list:
                    out_frame_dict["detail"]["instance_list"].append(obj)

            # 交通灯灯芯
            # traffic_id = m_time_to_traffic_id[bev_timestamp_us]
            # out_one_obj_list = bev_reader.get_traffic_light_bulb_qzc(bev_data_trafic_lights[traffic_id])
            # if out_one_obj_list is not None:
            #     for obj in out_one_obj_list:
            #         out_frame_dict["detail"]["instance_list"].append(obj)

        save_json = os.path.join(save_folder, "bev_label", str(bev_timestamp_us)+".json")
        dict_to_json_file(save_json, out_frame_dict)

    # if len(bev_data_ddld_landmarks) > 0:
    out_queue.put(len(bev_data_ddld_landmarks))
    # out_queue.put(0)
    # out_queue.put(-1)

    # num = random.randint(1,5)
    # if num % 2 == 1:
    #     out_queue.put(0)
    # else:
    #     out_queue.put(-1)


def download_byd_lidar_one_trail(one_trail, download_folder: str, use_opt_pose: int, data_type: str, out_queue):


    tile_id = one_trail["download_tile_id"]
    trail_id = one_trail["trail_id"]
    trail_name = one_trail["trail_name"]
    save_folder = os.path.join(download_folder, str(tile_id), str(trail_id))
    # save_json = os.path.join(save_folder, "bev_label", str(bev_timestamp_us)+".json")
    del_folder_if_exist(save_folder+"/bev_label")
    add_folder_if_no_exist(save_folder)
    print("start download_byd_one_trail")

    # TODO:qzc 解析 mmt 数据
    bev_reader = BevReaderByd()
    # 解析 轨迹
    traj_data = {}

    data_engine_dir = os.path.join(download_folder, "../../../source/data_engine")
    parse_dir = os.path.join(data_engine_dir, "parse")
    data_folder = os.path.join(parse_dir, trail_name)

    egopose_path = os.path.join(data_folder, "position/egopose.json")
    if use_opt_pose:
        bev_dir = os.path.join(download_folder, "../../../source/lidar_mapping/output/trajs", trail_name)
        egopose_path = os.path.join(bev_dir, "_mla_egopose_refine.json")
    # 先拷贝一份：
    dst_egopose_path = os.path.join(save_folder, "_mla_egopose.json")
    shutil.copyfile(egopose_path, dst_egopose_path)

    save_egopose_txt = os.path.join(save_folder, "nav_ins.txt")
    bev_reader.get_ins_dict_qzc(egopose_path, save_egopose_txt)

    save_egopose_txt = os.path.join(save_folder, "odometry.txt")
    bev_reader.get_egopose_dict_qzc(egopose_path, save_egopose_txt, use_opt_pose)

    # 直接下载到 mapping_output/veh_pose ： pose 需要转到 wgs84
    opt_egopose_path = os.path.join(data_folder, "position/egopose.json")
    if use_opt_pose:
        bev_dir = os.path.join(download_folder, "../../../source/lidar_mapping/output/trajs", trail_name)
        opt_egopose_path = os.path.join(bev_dir, "_mla_egopose_refine.json")

    save_egopose_txt = os.path.join(save_folder, "opt_pose_veh.txt")
    ret = bev_reader.get_egopose_dict_qzc(opt_egopose_path, save_egopose_txt, use_opt_pose)
    if ret == -1:
        out_queue.put(-1)
        return 
    
    # 解析到： save_folder/bev_label 目录中
    pcd_path = os.path.join(download_folder, "../../../source/lidar_mapping/output/layer_pointcloud_semantic.pcd")
    if os.path.exists(pcd_path):
        out_queue.put(1)
    else:
        out_queue.put(0)
    
def download_byd_bev_data(bev_reader : BevReaderByd, data_type: str, data_folder, save_folder):
    # 读取信号灯
    bev_data_trafic_lights = {}
    try:
        # 尝试执行的代码
        traffic_light_path = os.path.join(data_folder, "bev_features/_perception_vision_fusion_traffic_info.json")
        with open(traffic_light_path, 'r', encoding='utf-8') as file:
            bev_data_trafic_lights = json.load(file)
    except Exception  as e:
        # 捕获特定类型的异常并处理
        print(f"发生异常：{e}")
        print("文件不存在：", traffic_light_path)


    # 用于同步 traffic 和 其他要素

    m_time_to_traffic_id = {}
    traffic_light_size = len(bev_data_trafic_lights)
    for id in range(traffic_light_size):
        bev_frame_data = bev_data_trafic_lights[id]
        if ('header' in bev_frame_data) and ('measurement_timestamp' in bev_frame_data['header']):
            bev_timestamp_us = int((bev_frame_data['header']['measurement_timestamp'])*1e6)
            m_time_to_traffic_id[bev_timestamp_us] = id

    # 读取其他要素
    ddld_landmark_path = os.path.join(data_folder, "bev_features/_perception_detection_bev_lane_track.json")
    bev_data_ddld_landmarks = {}
    with open(ddld_landmark_path, 'r', encoding='utf-8') as file:
        bev_data_ddld_landmarks = json.load(file)

    # 去除重复的帧
    abnormal_frame_cnt = 0
    duplicate_frame_cnt = 0
    m_time_to_duplicate = {}
    for bev_frame_data in bev_data_ddld_landmarks:
        bev_timestamp_us = int((bev_frame_data['header']['measurement_timestamp'])*1e6)
        sequence_num = int(bev_frame_data['header']['sequence_num'])
        # if sequence_num > 10000:
        #     abnormal_frame_cnt+=1
        #     continue
        if bev_timestamp_us in m_time_to_duplicate:
            duplicate_frame_cnt+= 1
        m_time_to_duplicate[bev_timestamp_us] = sequence_num

    if len(m_time_to_duplicate) == 0:
        print("帧异常，原始帧数:{}, 异常帧:{}, 重复帧:{}".format(len(m_time_to_duplicate), abnormal_frame_cnt, duplicate_frame_cnt))
        return 0

    # 解析数据'
    for bev_frame_data in bev_data_ddld_landmarks:
        bev_timestamp_us = int((bev_frame_data['header']['measurement_timestamp'])*1e6)
        sequence_num = int(bev_frame_data['header']['sequence_num'])

        if bev_timestamp_us not in m_time_to_duplicate or m_time_to_duplicate[bev_timestamp_us] != sequence_num:
            continue

        out_frame_dict = {
            "cat_version": "BYD_BEV",
            "sensor_timestamp_us": bev_timestamp_us,
            "detail":{
                "instance_list":[]
            }
        }
        # 分合流信息
        # out_obj_list = bev_reader.get_lane_sm_qzc(bev_frame_data)
        # for obj in out_obj_list:
        #     out_frame_dict["detail"]["instance_list"].append(obj)   

        # 1. 车道中心线
        out_obj_list = bev_reader.get_lane_center_qzc(bev_frame_data)
        for obj in out_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)
        
        # 2. 道路边界线
        out_obj_list = bev_reader.get_road_boundary_qzc(bev_frame_data)
        for obj in out_obj_list:
            out_frame_dict["detail"]["instance_list"].append(obj)

        # 3. 路面其他要素都在这里分发读取
        # 车道边界\道路边界\停止位置\arrow\road mark\cross walk
        if 'lane_markers' in bev_frame_data:
            out_obj_list = bev_reader.get_lane_marker_qzc(bev_frame_data, data_type)
            for obj in out_obj_list:
                out_frame_dict["detail"]["instance_list"].append(obj)

        # TODO:qzc use file :_worldmodel_traffic_light.json
        # 交通灯
        if bev_timestamp_us in m_time_to_traffic_id:
            traffic_id = m_time_to_traffic_id[bev_timestamp_us]
            out_one_obj_list = bev_reader.get_traffic_light_qzc(bev_data_trafic_lights[traffic_id])
            # if out_one_obj_list is not None:
            for obj in out_one_obj_list:
                out_frame_dict["detail"]["instance_list"].append(obj)

        save_json = os.path.join(save_folder, "bev_label", str(bev_timestamp_us)+".json")
        dict_to_json_file(save_json, out_frame_dict)

    return len(bev_data_ddld_landmarks)


import glob
def find_files_with_extension(root_folder_path='', extension = "") -> List[str]:
    subfolders_path = []
    if not os.path.exists(root_folder_path):
        return []
    pattern = os.path.join(root_folder_path, extension)
    subfolders_path = glob.glob(pattern)
    return sorted(subfolders_path)


def download_byd_lidar_data(data_folder, save_folder):
    out_frame_dict = {
        "cat_version": "BYD_LIDAR_BEV_B",
        # "sensor_timestamp_us": bev_timestamp_us,
        "sensor_timestamp_us": [],
        "pcd_files": []
    }
    pcd_files = find_files_with_extension(data_folder, "*.pcd")
    for file_path in pcd_files:
        file_name = os.path.basename(file_path)
        # file_dir = os.path.dirname(file_path)
        lidar_timestamp_us = int(file_name[:-4])
        out_frame_dict["sensor_timestamp_us"].append(lidar_timestamp_us)
        out_frame_dict["pcd_files"].append(file_path)
    save_json = os.path.join(save_folder, "lidar_label", "all_pcd_path.json")
    dict_to_json_file(save_json, out_frame_dict)

    return len(out_frame_dict["pcd_files"])


def download_byd_one_trail(one_trail, download_folder: str, use_opt_pose: int, data_type: str, out_queue):
    tile_id = one_trail["download_tile_id"]
    trail_id = one_trail["trail_id"]
    trail_name = one_trail["trail_name"]
    save_folder = os.path.join(download_folder, str(tile_id), str(trail_id))
    del_folder_if_exist(save_folder+"/bev_label")
    add_folder_if_no_exist(save_folder)
    del_folder_if_exist(save_folder+"/lidar_label")
    add_folder_if_no_exist(save_folder)
    print("start download_byd_one_trail")

    # TODO:qzc 解析 mmt 数据
    bev_reader = BevReaderByd()

    pose_dir = ""
    lidar_dir = ""
    if data_type == DataType.BYD_LIDAR_BEV_B.name:
        # byd_dir = "../../../source/lidar_mapping/output/trajs"
        byd_dir = "../../../source/lidar_mapping/output/multi_mapping"
        lidar_dir_tmp = "../../../source/lidar_mapping/output/multi_mapping"
        pose_dir = os.path.join(download_folder, byd_dir, trail_name)
        lidar_dir = os.path.join(download_folder, lidar_dir_tmp, trail_name, "pcd")
    elif data_type == DataType.BYD_LIDAR_B.name:
        byd_dir = "../../../source/lidar_mapping/output/trajs"
        pose_dir = os.path.join(download_folder, byd_dir, trail_name)
    elif data_type == DataType.BYD_BEV.name:
        byd_dir = "../../../source/bev_mapping/output/multi_mapping"
        pose_dir = os.path.join(download_folder, byd_dir, trail_name)

    data_engine_dir = os.path.join(download_folder, "../../../source/data_engine")
    parse_dir = os.path.join(data_engine_dir, "parse")
    data_folder = os.path.join(parse_dir, trail_name)

    egopose_path = os.path.join(data_folder, "position/egopose.json")
    if use_opt_pose:
        egopose_path = os.path.join(pose_dir, "_mla_egopose_refine.json")
    # 先拷贝一份：
    dst_egopose_path = os.path.join(save_folder, "_mla_egopose.json")
    shutil.copyfile(egopose_path, dst_egopose_path)

    save_egopose_txt = os.path.join(save_folder, "nav_ins.txt")
    bev_reader.get_ins_dict_qzc(egopose_path, save_egopose_txt)

    save_egopose_txt = os.path.join(save_folder, "odometry.txt")
    bev_reader.get_egopose_dict_qzc(egopose_path, save_egopose_txt, use_opt_pose)

    # 直接下载到 mapping_output/veh_pose ： pose 需要转到 wgs84
    opt_egopose_path = os.path.join(data_folder, "position/egopose.json")
    if use_opt_pose:
        opt_egopose_path = os.path.join(pose_dir, "_mla_egopose_refine.json")

    save_egopose_txt = os.path.join(save_folder, "opt_pose_veh.txt")
    ret = bev_reader.get_egopose_dict_qzc(opt_egopose_path, save_egopose_txt, use_opt_pose)
    if ret == -1:
        out_queue.put(-1)
        return 

    ################### 1. 读取 bev 结果 ####################################

    ################### 2. 读取 lidar 结果 ####################################
    if data_type == DataType.BYD_LIDAR_BEV_B.name:
        # 解析到： save_folder/bev_label 目录中
        bev_data_len = download_byd_bev_data(bev_reader, data_type, data_folder, save_folder)
        # 解析到： save_folder/lidar_label 目录中
        lidar_data_len = download_byd_lidar_data(lidar_dir, save_folder)
        # if lidar_data_len == 0: # 如果沒有激光数据
        #     out_queue.put(0)
        # else: # 纯激光 或 bev+激光
        #     out_queue.put(lidar_data_len + bev_data_len)
        out_queue.put(lidar_data_len + bev_data_len)
    elif data_type == DataType.BYD_LIDAR_B.name:
        # 解析到： save_folder/bev_label 目录中
        bev_data_len = 0
        # bev_data_len = download_byd_bev_data(bev_reader, data_type, data_folder, save_folder)
        pcd_path = os.path.join(download_folder, "../../../source/lidar_mapping/output/layer_pointcloud_semantic.pcd")
        if os.path.exists(pcd_path):
            if bev_data_len > 0: # 如果同时有bev和激光数据
                out_queue.put(bev_data_len)
            else: # 如果只有激光数据
                out_queue.put(1)
        else:
            out_queue.put(0)
    elif data_type == DataType.BYD_BEV.name:
        # 解析到： save_folder/bev_label 目录中
        bev_data_len = download_byd_bev_data(bev_reader, data_type, data_folder, save_folder)
        out_queue.put(bev_data_len)
    else:
        out_queue.put(0)

def download_check(download_info_json: str, download_folder: str,
                            tile_server: str, specific_types: str, use_opt_pose: int, data_type: str) -> ErrorCode:

    
    trail_download_info_list = json_file_to_dict(download_info_json)
    total_trail_cnt = len(trail_download_info_list)
    trail_cnt = 0

    q = multiprocessing.Manager().Queue()

    cpu_cnt = cpu_count()
    if total_trail_cnt == 1:
        cpu_cnt = 1

    #多线程处理
    print("total_trail_num:{}, cpu_count:{}".format(total_trail_cnt, cpu_cnt))
    p = Pool(cpu_cnt)
    # p = Pool(4)
    for one_trail in trail_download_info_list:
        # download_mmt_rc_one_trail(one_trail, download_folder, use_opt_pose)
        # trail_cnt+=1
        # print("trail_id(total_trail_num):{}/{}".format(trail_cnt, total_trail_cnt))
        
        if data_type == DataType.MMT_RC.name:
            p.apply_async(download_mmt_rc_one_trail, args=(one_trail, download_folder, use_opt_pose, data_type, q))
        # elif data_type == DataType.BYD_LIDAR_B.name:
        #     p.apply_async(download_byd_lidar_one_trail, args=(one_trail, download_folder, use_opt_pose, data_type, q))
        elif data_type == DataType.BYD_BEV.name or data_type == DataType.BYD_LIDAR_B.name or data_type == DataType.BYD_LIDAR_BEV_B.name:
            p.apply_async(download_byd_one_trail, args=(one_trail, download_folder, use_opt_pose, data_type, q))
            # TODO:qzc ziyan, debug close
            # download_byd_one_trail(one_trail, download_folder, use_opt_pose, data_type, q)

    p.close()
    p.join()

    results_num = []
    while not q.empty():
        results_num.append(q.get_nowait())
    print("success load tral num: ", len(results_num))

    data_check_error_log = "success"
    error_code = ErrorCode.中业运行正常
    if (total_trail_cnt == 0):
        error_code = ErrorCode.数据下载_文件缺失
        data_check_error_log = "数据下载_文件缺失"
        print(data_check_error_log)
    else:
        if (all(x == -1 for x in results_num)):
            error_code = ErrorCode.数据下载_BEV位姿缺失
            data_check_error_log = "数据下载_BEV或激光位姿缺失"
            print(data_check_error_log)
        elif(all(x == 0 for x in results_num)):
            error_code = ErrorCode.数据下载_感知数据缺失
            data_check_error_log = "数据下载_感知数据缺失"
            print(data_check_error_log)

    data_download_result = {"error_code": error_code.value, "data_check_error": data_check_error_log}
    dict_to_json_file(os.path.join(download_folder, "download_result.json"), data_download_result)
    print("data_download_result:", data_download_result)
    print("\n")


parser = argparse.ArgumentParser()
parser.add_argument('--download_info_json', type=str, default=project_root_folder+"/data_io/script/download_info_demo.json")
parser.add_argument('--download_folder', type=str, default="/tmp/test_download")
parser.add_argument('--tile_server', type=str, default="172.21.204.79:9382")
parser.add_argument('--specific_types', type=str, default="loc,nav_imu,nav_ins,ndm_location,odometry,odometry3d,perception_raw,lidar,f_ca_120,vehicle_status")
parser.add_argument('--task_type', type=str, default="zl")
parser.add_argument('--use_opt_pose', type=int, default=0)
parser.add_argument('--data_type', type=str, default=DataType.MMT_RC.name) # mmt: momenta生产数据, mmt_rc: momenta研采, byd: 比亚迪生产, byd_rc: 比亚迪研采
parser.add_argument('--debug_model', type=int, default=1, help="是否使用debug测试") 


if __name__ == "__main__":
    args = parser.parse_args()
    download_check(args.download_info_json, args.download_folder, args.tile_server, args.specific_types, args.use_opt_pose, args.data_type)
