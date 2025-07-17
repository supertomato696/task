import road_model_pipeline_multi_qzc as multi_traj_mapping
import sys
import json
import os
import socket
import time
# import pyproj
import argparse
import shutil
# import shapely.ops
# import shapely.affinity
# from shapely.geometry import Polygon, LineString, GeometryCollection
from multiprocessing import cpu_count, Pool

project_root_folder = os.path.abspath(__file__+"/../../")
# task_type_name_dict = {"zl": "连续直路", "lk2": "离散路口", "zb_manual": "众包人工", "zb_auto": "众包自动", "zb_auto_zc": "自采自动"}
sys.path.append(project_root_folder)
# import common.util.util as util
from common.util.util import *
# from common.data.error_code import ErrorCode
# from data_io.script.util import get_utm_num_from_gps_polygon_wkt
# from road_model.script.tran_imgpos_coor import trans_imgpos
from preprocess.script.util import log
from data_io.script.bev_label_map import DataType

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


def process_evaluation(workspace_folder, lukou_ids, debug_docker_tag, debug_run_evaluation, debug_evaluation_dir):
    ROAD_MAPPING_TAG="v3.06.24"
    # 1 解压生成的 shp 文件
    # unzip_dir = os.path.join(task_arg.task_folder, "bev_mapbuild_out", ROAD_MAPPING_TAG)

    # time_now = time.strftime("%Y%m%d%H%M")
    # debug_docker_tag = ROAD_MAPPING_TAG + "_" + time_now
    
    collection_dir = os.path.join(debug_evaluation_dir, debug_docker_tag)
    input_dir = os.path.join(collection_dir, "input")
    output_dir = os.path.join(collection_dir, "output")
    debug_dir = os.path.join(collection_dir, "debug")

    # 1 解压生成的 shp 文件
    for key, value in lukou_ids.items():
        data_type = key.split('/')[0]
        lukou_name = key.split('/')[1]
        # 保存调试图片
        debug_dir_bak = os.path.join(debug_dir, ROAD_MAPPING_TAG, data_type, lukou_name)
        add_folder_if_no_exist(debug_dir_bak)

        cp_dst_upload_path_root = os.path.join(workspace_folder, f"{key}/output/upload")
        zip_debug_file = "road_model_"+ROAD_MAPPING_TAG+'.zip'
        zip_debug_file_src = os.path.join(cp_dst_upload_path_root, zip_debug_file)
        if os.path.exists(zip_debug_file_src):
            shutil.copyfile(zip_debug_file_src, os.path.join(debug_dir_bak, zip_debug_file))
            cmd_str = "./script/unzip_file.sh " + zip_debug_file + " "  + debug_dir_bak
            if run_a_process(cmd_str, 3600*5) is None: # 正常为 None
                print("successful save debug info")

        # 保存shp文件
        unzip_dir = os.path.join(input_dir, ROAD_MAPPING_TAG, data_type, lukou_name)
        add_folder_if_no_exist(unzip_dir)

        zip_map_file = "map_shp_"+ROAD_MAPPING_TAG+'.zip'
        cp_dst_upload_path = os.path.join(workspace_folder, f"{key}/output/upload/map")
        cmd_str = "./script/unzip_file.sh " + os.path.join(cp_dst_upload_path, zip_map_file) + " "  + unzip_dir
        if run_a_process(cmd_str, 3600*5) is None: # 正常为 None
            pass

    # 2 拷贝评估的配置文件到 input 目录
    evaluate_config_file = os.path.join(input_dir, "config.yaml")
    shutil.copyfile("./data/evaluate_config_v2.yaml", evaluate_config_file)

    # 3 更新版本号
    cmd_str2 = "./script/evaluation/change_evaluate_version.sh " + evaluate_config_file + " " + ROAD_MAPPING_TAG
    if run_a_process(cmd_str2, 3600*5) is None: # 正常为 None

        # 4 开始进行评估
        add_folder_if_no_exist(output_dir)
        cmd_str3 = "./script/evaluation/docker_run_evaluation.sh " + input_dir + " " + output_dir + " " + str(debug_run_evaluation)
        if run_a_process(cmd_str3, 3600*5) is None: # 正常为 None
            print("successrun evaluation")


def check_results(workspace_folder, lukou_ids, debug_docker_tag, debug_run_evaluation, debug_evaluation_dir):
    # 用于保存所有路口的结果
    results = []
    check_result_flag = True
    for key, value in lukou_ids.items():
        output_path = os.path.join(workspace_folder, f"{key}/output/output.json")
        data_type = key.split('/')[0]
        lukou_name = key.split('/')[1]
        # 检查文件是否存在
        if os.path.exists(output_path):
            try:
                with open(output_path, 'r') as f:
                    output_data = json.load(f)
                
                result_string = f'{data_type} : {lukou_name}  :  {output_data} \n'
                check_result_flag = check_result_flag and output_data["status"]

                results.append(result_string)
            except json.JSONDecodeError:
                print(f"Error decoding JSON in file: {output_path}")
        else:
            print(f"File not found: {output_path}")
    
    # 保存所有结果到 check_results.json
    check_results_path = os.path.join(workspace_folder, "check_results.txt")
    with open(check_results_path, 'w') as output_file:
        for result in results:
            output_file.write(result) 
    
    if debug_run_evaluation > 0:
        process_evaluation(workspace_folder, lukou_ids, debug_docker_tag, debug_run_evaluation, debug_evaluation_dir)

    print("=================================================")
    print("===================check_results=================")
    print("=================================================")
    for result in results:
        print(result)

    print("=================================================")
    if check_result_flag == True:
        print("✔️  success ✔️")
    else:
        print("❌  Failed ❌")
    print("=================================================")
    print(f"check results saved to  : {check_results_path}")


class TaskArg(object):
    def __init__(self) -> None:
        self.workspace_folder = ""
        self.task_id = ""  # task id is str type
        self.task_type = ""
        self.task_folder = ""
        self.info_json_path = ""
        self.info_json = {}
        self.tile_branch = ""
        self.road_branch = ""
        self.tile_server = ""
        self.road_server = ""
        self.only_download = 0

        self.utm_num = 0  # 任务内统一使用的 UTM 带号
        self.t_utm_world = []  # world 系原点的utm坐标，[x,y,0]，world系是以任务区域中心为原点的局部坐标系
        self.diff_match = 0  # -1 重定位崩溃, 0 重定位失败, 1 重定位成功
        self.tile_id_list = []  # 资料上传的 tile
        self.start_stamp_ms = 0  # 任务启动时间戳
        self.data_meta = {}  # trail 与 link 数据元信息
        self.log = {"数据传输用时_m": 0}

        self.offline = 0  # 0 默认线上， 1 线下版本
        self.x_min = 0
        self.x_max = 0
        self.y_min = 0
        self.y_max = 0
        self.z_max = 0

        self.bev_data_folder = ""
        self.utm_frame_out = 0
        self.offline_flag = 1


#####################################################################################################
#####################################################################################################
#####################################################################################################
default_branch = time.strftime('test_%d%H%M%S', time.localtime())
default_tile_server = "172.21.26.28:8082"
default_road_server = "172.21.133.164:8081"
default_only_download = 0

# 下面目前非必须
default_task_type = "zb_auto"
default_info_json_path = project_root_folder+"/data/task_info_demo.json"
default_polygon_json_path = project_root_folder+"/data/polygon.json"

# 下面的是必须的
#cxf
# default_workspace_folder = "/mnt/c/02data/01zb_mapping/0605_10lukou"
default_workspace_folder = "/mnt/c/02data/01zb_mapping/0605_10lukou"
default_run_stage = 3 # 0:所有，1：拼图，2：矢量化，3：拓扑，4: 交通灯, 5:评测
default_topo_version = 1  # 0:旧版本 1：新版本

default_force_task_type = 0 # 调试使用,和另一个参数task_type意义不一样： 0：使用tasks.json中的类型，1: bev, 2: lidar, 3: bev & lidar 融合
default_debug_ldiar_bev_mapping = 0 # 0：不生成， 1: 在task type = 3 时，且存在bev资料，则用激光pose+BEV资料生成bev地图, 2: 融合前生成纯激光的地图， 3：分别生成bev 和 激光地图
default_debug_docker_tag="" # debug: !!不允许为空!!!，这个变量会用来创建目录，生成各个版本的评估结果; release: 不需要
default_debug_run_evaluation=0 # 0: 不评测， 1： 仅评测， 2：嵌入到主流程评测(默认)
default_debug_evaluation_dir="/data/dataset/4_auto_model/all_results" # "评估结果的保存位置"

# default_workspace_folder = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ziyan"
# default_debug_docker_tag="v3.01.03_qzc_03-01_11_07_22" # debug: !!不允许为空!!!，这个变量会用来创建目录，生成各个版本的评估结果; release: 不需要
# default_debug_run_evaluation=1 # 0: 不评测， 1： 仅评测， 2：嵌入到主流程评测(默认)
# default_debug_evaluation_dir="/mnt/d/01_code/04_dataset/dilabel/crowd_source/to_test/test_docker" # "评估结果的保存位置"
########################################## 以下非必须，无需修改 #################################################
default_debug_model = 0  #联调为0，本地自测用为1
default_rm_output = 1   #默认为1，如果run从0开始跑则，先删除output

parser = argparse.ArgumentParser()
parser.add_argument('--info_json_path', type=str, default=default_info_json_path)
parser.add_argument('--polygon_json_path', type=str, default=default_polygon_json_path)
parser.add_argument('--workspace_folder', type=str, default=default_workspace_folder)
# branch名称使用顺序：命令行传入 > 默认随机值
parser.add_argument('--tile_branch', type=str, default=default_branch)
parser.add_argument('--road_branch', type=str, default=default_branch)
# 服务器地址使用顺序：命令行传入 > 任务json传入 > 默认值
parser.add_argument('--tile_server', type=str, default=default_tile_server)
parser.add_argument('--road_server', type=str, default=default_road_server)
parser.add_argument('--task_type', type=str, default=default_task_type)
parser.add_argument('--force_task_type', type=int, default=default_force_task_type)
parser.add_argument('--bev_data_folder', type=str, default="")  # 没用
parser.add_argument('--dir_flag', type=int, default=0)  # 没用
parser.add_argument('--label_flag', type=str, default="label")  # 没用
parser.add_argument('--only_download', type=int, default=default_only_download)
parser.add_argument('--x_min', type=float, default=0)
parser.add_argument('--y_min', type=float, default=-50)
parser.add_argument('--x_max', type=float, default=50)
parser.add_argument('--y_max', type=float, default=50)
parser.add_argument('--z_max', type=float, default=0.2)
parser.add_argument('--utm_frame_out', type=int, default=1, help="单帧utm点云输出")  # 是否输出utm下的点云
parser.add_argument('--offline_flag', type=int, default=1, help="离线flag")
parser.add_argument('--run_stage', type=int, default=default_run_stage, help="拼图、矢量化、拓扑分段运行")
parser.add_argument('--use_opt_pose', type=int, default=1, help="是否使用 上游优化的pose")
parser.add_argument('--debug_model', type=int, default=default_debug_model, help="是否使用debug测试")
parser.add_argument('--debug_docker_tag', type=str, default=default_debug_docker_tag, help="是否使用评估")
parser.add_argument('--debug_run_evaluation', type=int, default=default_debug_run_evaluation, help="是否使用评估")
parser.add_argument('--debug_evaluation_dir', type=str, default=default_debug_evaluation_dir, help="评估结果的保存位置")
parser.add_argument('--default_topo_version', type=int, default=default_topo_version, help="topo 版本，临时")
parser.add_argument('--debug_lidar_bev_mapping', type=int, default=default_debug_ldiar_bev_mapping, help="task_type=3时,用激光pose+BEV建图")

lukou_ids = {
    ########################################################################
    ######################## 自研数据 #######################################
    ########################################################################
    # bev
    # "bev_byd/503601/auto_label": [
    #     # "402301"
    # ],
    # "lidar_byd/503601/auto_label": [
    #     "402301",
    #     "402275"
    # ],

    # # bev
    # "bev_byd/503601/auto_label": [
    #     # "402301"
    # ],
    # "bev_byd/505096/auto_label": [
    # ],
    # "bev_byd/503565/auto_label": [
    # ],
    # "bev_byd/504341/auto_label": [
    # ],
    # "bev_byd/503576/auto_label": [
    # ],

    # lidar
    # "lidar_byd/503601/auto_label": [
    #     # "402314"
    #     # "402310"
    #     # "402338"
    #     # "402275"
    #     # "402301"
    # ],
    "lidar_byd/505096/auto_label": [
    ],
    "lidar_byd/503565/auto_label": [
    ],
    "lidar_byd/504341/auto_label": [
    ],
    "lidar_byd/503576/auto_label": [
    ],

    ########################################################################
    ######################## mmt_rc 数据 ###################################
    ########################################################################
}

lukou_ids0= {
    ########################################################################
    ######################## 自研数据 #######################################
    ########################################################################
    # bev
    "bev_byd/503601/auto_label": [
        # "402301"
    ],
    "bev_byd/505096/auto_label": [
    ],
    "bev_byd/503565/auto_label": [
    ],
    "bev_byd/504341/auto_label": [
    ],
    "bev_byd/503576/auto_label": [
    ],

    # lidar
    "lidar_byd/503601/auto_label": [
        # "402314"
        # "402310"
        # "402338"
        # "402275"
        # "402301"
    ],
    "lidar_byd/505096/auto_label": [
    ],
    "lidar_byd/503565/auto_label": [
    ],
    "lidar_byd/504341/auto_label": [
    ],
    "lidar_byd/503576/auto_label": [
    ],

    ########################################################################
    ######################## mmt_rc 数据 ###################################
    ########################################################################
}

lukou_ids0 = {
    "byd_gf/8647/model/auto_label": [
    ],
    "lidar_data/blidar_0315/model/auto_label": [
    ],
    # "lidar_data/10010_mmt_lidar/model/auto_label": [
    # ],

    ########################################################################
    ######################## mmt_rc 数据 ###################################
    ########################################################################
    ############ 氢 ############
    # "mmt_rc/batch-10011/model/auto_label": [
    # ],
    ############ 氦 ############, 有真值
    "mmt_rc/batch-10010/model/auto_label": [
    ],
    ############ 锂 ############, 有真值
    "mmt_rc/batch-10017/model/auto_label": [
    ],
    ############ 铍 ############, 有真值
    "mmt_rc/batch-286/model/auto_label": [
    ],
    ############ 硼 ############, 有真值
    "mmt_rc/batch-195/model/auto_label": [
    ],
    ############ 碳 ############
    # "mmt_rc/batch-10004/model/auto_label": [
    # ],
    ############ 氮 ############
    # "mmt_rc/batch-290/model/auto_label" : [
    # ],
    ############ 氧 ############, 有真值
    "mmt_rc/batch-312/model/auto_label": [
    ],
    ############ 氟 ############
    # "mmt_rc/batch-323/model/auto_label": [
    # ],
    ########### 氖 ############, 有真值
    "mmt_rc/batch-10013/model/auto_label": [
    ],
    # # ########## 联调测试数据 ##########
    # "mmt_rc/mnt/auto_label": [
    # ],

    ########################################################################
    ######################## 自研数据 #######################################
    ########################################################################
    ############ 氦 ############, 有真值
    "byd/batch-10010/model/auto_label": [
    ],
    # ############ 锂 ############, 有真值
    "byd/batch-10017/model/auto_label": [
    ],
    ############ 铍 ############, 有真值
    "byd/batch-286/model/auto_label": [
    ],
    # ############ 硼 ############, 有真值
    "byd/batch-195/model/auto_label": [
    ],
}

def rm_dir(traj_folder):
    # 判断目录是否存在
    if os.path.exists(traj_folder):
        # 删除目录及其内容
        try:
            shutil.rmtree(traj_folder)
            print(f"rm:  {traj_folder} ")
        except Exception as e:
            print(f"Error rm: {traj_folder}: {e}")
    else:
        print(f"Directory {traj_folder} does not exist.")

def rm_output_results(to_del_lukou_ids, args_, default_rm_output):
    #删除output目录
    if default_rm_output and args_.run_stage == 0:
        total_lukou_cnt = len(to_del_lukou_ids)
        print("total_lukou_num:{}, cpu_count:{}".format(total_lukou_cnt, cpu_count()))
        # p = Pool(cpu_count())

        cpu_num = len(to_del_lukou_ids)
        if cpu_count() < cpu_num:
            cpu_num = cpu_count()        
        p = Pool(cpu_num)
        for key, value in to_del_lukou_ids.items():
            traj_folder = os.path.join(args_.workspace_folder, key, "output")
            p.apply_async(rm_dir, args=(traj_folder, ))
        p.close()
        p.join()



if __name__ == "__main__":
    args_ = parser.parse_args()
    print("debug_docker_tag:", args_.debug_docker_tag)
    print("debug_run_evaluation:", args_.debug_run_evaluation)
    print("debug_evaluation_dir:", args_.debug_evaluation_dir)
    
    if args_.debug_docker_tag == "" and args_.debug_run_evaluation > 0:
        exit("!!! debug_docker_tag is None, This is necessary !!!")

    if args_.debug_run_evaluation == 1:
        check_results(args_.workspace_folder, lukou_ids, args_.debug_docker_tag, args_.debug_run_evaluation, args_.debug_evaluation_dir)

    else:
        # 如果run_stage从0开始跑，那么就删除旧数据
        rm_output_results(lukou_ids, args_, default_rm_output)

        for key, value in lukou_ids.items():
            print(key, value)
            traj_folder = os.path.join(args_.workspace_folder, key)
            run_secific_trajs = value
            debug_model = args_.debug_model
            data_type = multi_traj_mapping.get_data_type(traj_folder, args_.force_task_type)
            if data_type == "":
                pass
            log.info("data_type: ", data_type)

            # TODO:qzc ziyan debug close
            # 强制使用 bev pose
            # if data_type in [DataType.MMT_RC.name, DataType.BYD_LIDAR_B.name]:
            #     args_.use_opt_pose = 1
            # else: # DataType.BYD_BEV.name
            #     args_.use_opt_pose = 0
            # args_.use_opt_pose = 1

            multi_traj_mapping.run(args_.info_json_path,
                                traj_folder,
                                args_.tile_branch,
                                args_.road_branch,
                                args_.tile_server,
                                args_.road_server,
                                args_.task_type,
                                args_.dir_flag,
                                args_.label_flag,
                                args_.only_download,
                                args_.x_min,
                                args_.x_max,
                                args_.y_min,
                                args_.y_max,
                                args_.z_max,
                                args_.bev_data_folder,
                                args_.utm_frame_out,
                                args_.offline_flag,
                                args_.polygon_json_path,
                                args_.run_stage,
                                args_.use_opt_pose,
                                data_type,
                                debug_model,
                                run_secific_trajs,
                                args_.debug_run_evaluation,
                                args_.default_topo_version,
                                args_.debug_lidar_bev_mapping,
                                )
        
        # check 7 lukou output/output.json
        # "195": {}

        check_results(args_.workspace_folder, lukou_ids, args_.debug_docker_tag, args_.debug_run_evaluation, args_.debug_evaluation_dir)

