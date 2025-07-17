import sys
import json
import os
import socket
import time
import pyproj
import argparse
import shapely.ops
import shapely.affinity
from shapely.geometry import Polygon, LineString, GeometryCollection
from multiprocessing import Pool
from collections import defaultdict, OrderedDict

os.environ['PROJ_LIB'] = '/usr/share/proj'

project_root_folder = os.path.abspath(__file__+"/../../")
task_type_name_dict = {"zl": "连续直路", "lk2": "离散路口", "zb_manual": "众包人工", "zb_auto": "众包自动", "zb_auto_zc": "自采自动"}

sys.path.append(project_root_folder)
import common.util.util as util
from common.util.util import *
from common.data.error_code import ErrorCode
from data_io.script.util import get_utm_num_from_gps_polygon_wkt
from road_model.script.tran_imgpos_coor import trans_imgpos
from preprocess.script.util import log
from data_io.script.bev_label_map import DataType


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

        self.offline = 0 # 0 默认线上， 1 线下版本
        self.x_min = 0 
        self.x_max = 0 
        self.y_min = 0 
        self.y_max = 0 
        self.z_max = 0 

        self.bev_data_folder =""
        self.utm_frame_out = 0
        self.offline_flag = 1

def make_result_json(task_arg: TaskArg, error_code: ErrorCode):
    try:
        task_type_name = task_type_name_dict[task_arg.task_type] if task_arg.task_type in task_type_name_dict.keys() else "未知类型"
        host_name = f"✔️{socket.gethostname()}" if error_code == ErrorCode.中业运行正常 else f"❌{socket.gethostname()}"
        version = util.read_first_line_from_file(os.path.join(project_root_folder, "CHANGELOG.md"))
        middle = task_arg.info_json["middle"] if "middle" in task_arg.info_json.keys() else {}
        task_arg.log["trail_num"] = len(task_arg.data_meta["trail_dict"])
        task_arg.log["link_num"] = len(task_arg.data_meta["link_dict"])
        task_arg.log["cost_minute"] = round((int(round(time.time() * 1000)) - task_arg.start_stamp_ms) / 60000, 2)
        task_arg.log["task_length_m"] = round(task_arg.data_meta["task_length_m"], 2)
        task_arg.log["process_km_h"] = round((task_arg.data_meta["task_length_m"]/(task_arg.log["cost_minute"]+0.001))*0.06, 1)
        sys_log={}
        if os.path.exists(os.path.join(task_arg.task_folder, "debug/sys_monitor_log.txt")):
            sys_log = util.json_file_to_dict(os.path.join(task_arg.task_folder, "debug/sys_monitor_log.txt"))
            task_arg.log["ram_max_g"] = sys_log["container"]["ram_max"]
            del sys_log["realtime"]
        task_arg.log["算法处理用时_m"] = round(task_arg.log["cost_minute"]-task_arg.log["数据传输用时_m"], 2)
        task_arg.log["数据传输用时占比"] = round(task_arg.log["数据传输用时_m"]/(task_arg.log["cost_minute"]+0.001), 2)
        task_arg.log["proj_root"] = project_root_folder
        task_arg.log["start_stamp_ms"] = task_arg.start_stamp_ms
        backload_tile_id_set = set(task_arg.tile_id_list)
        for trail_info in task_arg.data_meta["trail_dict"].values():
            backload_tile_id_set.add(int(trail_info["download_tile_id"]))
        backload_tile_id_str = str(list(backload_tile_id_set)).replace(" ", "")
        task_arg.log["backload_cmd"] = f"python3 ./common/script/backload.py --download_folder=$HOME/lidar_mapping_ws/{task_arg.task_id}_backload --tile_branch={task_arg.tile_branch} --tile_id_list={backload_tile_id_str} --trail_id={task_arg.start_stamp_ms} --road_branch={task_arg.road_branch} --utm_num={task_arg.utm_num} --tile_server={task_arg.tile_server} --road_server={task_arg.road_server}"
    except Exception as e:
        print(e)

    result_json: dict = {
        f"fsdmap-{task_type_name}": host_name,
        "task_id": task_arg.task_id,
        "version": version,
        "tile_branch": task_arg.tile_branch,
        "road_branch": task_arg.road_branch,
        "tile_id_list": task_arg.tile_id_list,
        "error_code": error_code.value,
        "error_text": error_code.name,
        "diff_match": task_arg.diff_match,
        "tile_server": task_arg.tile_server,
        "road_server": task_arg.road_server,
        "utm_num": task_arg.utm_num,
        "t_utm_world": task_arg.t_utm_world,
        "log": task_arg.log,
        "middle": middle,
        "sys_log": sys_log,
    }
    # print(json.dumps(result_json, ensure_ascii=False))
    util.dict_to_json_file(os.path.join(task_arg.task_folder, "result.json"), result_json)


def save_result(task_arg: TaskArg, error_code: ErrorCode):
    LATEST_VERSION="v3.9.0 2025-07-09:(new:V3.9.0_20250715_mapping_autolabeling_v3.9.0_base:v3.8.9_qzc_07-09_10_45_53)"
    ROAD_MAPPING_TAG="v3.9.0"

    # 保存debug日志
    cp_dst_upload_path_root = os.path.join(task_arg.task_folder, "upload")
    add_folder_if_no_exist(cp_dst_upload_path_root)
    cp_src_path = os.path.join(task_arg.task_folder, "bev_mapbuild_out")
    zip_debug_file = "road_model_"+ROAD_MAPPING_TAG+'.zip'
    cmd_str = "./script/zip_file.sh " + os.path.join(cp_dst_upload_path_root, zip_debug_file) + " "  + cp_src_path
    if run_a_process(cmd_str, 3600*5) is not None:
        # return ErrorCode.激光建图_超时退出
        print("save debug file failed!")

    if error_code != ErrorCode.中业运行正常:
        check_status  = -1
        # return_error_code = ErrorCode(download_result["error_code"])
        check_message = error_code.name
        print("status: {}, messages:{}".format(check_status, check_message))
        output_result_data = {
            "status": check_status, # -1 : 失败， 1 ： 成功
            "message": check_message
        }

        output_result_file = os.path.join(task_arg.task_folder, "output.json")
        util.dict_to_json_file(output_result_file, output_result_data)
        return

    # 输出数据
    check_status = 0
    check_message = str(error_code.value)

    check_file1 = os.path.join(task_arg.task_folder, "bev_mapbuild_out/export_to_shp/lane_boundary.dbf")
    check_file2 = os.path.join(task_arg.task_folder, "bev_mapbuild_out/export_to_shp/road_boundary.dbf")
    if os.path.exists(check_file1) and os.path.exists(check_file2):
        file_size1 = os.path.getsize(check_file1)
        file_size2 = os.path.getsize(check_file2)
        check_status =  int(file_size1 > 1024 or file_size2 > 1024)
    else:
        print("check_file1 is not exist", check_file1)
        print("check_file2 is not exist", check_file2)

    if check_status == 1:
        cp_dst_upload_path = os.path.join(task_arg.task_folder, "upload", "map")
        add_folder_if_no_exist(cp_dst_upload_path)
        cp_src_path = os.path.join(task_arg.task_folder, "bev_mapbuild_out/export_to_shp")

        ########### 交通灯 ###########
        post_fixs = [".shp", ".shx", ".prj", ".dbf", ".cpg"]
        for post_fix in post_fixs:
            src_trafic_light = os.path.join(task_arg.task_folder, "trafficlight_processing", "traffic_light"+post_fix)
            dst_trffic_light = os.path.join(cp_src_path, "traffic_light"+post_fix)
            if os.path.exists(src_trafic_light):
                shutil.copyfile(src_trafic_light, dst_trffic_light)
        ########### 交通灯 ###########

        zip_map_file = "map_shp_"+ROAD_MAPPING_TAG+'.zip'
        cmd_str = "./script/zip_file.sh " + os.path.join(cp_dst_upload_path, zip_map_file) + " "  + cp_src_path
        if run_a_process(cmd_str, 3600*5) is not None:
            # return ErrorCode.激光建图_超时退出
            check_message = "shp file is empty"

        # TODO:qzc save data
        # cp_dst_path = os.path.join(task_arg.task_folder, "map_middle")
        # add_folder_if_no_exist(cp_dst_path)
        # util.copy_folder(cp_src_path, cp_dst_path)
    else:
        check_message = "shp file is empty"

    cp_dst_upload_path = os.path.join(task_arg.task_folder, "upload", "cloud_map")
    add_folder_if_no_exist(cp_dst_upload_path)
    util.move_file(os.path.join(task_arg.task_folder, "model_pcd/pointcloud_semantic_pure_lidar_gcj02.pcd"), os.path.join(cp_dst_upload_path, "pointcloud_semantic_pure_lidar_gcj02.pcd"))
    util.move_file(os.path.join(task_arg.task_folder, "model_pcd/pointcloud_semantic_gcj02.pcd"), os.path.join(cp_dst_upload_path, "pointcloud_semantic_gcj02.pcd"))
    util.move_file(os.path.join(task_arg.task_folder, "model_pcd/global_cloud.pcd"), os.path.join(cp_dst_upload_path, "global_cloud.pcd"))

    print("status: {}, messages:{}".format(check_status, check_message))
    output_result_data = {
        "status": check_status, # -1 : 失败， 1 ： 成功
        "message": check_message
    }

    output_result_file = os.path.join(task_arg.task_folder, "output.json")
    util.dict_to_json_file(output_result_file, output_result_data)


def run_cmd(cmd):
    print(cmd)
    os.system(cmd)

def pwyh_info(task_arg: TaskArg) -> ErrorCode:
    log.info("=== 字段检查 ===")
    if "links" not in task_arg.info_json["middle"].keys():
        log.error("无任务link信息")
        return ErrorCode.无任务link信息

    log.info("=== 任务类型 ===")
    if "task_type" not in task_arg.info_json["middle"].keys():
        task_arg.task_type = "zl"
    elif task_arg.info_json["middle"]["task_type"] == "":
        task_arg.task_type = "zl"
    else:
        task_arg.task_type = task_arg.info_json["middle"]["task_type"]
    log.info("task_arg.task_type:", task_arg.task_type)

    log.info("=== 任务坐标系 ===")
    # task_arg.utm_num = get_utm_num_from_gps_wkt(task_arg.info_json["middle"]["links"][0]["task_geom"])
    task_arg.utm_num = task_arg.info_json['utm_num']
    proj_transform = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:326{task_arg.utm_num}", always_xy=True).transform
    # task_utm_poly: Polygon = Polygon()
    # # task_utm_poly1: Polygon = Polygon()
    # if ("task_geom" in task_arg.info_json["middle"].keys()):  # 对于有 task_geom 字段的，使用 task_geom 字段作为任务范围
    #     task_utm_poly = shapely.ops.transform(proj_transform, shapely.wkt.loads(task_arg.info_json["middle"]["task_geom"]))
    # else:  # 对于有 links 字段的，使用 links 范围合成任务范围
    #     for one_link in task_arg.info_json["middle"]["links"]:
    #         new_link_polygon = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["task_geom"])).buffer(0.01)
    #         task_utm_poly = task_utm_poly.union(new_link_polygon)
    #         # task_utm_poly = task_utm_poly.union(shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["task_geom"])))
    #         # task_utm_poly1 = task_utm_poly1.cascaded_union(shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["task_geom"])))
    # # TODO:qzc
    # task_utm_poly = task_utm_poly.convex_hull
    task_arg.t_utm_world = task_arg.info_json["t_utm_world"]
    # task_world_poly = shapely.affinity.translate(task_utm_poly, -task_arg.t_utm_world[0], -task_arg.t_utm_world[1])
    log.info("task_arg.utm_num:", task_arg.utm_num)
    log.info("task_arg.t_utm_world:", task_arg.t_utm_world)

    log.info("=== 数据元信息 ===")
    task_arg.data_meta = {"trail_dict": {},
                          "link_dict": {},
                          "task_geom_local": "",
                          "task_length_m": 0,
                          "utm_num": task_arg.utm_num,
                          "t_utm_world": task_arg.t_utm_world}
    all_link_world_ls = LineString()
    for one_link in task_arg.info_json["middle"]["links"]:
        link_utm_ls = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["link_geom"]))
        link_world_ls = shapely.affinity.translate(link_utm_ls, -task_arg.t_utm_world[0], -task_arg.t_utm_world[1])
        all_link_world_ls = all_link_world_ls.union(link_world_ls)
        link_id = str(one_link["link_id"])
        task_arg.data_meta["link_dict"][link_id] = {"link_id": one_link["link_id"],
                                                    "line_string": link_world_ls.wkt,
                                                    "link_direction": one_link["link_direction"]}
        for one_trail in one_link["tracks"]:
            trail_id = str(one_trail["trail_id"])
            if trail_id not in task_arg.data_meta["trail_dict"].keys():
                task_arg.data_meta["trail_dict"][trail_id] = {"trail_id": one_trail["trail_id"],
                                                              "download_tile_id": one_trail["tile_id"],
                                                              "download_branch": one_trail["branch"],
                                                              "link_id_list": [one_link["link_id"]],
                                                              "trail_name": one_trail["trail_name"]
                                                              }
            else:
                task_arg.data_meta["trail_dict"][trail_id]["link_id_list"].append(one_link["link_id"])
    # all_link_in_task_world_ls = all_link_world_ls.intersection(task_world_poly)
    # task_arg.data_meta["task_length_m"] = all_link_in_task_world_ls.length

    # proj_transform_inverse = pyproj.Transformer.from_crs(f"EPSG:326{task_arg.utm_num}", "EPSG:4326", always_xy=True).transform
    # all_link_in_task_world_ls2 = shapely.affinity.translate(all_link_in_task_world_ls.buffer(50), task_arg.t_utm_world[0], task_arg.t_utm_world[1])
    # all_link_in_task_world_ls_inverse = shapely.ops.transform(proj_transform_inverse, all_link_in_task_world_ls2)

    # task_geom_global = all_link_in_task_world_ls_inverse.wkt # gcj02, "POLYGON ((xxxx))"
    # task_geom_local = all_link_in_task_world_ls.buffer(50).wkt # utm local, "POLYGON ((xxxx))"

        
    task_arg.data_meta["task_geom_local"] = task_arg.info_json["middle"]["task_geom_local_expand"]


    # if not task_arg.info_json["middle"]["site_center"]:
    #     cur_links1 = shapely.wkt.loads(task_arg.data_meta["task_geom_local"])

    #     # 计算几何中心
    #     centroid = cur_links1.centroid
    #     task_arg.info_json["middle"]["site_center"] = [centroid.x, centroid.y]
    #     print(f"几何中心点坐标: {task_arg.info_json['middle']['site_center']}")

    log.info("任务 trail 数:", len(task_arg.data_meta["trail_dict"]))

    log.info("=== 输出调试文件 ===")
    # # shapely_to_obj(all_link_world_ls, os.path.join(task_arg.task_folder, f"debug/all_link_geom.obj"))
    # shapely_to_obj(task_world_poly, os.path.join(task_arg.task_folder, f"debug/task_geom.obj"))
    # shapely_to_obj(all_link_in_task_world_ls, os.path.join(task_arg.task_folder, f"debug/data_scope_origin.obj"))
    # shapely_to_obj(all_link_in_task_world_ls.buffer(50), os.path.join(task_arg.task_folder, f"debug/data_scope_buffer50.obj"))
    # for one_link in task_arg.info_json["middle"]["links"]:
    #     link_utm_ls = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["link_geom"]))
    #     link_world_ls = shapely.affinity.translate(link_utm_ls, -task_arg.t_utm_world[0], -task_arg.t_utm_world[1])
    #     shapely_to_obj(link_world_ls, os.path.join(task_arg.task_folder, f"debug/link-{one_link['link_id']}.obj"))
    return ErrorCode.中业运行正常


def init_task(info_json_path: str, workspace_folder: str, tile_branch: str, road_branch: str, tile_server: str, road_server: str,  only_download , x_min, x_max, y_min, y_max, z_max,bev_data_folder,utm_frame_out,offline_flag, polygon_json_path):
    task_arg: TaskArg = TaskArg()
    try:
        task_arg.tile_branch = tile_branch
        task_arg.road_branch = road_branch
        task_arg.workspace_folder = workspace_folder
        task_arg.info_json_path = info_json_path
        task_arg.only_download = only_download
        task_arg.info_json = json_file_to_dict(info_json_path)

        # 服务器地址使用顺序：命令行传入 > 任务json传入 > 默认值
        if tile_server != "":
            task_arg.tile_server = tile_server
        elif "tile_server" in task_arg.info_json.keys():
            task_arg.tile_server = task_arg.info_json["tile_server"]
        else:
            task_arg.tile_server = default_tile_server
        if road_server != "":
            task_arg.road_server = road_server
        elif "road_server" in task_arg.info_json.keys():
            task_arg.road_server = task_arg.info_json["road_server"]
        else:
            task_arg.road_server = default_road_server
        task_arg.task_id = str(task_arg.info_json["task_id"])
        task_arg.task_folder = os.path.join(workspace_folder, task_arg.task_id)
        task_arg.tile_branch = tile_branch
        task_arg.road_branch = road_branch
        task_arg.start_stamp_ms = int(round(time.time() * 1000))

        task_arg.x_min = x_min 
        task_arg.x_max = x_max 
        task_arg.y_min = y_min 
        task_arg.y_max = y_max 
        task_arg.z_max = z_max
        task_arg.bev_data_folder = bev_data_folder
        task_arg.utm_frame_out = utm_frame_out
        task_arg.offline_flag = offline_flag
        task_arg.polygon_json_path = polygon_json_path

        if "data_source" in task_arg.info_json["middle"].keys():
            task_arg.data_source = task_arg.info_json["middle"]["data_source"]
        elif "data_source" in task_arg.info_json.keys():
            task_arg.data_source = task_arg.info_json["data_source"]
        else:
            task_arg.data_source = 0
        task_arg.info_json["middle"]["data_source"] = task_arg.data_source

        prase_status = pwyh_info(task_arg)
        json_wgs_wkt_simple(task_arg.info_json["middle"])  # 化简 wkt 字符串

        dict_to_json_file(os.path.join(task_arg.task_folder, "task_info.json"), task_arg.info_json)
        dict_to_json_file(task_arg.info_json_path, task_arg.info_json)
        dict_to_json_file(os.path.join(task_arg.task_folder, "data_meta.json"), task_arg.data_meta)

        make_result_json(task_arg, prase_status)
        return ErrorCode.中业运行正常, task_arg
    except Exception as e:
        log.error("任务初始化失败:", e)
        return ErrorCode.任务启动失败, task_arg


def run_data_download(task_arg: TaskArg) -> int:
    download_st = time.time()
    # add_folder_if_no_exist(os.path.join(task_arg.task_folder, "backload_data"))
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["数据下载"])
    tile_download_folder = os.path.join(task_arg.task_folder, "tile_download")
    dict_to_json_file(os.path.join(tile_download_folder, "download_info.json"), list(task_arg.data_meta["trail_dict"].values()))
    if task_arg.only_download == 1:
        cmd = "python3 {} --download_info_json={} --download_folder={} --tile_server={} --specific_types={}".format(
            os.path.join(project_root_folder, "data_io/script/data_download.py"),
            os.path.join(tile_download_folder, "download_info.json"),
            tile_download_folder,
            task_arg.tile_server,
            "loc,nav_imu,nav_ins,ndm_location,odometry,odometry3d,perception_raw,lidar,f_ca_120,vehicle_status,imgpos,bev_label,bevlane_f_ca_120,perception_laneline,bevlane_perception_f_ca_120,bevlane_semantic_mask_f_ca_120")
        if run_a_process(cmd, 3600*5) is not None:
            return ErrorCode.数据下载_超时退出  # 数据下载-超时退出
        entry = os.path.join(project_root_folder, "data_io/script/load_optpose.py")
        cmd_down_opt_pose = "python3 {} --download_folder={} --info_json_path={}".format(
            entry,
            os.path.join(task_arg.task_folder, "backload_data"),
            task_arg.info_json_path
        ) 
        print(cmd_down_opt_pose)
        os.system(cmd_down_opt_pose)

        return ErrorCode.中业运行正常
    else:
        cmd = "python3 {} --download_info_json={} --download_folder={} --tile_server={} --specific_types={} --use_opt_pose={} --data_type={} --debug_model={}".format(
            os.path.join(project_root_folder, "data_io/script/data_download_qzc.py"),
            os.path.join(tile_download_folder, "download_info.json"),
            tile_download_folder,
            task_arg.tile_server,
            "loc,nav_imu,nav_ins,ndm_location,odometry,odometry3d,perception_raw,lidar,f_ca_120,vehicle_status,imgpos,bev_label,bevlane_f_ca_120,perception_laneline,bevlane_perception_f_ca_120,bevlane_semantic_mask_f_ca_120",
            int(task_arg.use_opt_pose),
            task_arg.data_type,
            task_arg.debug_model
            )
        if run_a_process(cmd, 3600*5) is not None:
            return ErrorCode.数据下载_超时退出  # 数据下载-超时退出
    download_result = json_file_to_dict(os.path.join(tile_download_folder, "download_result.json"))
    if download_result == {}:
        return ErrorCode.数据下载_崩溃退出  # 数据下载-崩溃退出
    return_error_code = ErrorCode(download_result["error_code"])
    task_arg.log["download_time_s"] = round(time.time()-download_st)
    # task_arg.log["download_size_g"] = round(util.get_folder_size_GB(os.path.join(tile_download_folder)), 2)
    task_arg.log["download_size_g"] = 0
    task_arg.log["数据传输用时_m"] = round((time.time()-download_st)/60, 2)
    # entry = os.path.join(project_root_folder, "data_io/script/load_optpose_1.py")
    # cmd_down_opt_pose = "python3 {} --download_folder={} --info_json_path={} --tile_server={}".format(
    #     entry,
    #     os.path.join(task_arg.task_folder, "backload_data"),
    #     task_arg.info_json_path,
    #     task_arg.tile_server
    # ) 
    # print(cmd_down_opt_pose)
    # os.system(cmd_down_opt_pose)

    return return_error_code


def run_data_preprocess(task_arg: TaskArg) -> int:
    sparse_data = False
    # sparse_data = True
    cmd = "python3 {} --task_folder={} --utm_num={} --t_utm_world={} --data_type={} --sparse_data={}".format(
        os.path.join(project_root_folder, "preprocess/script/data_preprocess_qzc.py"),
        task_arg.task_folder,
        task_arg.utm_num,
        str(task_arg.t_utm_world).replace(" ", ""),
        task_arg.data_type,
        sparse_data)
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "data/data_preprocess_cmd.sh"), [cmd])
    if run_a_process(cmd, 3600*3) is not None:
        return ErrorCode.数据预处理_超时退出
    preprocess_result = json_file_to_dict(os.path.join(task_arg.task_folder, "data/preprocess_result.json"))
    if preprocess_result == {}:
        return ErrorCode.数据预处理_异常退出
    return_error_code = ErrorCode(preprocess_result["error_code"])
    task_arg.log["lidar_firmware"] = preprocess_result["lidar_firmware"]
    task_arg.log["bad_ins_rate"] = preprocess_result["bad_ins_rate"]
    return return_error_code


def run_pose_optim(task_arg: TaskArg) -> int:
    if task_arg.task_type == "zb_manual":
        util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["LiSam位姿优化"])
        cmd = "python3 {} --task_folder={}".format(
            os.path.join(project_root_folder, "lisam/scripts/liosam_process.py"),
            task_arg.task_folder)
        if run_a_process(cmd, 3600*3) is not None:
            return ErrorCode.lisam_超时退出  # 超时退出
        cmd = "python3 {} --task_folder={} --utm_num={} --t_utm_world={}".format(
            os.path.join(project_root_folder, "mapping/script/run_single_pose_optim.py"),
            task_arg.task_folder,
            task_arg.utm_num,
            str(task_arg.t_utm_world).replace(" ", ""))
    else:
        util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["多趟位姿优化"])
        cmd = "python3 {} --task_folder={} --utm_num={} --t_utm_world={}".format(
            os.path.join(project_root_folder, "mapping/script/run_pose_optim.py"),
            task_arg.task_folder,
            task_arg.utm_num,
            str(task_arg.t_utm_world).replace(" ", ""))

    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "mapping/pose_optim_cmd.sh"), [cmd])
    if run_a_process(cmd, 3600*5) is not None:
        return ErrorCode.激光建图_超时退出
    mapping_result = json_file_to_dict(os.path.join(task_arg.task_folder, "mapping_output/mapping_result.json"))
    if mapping_result == {}:
        return ErrorCode.激光建图_异常退出
    task_arg.log["use_ba"] = mapping_result["use_ba"] if "use_ba" in list(mapping_result.keys()) else None
    return_error_code = ErrorCode(mapping_result["error_code"] if "error_code" in list(mapping_result.keys()) else ErrorCode.激光建图_异常退出)
    return return_error_code

# 输入：mapping_output/veh_pose
def load_backload_pose(task_arg: TaskArg) -> int:
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/current_stage.txt"), ["载入backload优化位姿"])
    cmd = "python3 {} --task_folder={} --t_utm_world={} --utm_frame_out={}".format(
        os.path.join(project_root_folder, "mapping/script/opt_pose_mapping.py"),
        task_arg.task_folder,
        str(task_arg.t_utm_world).replace(" ", ""),
        # json.dumps(task_arg.info_json["t_utm_world"]).replace(' ', ''),
        task_arg.utm_frame_out)
    run_a_process(cmd, 3600*5)
    load_opt_folder = os.path.join(task_arg.task_folder, "backload_data/opt_veh_pose")
    if not os.path.exists(load_opt_folder):
        return ErrorCode.云端label_数据下载失败
    return ErrorCode.中业运行正常


def run_pointcloud_mapping(task_arg: TaskArg) -> int:
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["点云建图"])
    # cmd = "python3 {} --task_folder={} --utm_num={} --t_utm_world={} --flag=no_post_process".format(
    #     os.path.join(project_root_folder, "mapping/script/run_pointcloud_mapping.py"),
    #     task_arg.task_folder,
    #     task_arg.utm_num,
    #     str(task_arg.t_utm_world).replace(" ", ""))
    
    pos_txt_path = os.path.join(task_arg.task_folder, "../../source/lidar_mapping/output/rtk_origin.txt")
    with open(pos_txt_path, 'r') as file:
        line = file.readline().strip()
        while line[0] == '#':
            line = file.readline().strip()
        try:
            lla_str = line.split()
            lla = [float(num) for num in lla_str]
            origin_wgs_lon, origin_wgs_lat, origin_wgs_alt = lla[0], lla[1], 0
        except Exception as e:
            log.error("!!!! PointCloudConvert: WGS84_LLA NOT NORMAL: {} !!!!!!".format(line))
            print("WGS84_LLA not normal: ", line)
            return ErrorCode.激光建图_异常退出
        
    lla_str = '{}_{}_{}'.format(lla_str[0], lla_str[1], lla_str[2])
    utm_str = '{}_{}_{}'.format(task_arg.t_utm_world[0], task_arg.t_utm_world[1], 0)
    
    pc_wgs_path = os.path.join(task_arg.task_folder, '../../source/lidar_mapping/output/layer_pointcloud_semantic.pcd')
    pc_utm_path = os.path.join(task_arg.task_folder, 'model_pcd/global_cloud.pcd')
    # pc_utm_path = os.path.join(task_arg.task_folder, 'model_pcd/global_cloud_lidar.pcd')
    os.makedirs(os.path.join(task_arg.task_folder, 'model_pcd'), exist_ok=True)
    
    cmd_string = project_root_folder + "/mapping/pointcloudconvert/bin/pointcloudconvert"
    cmd_string += " --input=" + pc_wgs_path
    cmd_string += " --output=" + pc_utm_path
    cmd_string += " --wgs84_lla=" + lla_str
    cmd_string += " --utm_num=" + str(task_arg.utm_num)
    cmd_string += " --utm_lla=" + utm_str
    print(lla_str, task_arg.utm_num, utm_str)

    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "mapping_output/pointcloud_converting_cmd.sh"), [cmd_string])
    if run_a_process(cmd_string, 3600*5) is not None:
        return ErrorCode.激光建图_超时退出
    return ErrorCode.中业运行正常


def run_bev_mapping(task_arg: TaskArg) -> int:
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["BEV建图"])
    cmd = "python3 {} --task_folder={} --info_json_path={} --data_type={} --debug_lidar_bev_mapping={}".format(
        os.path.join(project_root_folder, "mapping/script/run_bev_mapping.py"),
        task_arg.task_folder,
        task_arg.info_json_path,
        task_arg.data_type,
        task_arg.debug_lidar_bev_mapping)
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "bev_mapping/bev_mapping_cmd.sh"), [cmd])
    if run_a_process(cmd, 3600*5) is not None:
        return ErrorCode.激光建图_超时退出
    return ErrorCode.中业运行正常


def run_mapbuild(task_arg: TaskArg, task_type) -> int:
    if task_arg.only_download == 1:
        return ErrorCode.中业运行正常
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["道路建模"])
    # cmd = "python3 {} --task_folder={} --utm_num={} --t_utm_world={}".format(
    #     os.path.join(project_root_folder, "modeling/script/run_modeling.py"),
    #     task_arg.task_folder,
    #     task_arg.utm_num,
    #     str(task_arg.t_utm_world).replace(" ", ""))
    process_mode = "2"
    # if task_arg.task_type == "zb_manual":
    #     process_mode = "1"
    global_pcd_path = os.path.join(task_arg.task_folder, "mapping_output/global_map_ground.pcd")
    entry = os.path.join(project_root_folder, "road_model/script/run.py")
    cmd = "python3 {} --workspace_folder={} --task_id={} --road_branch={} --info_json_path={} --utm_num={} --t_utm_world={} --process_mode={} --global_pcd_path={} --polygon_json_path={} --run_stage={} --default_topo_version={} --data_type={}".format(
            entry,
            task_arg.workspace_folder,
            task_arg.task_id,
            task_arg.road_branch,
            task_arg.info_json_path,
            task_arg.utm_num,
            json.dumps(task_arg.t_utm_world).replace(' ', ''),
            process_mode,
            global_pcd_path,
            task_arg.polygon_json_path,
            task_arg.run_stage,
            task_arg.default_topo_version,
            task_arg.data_type
            )
    if run_a_process(cmd, 3600*5) is not None:
        return ErrorCode.道路建模_超时退出  # 超时退出
    return ErrorCode.中业运行正常

def run_process_trafficlight(task_arg: TaskArg, task_type) -> int:
    if task_arg.only_download == 1:
        return ErrorCode.中业运行正常
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["交通灯建模"])
    # global_pcd_path = os.path.join(task_arg.task_folder, "mapping_output/global_map_ground.pcd")
    entry = os.path.join(project_root_folder, "road_model/process_air/process_trafficlight.py")
    cmd = "python3 {} --task_folder={}".format(
            entry,
            task_arg.task_folder)
    if run_a_process(cmd, 3600*5) is not None:
        return ErrorCode.交通灯建模_超时退出  # 超时退出
    return ErrorCode.中业运行正常

def cp_data_for_offline(task_arg: TaskArg) -> int:
    if task_arg.only_download == 1:
        return ErrorCode.中业运行正常
    
    # cp_src_path = os.path.join(task_arg.task_folder, "backload_data/opt_veh_pose/*")
    # cp_dst_path = os.path.join(task_arg.task_folder, "mapping_output/veh_pose")
    # add_folder_if_no_exist(cp_dst_path)
    # cmd_cp = "cp " + cp_src_path + " " + cp_dst_path
    # print(cmd_cp)
    # os.system(cmd_cp)

    if task_arg.bev_data_folder != "":
        print("copy tile_download")
        cp_dst_path = os.path.join(task_arg.task_folder, "tile_download")
        add_folder_if_no_exist(cp_dst_path)
        cp_src_folder = os.path.join(task_arg.bev_data_folder, task_arg.task_id , "tile_download")
        sub_folder = get_subfolders_path(cp_src_folder)
        p = Pool(20)
        for per_subfolder in sub_folder:
            cmd_cp = "cp -r -f " + per_subfolder + " " + cp_dst_path
            p.apply_async(run_cmd,args=(cmd_cp,) )
        p.close()
        p.join()
        

    ## 删除当前data目录
    print("开始删除当前data目录")
    data_path = os.path.join(task_arg.task_folder,"data")
    del_folder_if_exist(data_path)
    print("data目录删除完成")
    add_folder_if_no_exist(data_path)

    return ErrorCode.中业运行正常

def run_data_upload(task_arg: TaskArg) -> int:
    if task_arg.only_download == 1:
        return ErrorCode.中业运行正常
    upload_st = time.time()
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/sys_monitor_stage.txt"), ["数据上传"])
    cmd = "python3 {} --task_folder={} --utm_num={} --t_utm_world={} --tile_branch={} --tile_server={} --task_stamp_ms={}".format(
        os.path.join(project_root_folder, "data_io/script/data_upload.py"),
        task_arg.task_folder,
        task_arg.utm_num,
        str(task_arg.t_utm_world).replace(' ', ''),
        task_arg.tile_branch,
        task_arg.tile_server,
        task_arg.start_stamp_ms)
    if run_a_process(cmd, 10800) is not None:
        return ErrorCode.数据上传_超时退出  # 超时退出
    if not os.path.exists(os.path.join(task_arg.task_folder, "mapping_output/tile_id_list.txt")):
        return ErrorCode.数据上传_异常退出
    task_arg.tile_id_list = util.json_file_to_dict(os.path.join(task_arg.task_folder, "mapping_output/tile_id_list.txt"))
    task_arg.log["数据传输用时_m"] += round((time.time()-upload_st)/60, 2)
    return ErrorCode.中业运行正常

def run_cloud_label(task_arg: TaskArg) -> int:
    if task_arg.only_download == 1:
        return ErrorCode.中业运行正常
    util.write_lines_to_file_override(os.path.join(task_arg.task_folder, "debug/current_stage.txt"), ["优化位姿建图"])
    entry = os.path.join(project_root_folder, "mapping/script/opt_pose_mapping.py")
    true_utm_world = task_arg.info_json["t_utm_world"]
    cmd = "python3 {} --task_folder={} --t_utm_world={}".format(
        entry,
        task_arg.task_folder,
        json.dumps(true_utm_world).replace(' ', ''))
    run_a_process(cmd, 3600*5)
    
    cp_dst_pcd_path = os.path.join(task_arg.task_folder, "model_pcd/global_cloud.pcd")
    cp_global_pcd_path = os.path.join(task_arg.task_folder, "mapping_output/global_map_ground.pcd")
    cmd_cp_pcd = "cp " + cp_dst_pcd_path + " " + cp_global_pcd_path
    print(cmd_cp_pcd)
    os.system(cmd_cp_pcd)

    if not os.path.exists(os.path.join(task_arg.task_folder, "model_pcd/global_cloud.pcd")):
        return ErrorCode.云端label_异常退出
    return ErrorCode.中业运行正常

def generate_task_info(workspace_folder, use_opt_pose) -> int:
    entry = os.path.join(project_root_folder, "script/csv_to_info_json.py")
    data_file = os.path.join(workspace_folder, "_mla_egopose_gps_matched.csv")
    output_file = os.path.join(workspace_folder, "task_info_demo.json")

    cmd = "python3 {} --data_file={} --output_file={} --use_opt_pose={}".format(
        entry,
        data_file,
        output_file,
        int(use_opt_pose)
    )
    run_a_process(cmd, 3600*5)

    if not os.path.exists(output_file):
        return ""


    return output_file

def generate_task_info3(workspace_folder, use_opt_pose, data_type, debug_model, run_secific_trajs) -> int:
    entry = os.path.join(project_root_folder, "script/csv_to_info_json.py")
    # data_file = os.path.join(workspace_folder, "_mla_egopose_gps_matched.csv")
    output_file = os.path.join(workspace_folder, "task_info_demo.json")

    str_run_secific_trajs = ','.join(run_secific_trajs) 

    cmd = "python3 {} --data_folder={} --output_file={} --use_opt_pose={} --data_type={} --debug_model={} --run_secific_trajs={}".format(
        entry,
        workspace_folder,
        output_file,
        int(use_opt_pose),
        data_type,
        debug_model,
        str_run_secific_trajs
    )
    run_a_process(cmd, 3600*5)

    if not os.path.exists(output_file):
        return ErrorCode.任务配置文件生成失败, ""

    return ErrorCode.中业运行正常, output_file


def run(info_json_path, workspace_folder, tile_branch, road_branch, tile_server, road_server, task_type ,
        dir_flag, label,only_download , x_min, x_max, y_min, y_max, z_max,bev_data_folder, utm_frame_out,
        offline_flag, polygon_json_path, run_stage, use_opt_pose, data_type, debug_model, run_secific_trajs,
        debug_run_evaluation, default_topo_version, debug_lidar_bev_mapping):

    error_code, new_info_json_path = generate_task_info3(workspace_folder, use_opt_pose, data_type, debug_model, run_secific_trajs)
    if new_info_json_path != "":
        info_json_path = new_info_json_path

    print("============================================")
    print("                 任务初始化                   ")
    print("============================================")
    if error_code == ErrorCode.中业运行正常:
        error_code, task_arg = init_task(info_json_path, workspace_folder, tile_branch, road_branch, tile_server, road_server, only_download, x_min, x_max, y_min, y_max, z_max, bev_data_folder, utm_frame_out, offline_flag, polygon_json_path)
        task_arg.run_stage = run_stage
        task_arg.use_opt_pose = use_opt_pose
        task_arg.data_type = data_type
        task_arg.debug_model = debug_model
        task_arg.debug_run_evaluation = debug_run_evaluation
        task_arg.default_topo_version = default_topo_version
        task_arg.debug_lidar_bev_mapping = debug_lidar_bev_mapping

    if run_stage == 0 or run_stage == 1:
        print("============================================")
        print("                  数据下载                   ")
        print("============================================")
        if error_code == ErrorCode.中业运行正常:
            error_code = run_data_download(task_arg)

        
        print("============================================")
        print("                 数据同步                    ")
        print("============================================")
        if error_code == ErrorCode.中业运行正常:
            error_code = cp_data_for_offline(task_arg)

        print("============================================")
        print("                 数据预处理                   ")
        print("============================================")
        if error_code == ErrorCode.中业运行正常:
            error_code = run_data_preprocess(task_arg)
        print(error_code)
        
        if data_type == DataType.BYD_LIDAR_BEV_B.name:
            print("============================================")
            print("                 BEV&Lidar融合建图                   ")
            print("============================================")
            if error_code == ErrorCode.中业运行正常:
                error_code= run_bev_mapping(task_arg)
        elif data_type == DataType.BYD_LIDAR_B.name:
            # print("============================================")
            # print("                 BEV建图                   ")
            # print("============================================")
            # if error_code == ErrorCode.中业运行正常:
            #     error_code= run_bev_mapping(task_arg)

            print("============================================")
            print("                 点云建图                   ")
            print("============================================")
            if error_code == ErrorCode.中业运行正常:
                error_code = run_pointcloud_mapping(task_arg)
                
        else:
            print("============================================")
            print("                 BEV建图                   ")
            print("============================================")
            if error_code == ErrorCode.中业运行正常:
                error_code= run_bev_mapping(task_arg)

    print("============================================")
    print("                 道路建模                    ")
    print("============================================")
    if error_code == ErrorCode.中业运行正常:
        error_code = run_mapbuild(task_arg, task_type)
    print(error_code)

    # ------------------------------------------------------------------
    print("============================================")
    print("                 交通灯建模                 ")
    print("============================================")
    # if run_stage == 0 or run_stage == 3 or run_stage == 4:
    if run_stage == 0 or run_stage == 4:
        if error_code == ErrorCode.中业运行正常:
            if data_type in [DataType.BYD_BEV.name, DataType.MMT_RC.name]:
                error_code = run_process_trafficlight(task_arg, task_type)
            #### TODO 补充lidar的处理
        print(error_code)
    # ------------------------------------------------------------------

    # print("============================================")
    # print("                  资料上传                   ")
    # print("============================================")
    # if error_code == ErrorCode.中业运行正常:
    #     error_code = run_data_upload(task_arg)

    print("============================================")
    print("              结果生成 & 发送通知             ")
    print("============================================")
    if task_arg.only_download == 0:
        make_result_json(task_arg, error_code)
        save_result(task_arg, error_code)
        # feishu_publish(task_arg.task_folder)

def get_data_type(data_folder, force_task_type):
    tasks_json_path = os.path.join(data_folder, "input/tasks.json")
    if not os.path.exists(tasks_json_path):
        log.error("!!!! No auto_label/input/tasks.json file !!!!!!")
        return ""
    
    with open(tasks_json_path,'r',encoding='utf8')as fp:
        task_json_data = json.load(fp)
    fp.close()

    if 'bev_bag_list' not in task_json_data and 'lidar_bag_list' not in task_json_data:
        log.error("!!!! No attr 'bev_bag_list' or 'lidar_bag_list' in tasks.json file !!!!!!")
        return ""
    
    if "task_type" not in task_json_data:
        log.error("!!!! No attr 'task_type' in tasks.json file !!!!!!")
        return ""
    task_type = task_json_data["task_type"]
    log.info("task_type: ", task_type)

    if force_task_type > 0:
        task_type = force_task_type
        log.warning("force_task_type: ", task_type)

    if task_type not in [1, 2, 3]:
        log.error("!!!! tasks.json: 'task_type' NOT NORMAL !!!!!!")
        return ""
    if task_type == 1:
        bag_list=task_json_data["bev_bag_list"]
    elif task_type == 2 or task_type == 3:
        bag_list=task_json_data["lidar_bag_list"]
    
    # 默认为0
    data_type_counter_dict = defaultdict(int)
    for bag_info in bag_list:
        if 'type' not in bag_info:
            continue
        bag_type = bag_info['type']
        if task_type == 1:
            if bag_type in ['mmt', 'mmt_rc']: # 目前 mmt研采和量产数据，都使用同一个pose type == "mmt_rc"
                bag_type = DataType.MMT_RC.name
            elif bag_type in ['byd', 'byd_gf', 'byd_b', 'byd_bd']:
                bag_type = DataType.BYD_BEV.name
            else:
                log.error("!!!! task_type:{} not match datatype:{} !!!!!!".format(task_type, bag_type))
                continue
        elif task_type == 2:
            if bag_type in ['mmt', 'mmt_rc', 'byd_b']:
                bag_type = DataType.BYD_LIDAR_B.name
            else:
                log.error("!!!! task_type:{} not match datatype:{} !!!!!!".format(task_type, bag_type))
                continue
        elif task_type == 3:
            if bag_type in ['byd_b']:
                bag_type = DataType.BYD_LIDAR_BEV_B.name
            else:
                log.error("!!!! task_type:{} not match datatype:{} !!!!!!".format(task_type, bag_type))
                continue
        data_type_counter_dict[bag_type] += 1

    items_list = list(data_type_counter_dict.items())
    data_type_size = len(items_list)
    if data_type_size == 0:
        log.error("!!!! is empty !!!!!!")
        return ""
    elif data_type_size == 1:
        # 目前只支持同时跑一种数据源---> mmt: momenta生产数据, mmt_rc: momenta研采, byd: 比亚迪生产, byd_rc: 比亚迪研采
        if task_type == 1 and items_list[0][0] in [DataType.BYD_BEV.name, DataType.MMT_RC.name]:
            return items_list[0][0]
        elif task_type == 2 and items_list[0][0] == DataType.BYD_LIDAR_B.name:
            return items_list[0][0]
        elif task_type == 3 and items_list[0][0] == DataType.BYD_LIDAR_BEV_B.name:
            return items_list[0][0]
        else:
            log.error("!!!! inconsistent type : {} : {} !!!!!!".format(task_type, items_list[0][0]))
            return ""
    elif data_type_size > 1:
        log.error("!!!! has many data_type : {} : {} !!!!!!".format(items_list[0], items_list[1]))
        return ""


#####################################################################################################
#####################################################################################################
#####################################################################################################

default_branch = time.strftime('test_%d%H%M%S', time.localtime())
default_tile_server = "172.21.26.28:8082"
default_road_server = "172.21.133.164:8081"
default_only_download = 0

# 下面目前非必须
default_task_type = "zb_auto"
# default_info_json_path = project_root_folder+"/data/task_info_demo_1.json"
default_info_json_path = project_root_folder+"/data/task_info_demo.json"
default_polygon_json_path = project_root_folder+"/data/polygon.json"
# default_workspace_folder = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/test2"

# 下面的是必须的
# default_workspace_folder =  "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross/11002493878588"
default_workspace_folder =  "/mnt/c/02data/01zb_mapping/test/mnt"
default_run_stage=0 # 0:所有，1：拼图，2：矢量化，3：拓扑，4: 交通灯
default_topo_version = 1  # 0:旧版本 1：新版本

default_force_task_type = 0 # 调试使用,和另一个参数task_type意义不一样： 0：使用tasks.json中的类型，1: bev, 2: lidar, 3: bev & lidar 融合
default_debug_ldiar_bev_mapping = 0 # 0：不生成， 1: 在task type = 3 时，且存在bev资料，则用激光pose+BEV资料生成bev地图, 2: 融合前生成纯激光的地图， 3：分别生成bev 和 激光地图
default_debug_run_evaluation=1 # 0: 不评测， 1： 仅评测， 2：嵌入到主流程评测
default_use_opt_pose=1 # 是否使用 bev 优化的pose
default_debug_model=0 # 联调为0， 本地用为1
# 如果为空，那么 default_workspace_folder 路径下 PL 开头的 clips 都会运行； 否则，只跑该路口下面列表中部分clip数据
default_run_secific_trajs = [
]
# default_data_type = "byd" # mmt: momenta生产数据, mmt_rc: momenta研采, byd: 比亚迪生产, byd_rc: 比亚迪研采

parser = argparse.ArgumentParser()
parser.add_argument('--info_json_path', type=str, default=default_info_json_path)
parser.add_argument('--polygon_json_path', type=str, default=default_polygon_json_path)
# parser.add_argument('--workspace_folder', type=str, default=os.environ['HOME'] + "/disk/lidar_mapping_ws")
parser.add_argument('--workspace_folder', type=str, default=default_workspace_folder)
parser.add_argument('--tile_branch', type=str, default=default_branch)  # branch名称使用顺序：命令行传入 > 默认随机值
parser.add_argument('--road_branch', type=str, default=default_branch)
parser.add_argument('--tile_server', type=str, default=default_tile_server)  # 服务器地址使用顺序：命令行传入 > 任务json传入 > 默认值
parser.add_argument('--road_server', type=str, default=default_road_server)
parser.add_argument('--task_type', type=str, default=default_task_type)
parser.add_argument('--force_task_type', type=int, default=default_force_task_type)
parser.add_argument('--bev_data_folder', type=str, default="") # 没用
parser.add_argument('--dir_flag', type=int, default=0) # 没用
parser.add_argument('--label_flag', type=str, default="label") # 没用
parser.add_argument('--only_download', type=int, default=default_only_download)
parser.add_argument('--x_min', type=float, default=0) 
parser.add_argument('--y_min', type=float, default=-50) 
parser.add_argument('--x_max', type=float, default=50) 
parser.add_argument('--y_max', type=float, default=50) 
parser.add_argument('--z_max', type=float, default=0.2) 
parser.add_argument('--utm_frame_out', type=int, default=1, help="单帧utm点云输出")  # 是否输出utm下的点云
parser.add_argument('--offline_flag', type=int, default=1, help="离线flag") 
parser.add_argument('--run_stage', type=int, default=default_run_stage, help="拼图、矢量化、拓扑分段运行") 
parser.add_argument('--use_opt_pose', type=int, default=default_use_opt_pose, help="是否使用 上游优化的pose") 
# parser.add_argument('--data_type', type=str,
#                     default=default_data_type, help="数据源=> mmt: momenta生产数据, mmt_rc: momenta研采, byd: 比亚迪生产, byd_rc: 比亚迪研采")
parser.add_argument('--run_secific_trajs', type=list, default=default_run_secific_trajs, help="运行特定的数据") 
parser.add_argument('--debug_model', type=int, default=default_debug_model, help="是否使用debug测试") 
parser.add_argument('--debug_run_evaluation', type=int, default=default_debug_run_evaluation, help="是否使用评估")
parser.add_argument('--default_topo_version', type=int, default=default_topo_version, help="topo 版本，临时")
parser.add_argument('--debug_lidar_bev_mapping', type=int, default=default_debug_ldiar_bev_mapping, help="task_type=3时,用激光pose+BEV建图")

if __name__ == "__main__":
    args_ = parser.parse_args()

    data_type = get_data_type(os.path.join(args_.workspace_folder,"auto_label"), args_.force_task_type)
    if data_type == "":
        log.error("data_type is unkown: ", data_type)
        pass
    log.info("data_type: ", data_type)

    run(args_.info_json_path,
        # args_.workspace_folder,
        os.path.join(args_.workspace_folder,"auto_label"),
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
        args_.debug_model,
        args_.run_secific_trajs,
        args_.debug_run_evaluation,
        args_.default_topo_version,
        args_.debug_lidar_bev_mapping,
        )
