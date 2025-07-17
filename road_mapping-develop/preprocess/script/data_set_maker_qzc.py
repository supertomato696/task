import argparse
from trail_data import TrailData, FrameData
import pandas as pd
import numpy as np
import util
from util import *
from util import log
import os
import bisect
import shapely.wkt
import shapely.ops
from shapely.geometry import Point
import sys

lidar_mapping_root_folder = os.path.abspath(__file__+"/../../../")
project_root_folder = os.path.abspath(__file__+"/../../../")
# task_type_name_dict = {"zl": "连续直路", "lk2": "离散路口", "zb_manual": "众包人工", "zb_auto": "众包自动", "zb_auto_zc": "自采自动"}
sys.path.append(project_root_folder)
from data_io.script.bev_label_map import DataType

# import preprocess.script.util as util

class LinkInfo():
    def __init__(self) -> None:
        self.link_id = ""
        self.link_nodes = []
        self.link_nodes_heading = []
        self.link_direction = 1
        self.trail_ids = []  # link 关联的 trail


class DataSetMaker:
    def __init__(self) -> None:
        self.data_folder: str
        self.utm_num: int
        self.t_utm_world: list = []
        self.trail_data: TrailData = TrailData()
        self.data_scope: str = ""
        self.odom_frame_id_to_stamp_ms: Dict[int, int] = {}
        self.odom_frame_id_to_stamp_local: Dict[int, int] = {}
        self.binged_link_dict: Dict[int, LinkInfo] = {}
        self.pp_laneline_dict: Dict[int, Dict] = {}

    def read_calib_to_trail_data(self):
        lidar_calib_path = os.path.join(self.data_folder, "calib_lidar_params.json")
        lidar_calib_info = util.json_file_to_dict(lidar_calib_path)
        log.info(lidar_calib_info)
        if "lidar0" not in lidar_calib_info.keys() and "calib_lidar_params" in lidar_calib_info.keys():
            lidar_calib_info = lidar_calib_info["calib_lidar_params"]
        self.trail_data.q_veh_lidar = [lidar_calib_info["lidar0"]["q_vcs_lidar"]["qx"],
                                       lidar_calib_info["lidar0"]["q_vcs_lidar"]["qy"],
                                       lidar_calib_info["lidar0"]["q_vcs_lidar"]["qz"],
                                       lidar_calib_info["lidar0"]["q_vcs_lidar"]["qw"]]
        self.trail_data.t_veh_lidar = [lidar_calib_info["lidar0"]["t_vcs_lidar"]["x"],
                                       lidar_calib_info["lidar0"]["t_vcs_lidar"]["y"],
                                       lidar_calib_info["lidar0"]["t_vcs_lidar"]["z"]]
        self.trail_data.T_veh_lidar = util.qab_tab_to_Tab(self.trail_data.q_veh_lidar, self.trail_data.t_veh_lidar)
        log.info("[DataSerMaker] 完成lidar标定参数读取")

        cam_calib_path = os.path.join(self.data_folder, "calib_cam_params.json")
        cam_calib_info = util.json_file_to_dict(cam_calib_path)
        self.trail_data.q_veh_cam = [cam_calib_info["cam0"]["q_vcs_cam"]["x"],
                                     cam_calib_info["cam0"]["q_vcs_cam"]["y"],
                                     cam_calib_info["cam0"]["q_vcs_cam"]["z"],
                                     cam_calib_info["cam0"]["q_vcs_cam"]["w"]]
        self.trail_data.t_veh_cam = [cam_calib_info["cam0"]["t_vcs_cam"]["x"],
                                     cam_calib_info["cam0"]["t_vcs_cam"]["y"],
                                     cam_calib_info["cam0"]["t_vcs_cam"]["z"]]
        self.trail_data.T_veh_cam = util.qab_tab_to_Tab(self.trail_data.q_veh_cam, self.trail_data.t_veh_cam)
        self.trail_data.cam_width = cam_calib_info["cam0"]["resolution"]["ru"]
        self.trail_data.cam_height = cam_calib_info["cam0"]["resolution"]["rv"]
        self.trail_data.cam_fxycxy = [cam_calib_info["cam0"]["proj_param"]["fu"],
                                      cam_calib_info["cam0"]["proj_param"]["fv"],
                                      cam_calib_info["cam0"]["proj_param"]["cu"],
                                      cam_calib_info["cam0"]["proj_param"]["cv"]]
        if(cam_calib_info["cam0"]["dist_model"] == "radtan"):
            self.trail_data.cam_dist_type = "radtan"
            self.trail_data.cam_k12p12k3456 = [cam_calib_info["cam0"]["dist_param"]["k1"],
                                               cam_calib_info["cam0"]["dist_param"]["k2"],
                                               cam_calib_info["cam0"]["dist_param"]["p1"],
                                               cam_calib_info["cam0"]["dist_param"]["p2"],
                                               cam_calib_info["cam0"]["dist_param"]["k3"],
                                               cam_calib_info["cam0"]["dist_param"]["k4"],
                                               cam_calib_info["cam0"]["dist_param"]["k5"],
                                               cam_calib_info["cam0"]["dist_param"]["k6"]]
        if(cam_calib_info["cam0"]["dist_model"] == "equidistant"):
            self.trail_data.cam_dist_type = "equidistant"
            self.trail_data.cam_k1234 = [cam_calib_info["cam0"]["dist_param"]["k1"],
                                         cam_calib_info["cam0"]["dist_param"]["k2"],
                                         cam_calib_info["cam0"]["dist_param"]["k3"],
                                         cam_calib_info["cam0"]["dist_param"]["k4"]]
        self.trail_data.cam_K = util.fxycxy_to_K(self.trail_data.cam_fxycxy)
        log.info("[DataSerMaker] 完成cam标定参数读取")

    def read_imu_to_csv(self):
        data = []
        imu_file_path = os.path.join(self.data_folder, "nav_imu.txt")
        lines = util.read_lines_from_file(imu_file_path)
        for one_line in lines:
            j = json.loads(one_line)
            data.append([j["header"]["frame_id"],  # FRD -> FLU
                         j["header"]["stamp"],
                         j["imu_time"],
                         j["linear_acceleration"]["x"],
                         -j["linear_acceleration"]["y"],
                         -j["linear_acceleration"]["z"],
                         np.deg2rad(j["angular_velocity"]["x"]),
                         -np.deg2rad(j["angular_velocity"]["y"]),
                         -np.deg2rad(j["angular_velocity"]["z"])])
        df = pd.DataFrame(data, columns=["id", "ts", "local_time", "ax", "ay", "az", "wx", "wy", "wz"])
        df.sort_values(by=['id'], inplace=True)
        df.to_csv(os.path.join(self.data_folder, "imu.txt"), sep=" ", index=False)

    def ins_file_to_csv(self, ins_file_lines: list, csv_path: str):
        R_ENU_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_FRD_FLU = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        data_frame = []
        for one_line in ins_file_lines:
            j = json.loads(one_line)
            ins_lonlatalt = [j["position"]["longitude"], j["position"]["latitude"], j["position"]["height"]]
            ins_euler = [j["euler_angles"]["x"], j["euler_angles"]["y"], j["euler_angles"]["z"]]
            ins_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*ins_lonlatalt, self.utm_num)
            ins_lonlatalt[0], ins_lonlatalt[1] = util.wgs2gcj(ins_lonlatalt[0], ins_lonlatalt[1])
            gcj_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*ins_lonlatalt, self.utm_num)
            r = scipy.spatial.transform.Rotation.from_euler('xyz', [ins_euler[0], ins_euler[1], ins_euler[2]], degrees=True)
            ins_R_NED_FRD = r.as_matrix()
            ins_R_ENU_FRD = R_ENU_NED.dot(ins_R_NED_FRD)
            ins_R_ENU_FLU = ins_R_ENU_FRD.dot(R_FRD_FLU)
            ins_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, ins_lonlatalt[0], ins_lonlatalt[1], ins_R_ENU_FLU)
            ins_q_ENU_FLU = util.Rab_to_qab(ins_R_ENU_FLU)
            data_frame.append([j["header"]["frame_id"],
                               j["header"]["stamp"],
                               j["ins_time"],
                               gcj_utm_xyz[0]-self.t_utm_world[0],
                               gcj_utm_xyz[1]-self.t_utm_world[1],
                               gcj_utm_xyz[2]-self.t_utm_world[2],
                               ins_utm_xyz[0]-self.t_utm_world[0],
                               ins_utm_xyz[1]-self.t_utm_world[1],
                               ins_utm_xyz[2]-self.t_utm_world[2],
                               j["linear_velocity"]["y"],    # E -> N
                               j["linear_velocity"]["x"],    # N -> E
                               j["linear_velocity"]["z"]*-1,  # D -> U
                               ins_q_ENU_FLU[3],
                               ins_q_ENU_FLU[0],
                               ins_q_ENU_FLU[1],
                               ins_q_ENU_FLU[2]])

        df = pd.DataFrame(data_frame, columns=["id", "ts", "local_time", "x", "y", "z", "x2", "y2", "z2", "vx", "vy", "vz", "qw", "qx", "qy", "qz"])
        df.sort_values(by=['id'], inplace=True)
        df.to_csv(csv_path, sep=" ", index=False)

    def read_nav_ins_to_trail_data(self):
        R_ENU_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_FRD_FLU = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        nav_ins_dict: Dict[int, Dict] = {}
        ins_file_lines = util.read_lines_from_file(os.path.join(self.data_folder, "nav_ins.txt"))
        for one_line in ins_file_lines:
            j = json.loads(one_line)
            stamp_ms = j["header"]["stamp"]
            nav_ins_dict[stamp_ms] = j
        nav_ins_dict = {key: nav_ins_dict[key] for key in sorted(nav_ins_dict.keys())}
        ins_stamp_ms_list: List = list(nav_ins_dict.keys())
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(ins_stamp_ms_list, frame_stamp_ms)
            if (loc == 0) or (loc == len(ins_stamp_ms_list)):
                frame.flag.append("no_ins")
                log.warning("frame 无对应 ins", frame_stamp_ms)
                continue
            older_stamp = ins_stamp_ms_list[loc-1]
            older_lla = [nav_ins_dict[older_stamp]["position"]["longitude"],
                         nav_ins_dict[older_stamp]["position"]["latitude"],
                         nav_ins_dict[older_stamp]["position"]["height"]]
            older_lla[0], older_lla[1] = util.wgs2gcj(older_lla[0], older_lla[1])
            older_euler = [nav_ins_dict[older_stamp]["euler_angles"]["x"],
                           nav_ins_dict[older_stamp]["euler_angles"]["y"],
                           nav_ins_dict[older_stamp]["euler_angles"]["z"]]
            ins_position_type = nav_ins_dict[older_stamp]["position_type"]
            newer_stamp = ins_stamp_ms_list[loc]
            newer_lla = [nav_ins_dict[newer_stamp]["position"]["longitude"],
                         nav_ins_dict[newer_stamp]["position"]["latitude"],
                         nav_ins_dict[newer_stamp]["position"]["height"]]
            newer_lla[0], newer_lla[1] = util.wgs2gcj(newer_lla[0], newer_lla[1])
            newer_euler = [nav_ins_dict[newer_stamp]["euler_angles"]["x"],
                           nav_ins_dict[newer_stamp]["euler_angles"]["y"],
                           nav_ins_dict[newer_stamp]["euler_angles"]["z"]]
            older_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*older_lla, self.utm_num)
            r = scipy.spatial.transform.Rotation.from_euler('xyz', [older_euler[0], older_euler[1], older_euler[2]], degrees=True)
            older_R_NED_FRD = r.as_matrix()
            older_R_ENU_FRD = R_ENU_NED.dot(older_R_NED_FRD)
            older_R_ENU_FLU = older_R_ENU_FRD.dot(R_FRD_FLU)
            older_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, older_lla[0], older_lla[1], older_R_ENU_FLU)
            older_q_ENU_FLU = util.Rab_to_qab(older_R_ENU_FLU)
            newer_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*newer_lla, self.utm_num)
            r = scipy.spatial.transform.Rotation.from_euler('xyz', [newer_euler[0], newer_euler[1], newer_euler[2]], degrees=True)
            newer_R_NED_FRD = r.as_matrix()
            newer_R_ENU_FRD = R_ENU_NED.dot(newer_R_NED_FRD)
            newer_R_ENU_FLU = newer_R_ENU_FRD.dot(R_FRD_FLU)
            newer_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, newer_lla[0], newer_lla[1], newer_R_ENU_FLU)
            newer_q_ENU_FLU = util.Rab_to_qab(newer_R_ENU_FLU)
            current_q, current_t = util.qt_chazhi(older_stamp, older_q_ENU_FLU, older_utm_xyz,
                                                  newer_stamp, newer_q_ENU_FLU, newer_utm_xyz, frame_stamp_ms)
            frame.ins_q_world_veh = current_q
            frame.ins_t_world_veh = current_t-self.t_utm_world
            frame.ins_position_type = ins_position_type

        # 判断黑天白天
        for stamp_ms, v in nav_ins_dict.items():
            if util.is_day_time(v["position"]["longitude"], v["position"]["latitude"], float(stamp_ms/1000)) == True:
                self.trail_data.tag.append("day")
            else:
                self.trail_data.tag.append("night")
            break
        self.ins_file_to_csv(ins_file_lines, os.path.join(self.data_folder, "ins.txt"))


    def ins_file_to_csv_qzc(self, ins_file_lines: list, csv_path: str):
        R_ENU_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_FRD_FLU = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        data_frame = []
        for one_line in ins_file_lines:
            j = json.loads(one_line)
            ins_lonlatalt = [j["position"]["longitude"], j["position"]["latitude"], j["position"]["height"]]
            ins_euler = [j["euler_angles"]["x"], j["euler_angles"]["y"], j["euler_angles"]["z"]]
            ins_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*ins_lonlatalt, self.utm_num)
            ins_lonlatalt[0], ins_lonlatalt[1] = util.wgs2gcj(ins_lonlatalt[0], ins_lonlatalt[1])
            gcj_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*ins_lonlatalt, self.utm_num)
            r = scipy.spatial.transform.Rotation.from_euler('xyz', [ins_euler[0], ins_euler[1], ins_euler[2]], degrees=True)
            ins_R_NED_FRD = r.as_matrix()
            ins_R_ENU_FRD = R_ENU_NED.dot(ins_R_NED_FRD)
            ins_R_ENU_FLU = ins_R_ENU_FRD.dot(R_FRD_FLU)
            ins_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, ins_lonlatalt[0], ins_lonlatalt[1], ins_R_ENU_FLU)
            ins_q_ENU_FLU = util.Rab_to_qab(ins_R_ENU_FLU)
            data_frame.append([j["header"]["frame_id"],
                               j["header"]["stamp"],
                               j["ins_time"],
                               gcj_utm_xyz[0]-self.t_utm_world[0],
                               gcj_utm_xyz[1]-self.t_utm_world[1],
                               gcj_utm_xyz[2]-self.t_utm_world[2],
                               ins_utm_xyz[0]-self.t_utm_world[0],
                               ins_utm_xyz[1]-self.t_utm_world[1],
                               ins_utm_xyz[2]-self.t_utm_world[2],
                               j["linear_velocity"]["y"],    # E -> N
                               j["linear_velocity"]["x"],    # N -> E
                               j["linear_velocity"]["z"]*-1,  # D -> U
                               ins_q_ENU_FLU[3],
                               ins_q_ENU_FLU[0],
                               ins_q_ENU_FLU[1],
                               ins_q_ENU_FLU[2]])

        df = pd.DataFrame(data_frame, columns=["id", "ts", "local_time", "x", "y", "z", "x2", "y2", "z2", "vx", "vy", "vz", "qw", "qx", "qy", "qz"])
        df.sort_values(by=['id'], inplace=True)
        df.to_csv(csv_path, sep=" ", index=False)

    def read_nav_ins_to_trail_data_qzc(self):
        R_ENU_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_FRD_FLU = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        nav_ins_dict: Dict[int, Dict] = {}
        ins_file_lines = util.read_lines_from_file(os.path.join(self.data_folder, "nav_ins.txt"))
        for one_line in ins_file_lines:
            j = json.loads(one_line)
            # stamp_ms = j["header"]["stamp"]
            stamp_us = int(j["header"]["stamp"])
            stamp_ms = int(round(stamp_us/1000))

            nav_ins_dict[stamp_ms] = j
        nav_ins_dict = {key: nav_ins_dict[key] for key in sorted(nav_ins_dict.keys())}
        ins_stamp_ms_list: List = list(nav_ins_dict.keys())
        frame: FrameData
        for frame in self.trail_data.lidar_frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(ins_stamp_ms_list, frame_stamp_ms)
            if (loc == 0) or (loc == len(ins_stamp_ms_list)):
                frame.flag.append("no_ins")
                log.warning("lidar frame 无对应 ins: ", frame_stamp_ms)
                continue
            older_stamp = ins_stamp_ms_list[loc-1]
            older_lla = [nav_ins_dict[older_stamp]["position"]["longitude"], 
                         nav_ins_dict[older_stamp]["position"]["latitude"],
                         nav_ins_dict[older_stamp]["position"]["height"]]
            # TODO:qzc 
            # older_lla[0], older_lla[1] = util.gcj2wgs(older_lla[0], older_lla[1]) # gcj02->wgs84
            
            # older_lla[0], older_lla[1] = util.wgs2gcj(older_lla[0], older_lla[1])
            # older_euler = [nav_ins_dict[older_stamp]["euler_angles"]["x"],
            #                nav_ins_dict[older_stamp]["euler_angles"]["y"],
            #                nav_ins_dict[older_stamp]["euler_angles"]["z"]]
            ins_position_type = nav_ins_dict[older_stamp]["position_type"]

            newer_stamp = ins_stamp_ms_list[loc]
            newer_lla = [nav_ins_dict[newer_stamp]["position"]["longitude"],
                         nav_ins_dict[newer_stamp]["position"]["latitude"],
                         nav_ins_dict[newer_stamp]["position"]["height"]]
            # TODO:qzc 
            # newer_lla[0], newer_lla[1] = util.gcj2wgs(newer_lla[0], newer_lla[1]) # gcj02->wgs84
            
            # newer_lla[0], newer_lla[1] = util.wgs2gcj(newer_lla[0], newer_lla[1])
            # newer_euler = [nav_ins_dict[newer_stamp]["euler_angles"]["x"],
            #                nav_ins_dict[newer_stamp]["euler_angles"]["y"],
            #                nav_ins_dict[newer_stamp]["euler_angles"]["z"]]
            older_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*older_lla, self.utm_num)
            # r = scipy.spatial.transform.Rotation.from_euler('xyz', [older_euler[0], older_euler[1], older_euler[2]], degrees=True)
            # older_R_NED_FRD = r.as_matrix()
            # older_R_ENU_FRD = R_ENU_NED.dot(older_R_NED_FRD)
            # older_R_ENU_FLU = older_R_ENU_FRD.dot(R_FRD_FLU)
            # older_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, older_lla[0], older_lla[1], older_R_ENU_FLU)
            # older_q_ENU_FLU = util.Rab_to_qab(older_R_ENU_FLU)
            older_q_ENU_FLU =  [
                                nav_ins_dict[older_stamp]["quaternion"]['x'],
                                nav_ins_dict[older_stamp]["quaternion"]['y'],
                                nav_ins_dict[older_stamp]["quaternion"]['z'],
                                nav_ins_dict[older_stamp]["quaternion"]['w']
                            ]

            newer_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*newer_lla, self.utm_num)
            # r = scipy.spatial.transform.Rotation.from_euler('xyz', [newer_euler[0], newer_euler[1], newer_euler[2]], degrees=True)
            # newer_R_NED_FRD = r.as_matrix()
            # newer_R_ENU_FRD = R_ENU_NED.dot(newer_R_NED_FRD)
            # newer_R_ENU_FLU = newer_R_ENU_FRD.dot(R_FRD_FLU)
            # newer_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, newer_lla[0], newer_lla[1], newer_R_ENU_FLU)
            # newer_q_ENU_FLU = util.Rab_to_qab(newer_R_ENU_FLU)
            newer_q_ENU_FLU =  [
                                nav_ins_dict[newer_stamp]["quaternion"]['x'],
                                nav_ins_dict[newer_stamp]["quaternion"]['y'],
                                nav_ins_dict[newer_stamp]["quaternion"]['z'],
                                nav_ins_dict[newer_stamp]["quaternion"]['w']
                            ]

            current_q, current_t = util.qt_chazhi(older_stamp, older_q_ENU_FLU, older_utm_xyz,
                                                  newer_stamp, newer_q_ENU_FLU, newer_utm_xyz, frame_stamp_ms)
            frame.ins_q_world_veh = current_q
            frame.ins_t_world_veh = current_t-self.t_utm_world
            frame.ins_position_type = ins_position_type

            # TODO:qzc use utm instead of opt_q_world_veh 
            frame.opt_q_world_veh = current_q
            frame.opt_t_world_veh = current_t-self.t_utm_world


        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(ins_stamp_ms_list, frame_stamp_ms)
            if (loc == 0) or (loc == len(ins_stamp_ms_list)):
                frame.flag.append("no_ins")
                log.warning("bev frame 无对应 ins: ", frame_stamp_ms)
                continue
            older_stamp = ins_stamp_ms_list[loc-1]
            older_lla = [nav_ins_dict[older_stamp]["position"]["longitude"], 
                         nav_ins_dict[older_stamp]["position"]["latitude"],
                         nav_ins_dict[older_stamp]["position"]["height"]]
            # TODO:qzc 
            # older_lla[0], older_lla[1] = util.gcj2wgs(older_lla[0], older_lla[1]) # gcj02->wgs84
            
            # older_lla[0], older_lla[1] = util.wgs2gcj(older_lla[0], older_lla[1])
            # older_euler = [nav_ins_dict[older_stamp]["euler_angles"]["x"],
            #                nav_ins_dict[older_stamp]["euler_angles"]["y"],
            #                nav_ins_dict[older_stamp]["euler_angles"]["z"]]
            ins_position_type = nav_ins_dict[older_stamp]["position_type"]

            newer_stamp = ins_stamp_ms_list[loc]
            newer_lla = [nav_ins_dict[newer_stamp]["position"]["longitude"],
                         nav_ins_dict[newer_stamp]["position"]["latitude"],
                         nav_ins_dict[newer_stamp]["position"]["height"]]
            # TODO:qzc 
            # newer_lla[0], newer_lla[1] = util.gcj2wgs(newer_lla[0], newer_lla[1]) # gcj02->wgs84
            
            # newer_lla[0], newer_lla[1] = util.wgs2gcj(newer_lla[0], newer_lla[1])
            # newer_euler = [nav_ins_dict[newer_stamp]["euler_angles"]["x"],
            #                nav_ins_dict[newer_stamp]["euler_angles"]["y"],
            #                nav_ins_dict[newer_stamp]["euler_angles"]["z"]]
            older_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*older_lla, self.utm_num)
            # r = scipy.spatial.transform.Rotation.from_euler('xyz', [older_euler[0], older_euler[1], older_euler[2]], degrees=True)
            # older_R_NED_FRD = r.as_matrix()
            # older_R_ENU_FRD = R_ENU_NED.dot(older_R_NED_FRD)
            # older_R_ENU_FLU = older_R_ENU_FRD.dot(R_FRD_FLU)
            # older_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, older_lla[0], older_lla[1], older_R_ENU_FLU)
            # older_q_ENU_FLU = util.Rab_to_qab(older_R_ENU_FLU)
            older_q_ENU_FLU =  [
                                nav_ins_dict[older_stamp]["quaternion"]['x'],
                                nav_ins_dict[older_stamp]["quaternion"]['y'],
                                nav_ins_dict[older_stamp]["quaternion"]['z'],
                                nav_ins_dict[older_stamp]["quaternion"]['w']
                            ]

            newer_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*newer_lla, self.utm_num)
            # r = scipy.spatial.transform.Rotation.from_euler('xyz', [newer_euler[0], newer_euler[1], newer_euler[2]], degrees=True)
            # newer_R_NED_FRD = r.as_matrix()
            # newer_R_ENU_FRD = R_ENU_NED.dot(newer_R_NED_FRD)
            # newer_R_ENU_FLU = newer_R_ENU_FRD.dot(R_FRD_FLU)
            # newer_R_ENU_FLU = R_TN_body_to_R_GN_body(self.utm_num, newer_lla[0], newer_lla[1], newer_R_ENU_FLU)
            # newer_q_ENU_FLU = util.Rab_to_qab(newer_R_ENU_FLU)
            newer_q_ENU_FLU =  [
                                nav_ins_dict[newer_stamp]["quaternion"]['x'],
                                nav_ins_dict[newer_stamp]["quaternion"]['y'],
                                nav_ins_dict[newer_stamp]["quaternion"]['z'],
                                nav_ins_dict[newer_stamp]["quaternion"]['w']
                            ]

            current_q, current_t = util.qt_chazhi(older_stamp, older_q_ENU_FLU, older_utm_xyz,
                                                  newer_stamp, newer_q_ENU_FLU, newer_utm_xyz, frame_stamp_ms)
            frame.ins_q_world_veh = current_q
            frame.ins_t_world_veh = current_t-self.t_utm_world
            frame.ins_position_type = ins_position_type

            # TODO:qzc use utm instead of opt_q_world_veh 
            frame.opt_q_world_veh = current_q
            frame.opt_t_world_veh = current_t-self.t_utm_world

        # 判断黑天白天
        for stamp_ms, v in nav_ins_dict.items():
            if util.is_day_time(v["position"]["longitude"], v["position"]["latitude"], float(stamp_ms/1000)) == True:
                self.trail_data.tag.append("day")
            else:
                self.trail_data.tag.append("night")
            break
        # self.ins_file_to_csv_qzc(ins_file_lines, os.path.join(self.data_folder, "ins.txt"))

    def read_veh_status_to_csv(self):
        file_path = os.path.join(self.data_folder, "vehicle_status.txt")
        if not os.path.exists(file_path):
            return
        WHEELDIR2INT = {"STANDSTILL": 0, "FORWARD": 1, "BACKWARD": 2, "UNKNOWN": 3}
        GEARTYPE2INT = {"INIT": 0, "R": 1, "N": 2, "D": 3, "P": 4, "INVALID": 7}

        def val_wheel(w):
            return WHEELDIR2INT[w] if w in WHEELDIR2INT else w

        def val_gear(g):
            return GEARTYPE2INT[g] if g in GEARTYPE2INT else g
        data = []
        lines = util.read_lines_from_file(file_path)
        for one_line in lines:
            j = json.loads(one_line)
            if j["header"]["stamp"] == 0:
                continue
            j_lon = j["vehicle_control_status"]["lon_control"]
            j_lat = j["vehicle_control_status"]["lat_control"]
            j_wheels = j_lon["vehicle_speed"]
            data.append([j["header"]["seq"],
                         j["header"]["stamp"],
                         j_wheels["vehicle_speed"],
                         j_wheels["wheel_speed_fl"]["speed"],
                         j_wheels["wheel_speed_fr"]["speed"],
                         j_wheels["wheel_speed_rl"]["speed"],
                         j_wheels["wheel_speed_rr"]["speed"],
                         j_wheels["wheel_speed_fl"]["pulse"],
                         j_wheels["wheel_speed_fr"]["pulse"],
                         j_wheels["wheel_speed_rl"]["pulse"],
                         j_wheels["wheel_speed_rr"]["pulse"],
                         val_wheel(j_wheels["wheel_speed_fl"]["direction"]),
                         val_wheel(j_wheels["wheel_speed_fr"]["direction"]),
                         val_wheel(j_wheels["wheel_speed_rl"]["direction"]),
                         val_wheel(j_wheels["wheel_speed_rr"]["direction"]),
                         val_gear(j["vehicle_body_status"]["gear"]["gear_pos"]),
                         j_lat["yaw_rate"],
                         j_lat["steering_angle"],
                         j_lat["steering_angle_speed"],
                         j_lon["longitude_acc"],
                         j_lat["lateral_acc"]])
        df = pd.DataFrame(data, columns=["id", "ts", "v", "v_fl", "v_fr", "v_rl", "v_rr", "o_fl", "o_fr", "o_rl",
                                         "o_rr", "dir_fl", "dir_fr", "dir_rl", "dir_rr", "gear", "yaw_rate",
                                         "steering_angle", "steering_angle_speed", "acc_lon", "acc_lat"])
        df.sort_values(by=['id'], inplace=True)
        df.to_csv(os.path.join(self.data_folder, "vehicle.txt"), sep=" ", index=False)

    def read_pp_laneline_file(self):
        file_path = os.path.join(self.data_folder, "perception_laneline.txt")
        lines = util.read_lines_from_file(file_path)
        for one_line in lines:
            item = json.loads(one_line)
            stamp_ms = item["timestamp"]
            self.pp_laneline_dict[stamp_ms] = item

    def add_scan_to_trail_data(self):
        scan_folder = os.path.join(self.data_folder, "lidar")
        pcd_files_path = util.get_files_path_with_suffix(scan_folder, ".pcd")
        for one_path in pcd_files_path:
            pcd_name = util.get_file_name_in_file_path(one_path)
            stamp_us = int(pcd_name.split(".")[0])
            stamp_ms = int(round(stamp_us/1000))
            frame_data: FrameData = FrameData()
            frame_data.scan_path = one_path
            frame_data.stamp_ms = stamp_ms
            self.trail_data.frame_dict[frame_data.stamp_ms] = frame_data

    def add_image_to_trail_data(self):
        class RawImageInfo(object):
            def __init__(self):
                self.stamp_ms: int = -1
                self.abs_path: str = ''
                self.file_name: str = ''
        image_dict: Dict[int, RawImageInfo] = {}
        all_jpg_file_path = sorted(util.get_files_path_with_suffix(os.path.join(self.data_folder, "front_camera_120"), '.jpg'))
        for raw_image_path in all_jpg_file_path:
            image_info: RawImageInfo = RawImageInfo()
            image_info.abs_path = raw_image_path
            image_info.file_name = util.get_file_name_in_file_path(raw_image_path)
            stamp_us = int(image_info.file_name.split(".")[0])
            stamp_ms = int(round(stamp_us/1000))
            image_info.stamp_ms = stamp_ms
            image_dict[image_info.stamp_ms] = image_info
        image_stamp_ms_list: List = list(image_dict.keys())
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(image_stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(image_stamp_ms_list):
                frame.flag.append("no_image")
                continue
            frame.front_image_path = image_dict[image_stamp_ms_list[loc]].abs_path

    def add_cloud_line_seg_to_trail_data(self):
        class RawLabelInfo(object):
            def __init__(self):
                self.stamp_ms: int = -1
                self.abs_path: str = ''
                self.file_name: str = ''
        label_dict: Dict[int, RawLabelInfo] = {}
        all_png_file_path = sorted(util.get_files_path_with_suffix(os.path.join(self.data_folder, "bevlane_f_ca_120"), '.png'))
        for raw_label_path in all_png_file_path:
            label_info: RawLabelInfo = RawLabelInfo()
            label_info.abs_path = raw_label_path
            label_info.file_name = util.get_file_name_in_file_path(raw_label_path)
            stamp_ms = int(label_info.file_name.split(".")[0])/1000
            label_info.stamp_ms = stamp_ms
            label_dict[label_info.stamp_ms] = label_info
        label_stamp_ms_list: List = list(label_dict.keys())
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(label_stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(label_stamp_ms_list):
                continue
            frame.cloud_line_seg_path = label_dict[label_stamp_ms_list[loc]].abs_path

    def add_cloud_pano_seg_to_trail_data(self):
        class RawLabelInfo(object):
            def __init__(self):
                self.stamp_ms: int = -1
                self.abs_path: str = ''
                self.file_name: str = ''
        label_dict: Dict[int, RawLabelInfo] = {}
        all_png_file_path = sorted(util.get_files_path_with_suffix(os.path.join(self.data_folder, "bevlane_semantic_mask_f_ca_120"), '.png'))
        for raw_label_path in all_png_file_path:
            label_info: RawLabelInfo = RawLabelInfo()
            label_info.abs_path = raw_label_path
            label_info.file_name = util.get_file_name_in_file_path(raw_label_path)
            stamp_ms = int(label_info.file_name.split(".")[0])/1000
            label_info.stamp_ms = stamp_ms
            label_dict[label_info.stamp_ms] = label_info
        label_stamp_ms_list: List = list(label_dict.keys())
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(label_stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(label_stamp_ms_list):
                continue
            frame.cloud_pano_seg_path = label_dict[label_stamp_ms_list[loc]].abs_path

    def add_pp_laneline_to_trail_data(self):
        stamp_ms_list: List = list(self.pp_laneline_dict.keys())
        if (len(stamp_ms_list) == 0):
            return
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            nearest_ms = min(stamp_ms_list, key=lambda x: abs(x-frame.stamp_ms))
            if abs(nearest_ms-frame.stamp_ms) > 50:
                continue
            lines = self.pp_laneline_dict[nearest_ms]["lines"]
            frame.pp_laneline_stamp = frame.stamp_ms
            if(len(lines) == 0):
                continue
            for line in lines:
                coeffs = line["coeffs"]
                if(len(coeffs) == 0):
                    continue
                if abs(coeffs[3]) >= 1e-04:
                    continue
                if "end_points" not in line.keys() or len(line["end_points"]) != 2:
                    continue

                start_x, end_x = min(line["end_points"][0]["x"], line["end_points"][1]["x"]), max(line["end_points"][0]["x"], line["end_points"][1]["x"])
                start_y, end_y = min(line["end_points"][0]["y"], line["end_points"][1]["y"]), max(line["end_points"][0]["y"], line["end_points"][1]["y"])

                line_type = format(line["type"], "b")
                if line["type"] >> 24 & 1:
                    # print(line["type"])
                    frame.pp_laneline_types.append(int(line["type"]))
                    frame.pp_laneline_coeffs.append(coeffs)
                    frame.pp_laneline_start_end_pts.append(line["end_points"])
                    c0 = coeffs[0]
                    c1 = coeffs[1]
                    c2 = coeffs[2]
                    c3 = coeffs[3]
                    veh_xy1 = []
                    for x_m in np.arange(start_x, end_x, 0.5):
                        y_m = c0+c1*x_m+c2*x_m*x_m+c3*x_m*x_m*x_m
                        # log.info("stamp_ms:{}    x_m:{}   y_m:{}  start_y:{} end_y:{}".format(frame.stamp_ms,x_m,y_m, start_y,  end_y))
                        # if (y_m > end_y or y_m < start_y):
                        #     continue
                        veh_xy1.append(["%.2f" % x_m, "%.2f" % y_m])
                    frame.pp_laneline_points.append(veh_xy1)

    def get_render_result(self):
        all_rgb_pcd_path = util.get_files_path_with_suffix(os.path.join(self.data_folder, "lidar_rgb"), ".pcd")
        for rgb_pcd_path in all_rgb_pcd_path:
            rgb_pcd_name = util.get_file_name_in_file_path(rgb_pcd_path)
            stamp_ms = int(rgb_pcd_name.split(".")[0])
            veh_scan_path = os.path.join(self.data_folder, "veh_scan", f"{stamp_ms}.pcd")
            un_image_path = os.path.join(self.data_folder, "un_image", f"{stamp_ms}.jpg")
            un_image_seg_path = os.path.join(self.data_folder, "un_image_seg", f"{stamp_ms}.png")
            if stamp_ms not in self.trail_data.frame_dict.keys():
                continue
            if not os.path.exists(veh_scan_path):
                continue
            if not os.path.exists(un_image_path):
                continue
            self.trail_data.frame_dict[stamp_ms].scan_rgb_path = rgb_pcd_path  # pcd_name就是以时间戳命名的
            self.trail_data.frame_dict[stamp_ms].veh_scan_path = veh_scan_path
            self.trail_data.frame_dict[stamp_ms].un_image_path = un_image_path
            self.trail_data.frame_dict[stamp_ms].un_image_seg_path = un_image_seg_path
        to_del = []
        for frame in self.trail_data.frame_dict.values():
            if frame.scan_rgb_path == "" or frame.veh_scan_path == "" or frame.un_image_path == "":
                log.warning("一帧点云生成为空", frame.scan_path)
                to_del.append(frame.stamp_ms)
        for one in to_del:
            del self.trail_data.frame_dict[one]
        key_frame_id: int = 0
        for frame in self.trail_data.frame_dict.values():
            frame.key_frame_id = key_frame_id
            key_frame_id += 1
        un_image_param = util.json_file_to_dict(os.path.join(self.data_folder, "un_image_param.json"))
        if un_image_param != {}:
            self.trail_data.un_cam_width = un_image_param["width"]
            self.trail_data.un_cam_height = un_image_param["height"]
            self.trail_data.un_cam_fxycxy = un_image_param["fxycxy"]
            self.trail_data.un_cam_K = util.fxycxy_to_K(self.trail_data.un_cam_fxycxy)

    def get_render_result_qzc(self):
        return

    def read_ndm_loc_to_trail_data(self):
        ndm_loc_dict: Dict[int, Dict] = {}
        lines = util.read_lines_from_file(os.path.join(self.data_folder, "ndm_location.txt"))
        for one_line in lines:
            item = json.loads(one_line)
            stamp_ms = item["header"]["stamp"]
            ndm_loc_dict[stamp_ms] = item
        ndm_loc_dict = {key: ndm_loc_dict[key] for key in sorted(ndm_loc_dict.keys())}
        loc_stamp_ms_list: List = list(ndm_loc_dict.keys())
        frame: FrameData
        for frame in self.trail_data.lidar_frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(loc_stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(loc_stamp_ms_list):
                frame.flag.append("no_loc")
                continue
            older_stamp = loc_stamp_ms_list[loc-1]
            older_lla = [ndm_loc_dict[older_stamp]["location"]["coordinate"]["lon"],
                         ndm_loc_dict[older_stamp]["location"]["coordinate"]["lat"],
                         ndm_loc_dict[older_stamp]["location"]["coordinate"]["height"]]
            older_q = [ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["x"],
                       ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["y"],
                       ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["z"],
                       ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["w"]]
            newer_stamp = loc_stamp_ms_list[loc]
            newer_lla = [ndm_loc_dict[newer_stamp]["location"]["coordinate"]["lon"],
                         ndm_loc_dict[newer_stamp]["location"]["coordinate"]["lat"],
                         ndm_loc_dict[newer_stamp]["location"]["coordinate"]["height"]]
            newer_q = [ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["x"],
                       ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["y"],
                       ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["z"],
                       ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["w"]]
            older_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*older_lla, self.utm_num)
            newer_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*newer_lla, self.utm_num)
            current_q, current_t = util.qt_chazhi(older_stamp, older_q, older_utm_xyz,
                                                  newer_stamp, newer_q, newer_utm_xyz, frame_stamp_ms)
            frame.loc_q_world_veh = current_q
            frame.loc_t_world_veh = current_t-self.t_utm_world
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(loc_stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(loc_stamp_ms_list):
                frame.flag.append("no_loc")
                continue
            older_stamp = loc_stamp_ms_list[loc-1]
            older_lla = [ndm_loc_dict[older_stamp]["location"]["coordinate"]["lon"],
                         ndm_loc_dict[older_stamp]["location"]["coordinate"]["lat"],
                         ndm_loc_dict[older_stamp]["location"]["coordinate"]["height"]]
            older_q = [ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["x"],
                       ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["y"],
                       ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["z"],
                       ndm_loc_dict[older_stamp]["location"]["odom"]["quaternion"]["w"]]
            newer_stamp = loc_stamp_ms_list[loc]
            newer_lla = [ndm_loc_dict[newer_stamp]["location"]["coordinate"]["lon"],
                         ndm_loc_dict[newer_stamp]["location"]["coordinate"]["lat"],
                         ndm_loc_dict[newer_stamp]["location"]["coordinate"]["height"]]
            newer_q = [ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["x"],
                       ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["y"],
                       ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["z"],
                       ndm_loc_dict[newer_stamp]["location"]["odom"]["quaternion"]["w"]]
            older_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*older_lla, self.utm_num)
            newer_utm_xyz = util.lonlatalt_utm_num_to_utm_xyz(*newer_lla, self.utm_num)
            current_q, current_t = util.qt_chazhi(older_stamp, older_q, older_utm_xyz,
                                                  newer_stamp, newer_q, newer_utm_xyz, frame_stamp_ms)
            frame.loc_q_world_veh = current_q
            frame.loc_t_world_veh = current_t-self.t_utm_world

    def odom_file_to_csv(self, odom_file_lines: list, csv_path: str):
        data = []
        for one_line in odom_file_lines:
            j = json.loads(one_line)
            data.append([j["header"]["frame_id"],
                         j["header"]["stamp"],
                         j["ts"],
                         j["position"]["x"],
                         j["position"]["y"],
                         j["position"]["z"],
                         j["quaternion"]["w"],
                         j["quaternion"]["x"],
                         j["quaternion"]["y"],
                         j["quaternion"]["z"],
                         j["speed"]])
        df = pd.DataFrame(data, columns=["id", "ts", "local_time", "x", "y", "z", "qw", "qx", "qy", "qz", "speed"])
        df.sort_values(by=['id'], inplace=True)
        df.to_csv(csv_path, sep=" ", index=False)

    def read_odom_to_trail_data(self):
        odom_dict: Dict[int, Dict] = {}
        odom_file_lines = util.read_lines_from_file(os.path.join(self.data_folder, "odometry.txt"))
        for one_line in odom_file_lines:
            j = json.loads(one_line)
            # stamp_ms = j["header"]["stamp"]
            stamp_us = int(j["header"]["stamp"])
            stamp_ms = int(round(stamp_us/1000))

            odom_dict[stamp_ms] = j
            frame_id = j["header"]["frame_id"]
            self.odom_frame_id_to_stamp_ms[frame_id] = stamp_ms
            self.odom_frame_id_to_stamp_local[frame_id] = j["ts"]
        odom_dict = {key: odom_dict[key] for key in sorted(odom_dict.keys())}
        stamp_ms_list: List = list(odom_dict.keys())
        frame: FrameData
        for frame in self.trail_data.lidar_frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(stamp_ms_list):
                frame.flag.append("no_odom")
                continue
            older_stamp = stamp_ms_list[loc-1]
            older_xyz = [odom_dict[older_stamp]["position"]["x"],
                         odom_dict[older_stamp]["position"]["y"],
                         odom_dict[older_stamp]["position"]["z"]]
            older_q = [odom_dict[older_stamp]["quaternion"]["x"],
                       odom_dict[older_stamp]["quaternion"]["y"],
                       odom_dict[older_stamp]["quaternion"]["z"],
                       odom_dict[older_stamp]["quaternion"]["w"]]
            newer_stamp = stamp_ms_list[loc]
            newer_xyz = [odom_dict[newer_stamp]["position"]["x"],
                         odom_dict[newer_stamp]["position"]["y"],
                         odom_dict[newer_stamp]["position"]["z"]]
            newer_q = [odom_dict[newer_stamp]["quaternion"]["x"],
                       odom_dict[newer_stamp]["quaternion"]["y"],
                       odom_dict[newer_stamp]["quaternion"]["z"],
                       odom_dict[newer_stamp]["quaternion"]["w"]]
            frame.odom_q_odom_veh, frame.odom_t_odom_veh = util.qt_chazhi(older_stamp, older_q, older_xyz,
                                                                          newer_stamp, newer_q, newer_xyz,
                                                                          frame_stamp_ms)
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(stamp_ms_list):
                frame.flag.append("no_odom")
                continue
            older_stamp = stamp_ms_list[loc-1]
            older_xyz = [odom_dict[older_stamp]["position"]["x"],
                         odom_dict[older_stamp]["position"]["y"],
                         odom_dict[older_stamp]["position"]["z"]]
            older_q = [odom_dict[older_stamp]["quaternion"]["x"],
                       odom_dict[older_stamp]["quaternion"]["y"],
                       odom_dict[older_stamp]["quaternion"]["z"],
                       odom_dict[older_stamp]["quaternion"]["w"]]
            newer_stamp = stamp_ms_list[loc]
            newer_xyz = [odom_dict[newer_stamp]["position"]["x"],
                         odom_dict[newer_stamp]["position"]["y"],
                         odom_dict[newer_stamp]["position"]["z"]]
            newer_q = [odom_dict[newer_stamp]["quaternion"]["x"],
                       odom_dict[newer_stamp]["quaternion"]["y"],
                       odom_dict[newer_stamp]["quaternion"]["z"],
                       odom_dict[newer_stamp]["quaternion"]["w"]]
            frame.odom_q_odom_veh, frame.odom_t_odom_veh = util.qt_chazhi(older_stamp, older_q, older_xyz,
                                                                          newer_stamp, newer_q, newer_xyz,
                                                                          frame_stamp_ms)
        self.odom_file_to_csv(odom_file_lines, os.path.join(self.data_folder, "liodometry.txt"))

    def read_opt_pose_to_trail_data_qzc(self):
        opt_pose_dict: Dict[int, Dict] = {}
        odom_file_lines = util.read_lines_from_file(os.path.join(self.data_folder, "opt_pose_veh.txt"))
        for one_line in odom_file_lines:
            j = json.loads(one_line)
            # stamp_ms = j["header"]["stamp"]
            stamp_us = int(j["header"]["stamp"])
            stamp_ms = int(round(stamp_us/1000))

            opt_pose_dict[stamp_ms] = j
            frame_id = j["header"]["frame_id"]
            self.odom_frame_id_to_stamp_ms[frame_id] = stamp_ms
            self.odom_frame_id_to_stamp_local[frame_id] = j["ts"]
        opt_pose_dict = {key: opt_pose_dict[key] for key in sorted(opt_pose_dict.keys())}
        stamp_ms_list: List = list(opt_pose_dict.keys())
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(stamp_ms_list):
                frame.flag.append("no_odom")
                continue
            older_stamp = stamp_ms_list[loc-1]
            older_xyz = [opt_pose_dict[older_stamp]["position"]["x"],
                         opt_pose_dict[older_stamp]["position"]["y"],
                         opt_pose_dict[older_stamp]["position"]["z"]]
            older_q = [opt_pose_dict[older_stamp]["quaternion"]["x"],
                       opt_pose_dict[older_stamp]["quaternion"]["y"],
                       opt_pose_dict[older_stamp]["quaternion"]["z"],
                       opt_pose_dict[older_stamp]["quaternion"]["w"]]
            newer_stamp = stamp_ms_list[loc]
            newer_xyz = [opt_pose_dict[newer_stamp]["position"]["x"],
                         opt_pose_dict[newer_stamp]["position"]["y"],
                         opt_pose_dict[newer_stamp]["position"]["z"]]
            newer_q = [opt_pose_dict[newer_stamp]["quaternion"]["x"],
                       opt_pose_dict[newer_stamp]["quaternion"]["y"],
                       opt_pose_dict[newer_stamp]["quaternion"]["z"],
                       opt_pose_dict[newer_stamp]["quaternion"]["w"]]
            frame.opt_q_world_veh, frame.opt_t_world_veh = util.qt_chazhi(older_stamp, older_q, older_xyz,
                                                                          newer_stamp, newer_q, newer_xyz,
                                                                          frame_stamp_ms)
    def odom3d_file_to_csv(self, odom3d_lines, csv_path):
        data = []
        for one_line in odom3d_lines:
            j = json.loads(one_line)
            frame_id = j["header"]["frame_id"]
            if frame_id not in list(self.odom_frame_id_to_stamp_ms.keys()):
                continue
            stamp_ms = self.odom_frame_id_to_stamp_ms[frame_id]
            local_time = self.odom_frame_id_to_stamp_local[frame_id]
            data.append([j["header"]["frame_id"],
                         stamp_ms,
                         local_time,
                         j["position"]["x"],
                         j["position"]["y"],
                         j["position"]["z"],
                         j["quaternion"]["w"],
                         j["quaternion"]["x"],
                         j["quaternion"]["y"],
                         j["quaternion"]["z"],
                         j["speed"]])
        df = pd.DataFrame(data, columns=["id", "ts", "local_time", "x", "y", "z", "qw", "qx", "qy", "qz", "speed"])
        df.sort_values(by=['id'], inplace=True)
        df.to_csv(csv_path, sep=" ", index=False)

    def read_odom3d_to_trail_data(self):
        odom3d_dict: Dict[int, Dict] = {}
        odom3d_lines = util.read_lines_from_file(os.path.join(self.data_folder, "odometry3d.txt"))
        for one_line in odom3d_lines:
            j = json.loads(one_line)
            frame_id = j["header"]["frame_id"]
            if frame_id not in list(self.odom_frame_id_to_stamp_ms.keys()):
                continue
            stamp_ms = self.odom_frame_id_to_stamp_ms[frame_id]
            odom3d_dict[stamp_ms] = j

        odom3d_dict = {key: odom3d_dict[key] for key in sorted(odom3d_dict.keys())}
        stamp_ms_list: List = list(odom3d_dict.keys())
        frame: FrameData
        for frame in self.trail_data.lidar_frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(stamp_ms_list):
                frame.flag.append("no_odom3d")
                continue
            older_stamp = stamp_ms_list[loc-1]
            older_xyz = [odom3d_dict[older_stamp]["position"]["x"],
                         odom3d_dict[older_stamp]["position"]["y"],
                         odom3d_dict[older_stamp]["position"]["z"]]
            older_q = [odom3d_dict[older_stamp]["quaternion"]["x"],
                       odom3d_dict[older_stamp]["quaternion"]["y"],
                       odom3d_dict[older_stamp]["quaternion"]["z"],
                       odom3d_dict[older_stamp]["quaternion"]["w"]]
            newer_stamp = stamp_ms_list[loc]
            newer_xyz = [odom3d_dict[newer_stamp]["position"]["x"],
                         odom3d_dict[newer_stamp]["position"]["y"],
                         odom3d_dict[newer_stamp]["position"]["z"]]
            newer_q = [odom3d_dict[newer_stamp]["quaternion"]["x"],
                       odom3d_dict[newer_stamp]["quaternion"]["y"],
                       odom3d_dict[newer_stamp]["quaternion"]["z"],
                       odom3d_dict[newer_stamp]["quaternion"]["w"]]
            frame.odom3d_q_odom_veh, frame.odom3d_t_odom_veh = util.qt_chazhi(older_stamp, older_q, older_xyz,
                                                                              newer_stamp, newer_q, newer_xyz,
                                                                              frame_stamp_ms)
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(stamp_ms_list, frame_stamp_ms)
            if loc == 0 or loc == len(stamp_ms_list):
                frame.flag.append("no_odom3d")
                continue
            older_stamp = stamp_ms_list[loc-1]
            older_xyz = [odom3d_dict[older_stamp]["position"]["x"],
                         odom3d_dict[older_stamp]["position"]["y"],
                         odom3d_dict[older_stamp]["position"]["z"]]
            older_q = [odom3d_dict[older_stamp]["quaternion"]["x"],
                       odom3d_dict[older_stamp]["quaternion"]["y"],
                       odom3d_dict[older_stamp]["quaternion"]["z"],
                       odom3d_dict[older_stamp]["quaternion"]["w"]]
            newer_stamp = stamp_ms_list[loc]
            newer_xyz = [odom3d_dict[newer_stamp]["position"]["x"],
                         odom3d_dict[newer_stamp]["position"]["y"],
                         odom3d_dict[newer_stamp]["position"]["z"]]
            newer_q = [odom3d_dict[newer_stamp]["quaternion"]["x"],
                       odom3d_dict[newer_stamp]["quaternion"]["y"],
                       odom3d_dict[newer_stamp]["quaternion"]["z"],
                       odom3d_dict[newer_stamp]["quaternion"]["w"]]
            frame.odom3d_q_odom_veh, frame.odom3d_t_odom_veh = util.qt_chazhi(older_stamp, older_q, older_xyz,
                                                                              newer_stamp, newer_q, newer_xyz,
                                                                              frame_stamp_ms)
        self.odom3d_file_to_csv(odom3d_lines, os.path.join(self.data_folder, "liodometry_3d.txt"))

    def add_perception_raw_to_trail_data(self):
        log.info("read perception_raw ...")
        image_seg_dict: Dict[int, str] = {}
        all_txt_file_path = sorted(util.get_files_path_with_suffix(os.path.join(self.data_folder, "perception_raw"), '.txt'))
        for raw_txt_path in all_txt_file_path:
            item = util.json_file_to_dict(raw_txt_path)
            if not item:
                continue
            stamp_ms = item["img_desc"][0]["time_stamp"]
            image_seg_dict[stamp_ms] = raw_txt_path
        stamp_ms_list: List = list(image_seg_dict.keys())
        if (len(stamp_ms_list) == 0):
            return
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            nearest_ms = min(stamp_ms_list, key=lambda x: abs(x-frame.stamp_ms))
            if(abs(nearest_ms-frame.stamp_ms)) > 50:
                frame.flag.append("no_perception_raw")
                frame.flag.append(f"nearest perception {nearest_ms}")
            else:
                frame.perception_data_path: str = image_seg_dict[nearest_ms]  # 第一个相机为前视120

    def remove_incomplete_frame(self):
        # 1 处理bev
        to_del = []
        for frame in self.trail_data.frame_dict.values():
            if (("no_ins" in frame.flag) or ("no_odom" in frame.flag) or ("no_image" in frame.flag)):
                log.warning("bad frame.flag:", frame.flag, frame.scan_path)
                to_del.append(frame.stamp_ms)
        log.info("bev: 剔除条件 no_ins + no_odom + no_image, 剔除数", len(to_del))
        for one in to_del:
            del self.trail_data.frame_dict[one]
        log.info("bev: remove_incomplete_frame 后的数量为：", len(self.trail_data.frame_dict))

        # 处理 lidar
        to_del = []
        for frame in self.trail_data.lidar_frame_dict.values():
            if (("no_ins" in frame.flag) or ("no_odom" in frame.flag) or ("no_image" in frame.flag)):
                log.warning("bad frame.flag:", frame.flag, frame.scan_path)
                to_del.append(frame.stamp_ms)
        log.info("lidar: 剔除条件 no_ins + no_odom + no_image, 剔除数", len(to_del))
        for one in to_del:
            del self.trail_data.lidar_frame_dict[one]
        log.info("lidar: remove_incomplete_frame 后的数量为：", len(self.trail_data.lidar_frame_dict))

    def remove_out_data_scope_frame_qzc(self):
        if self.data_scope == "":
            log.info("self.data_scope 未设置，不基于 data_scope 剔除")
            return
        data_scope_poly = shapely.wkt.loads(self.data_scope)
        # min_stamp_ms = 1e18
        # max_stamp_ms = 0
        # 1 处理 BEV
        to_del = []
        for frame in self.trail_data.frame_dict.values():
            if data_scope_poly.intersects(Point(frame.ins_t_world_veh[0:2])):
                pass
            else:
                to_del.append(frame.stamp_ms)
            # if data_scope_poly.intersects(Point(frame.ins_t_world_veh[0:2])):
            #     if frame.stamp_ms < min_stamp_ms:
            #         min_stamp_ms = frame.stamp_ms
            #     if frame.stamp_ms > max_stamp_ms:
            #         max_stamp_ms = frame.stamp_ms
        # log.warning(min_stamp_ms, max_stamp_ms)
        # for frame in self.trail_data.frame_dict.values():
        #     if frame.stamp_ms > min_stamp_ms and frame.stamp_ms < max_stamp_ms:
        #         pass
        #     else:
        #         to_del.append(frame.stamp_ms)
        for one in to_del:
            del self.trail_data.frame_dict[one]
        log.info("BEV: 剔除条件 out data_scope 剔除数", len(to_del))

        # 2 处理 lidar
        to_del = []
        for frame in self.trail_data.lidar_frame_dict.values():
            if data_scope_poly.intersects(Point(frame.ins_t_world_veh[0:2])):
                pass
            else:
                to_del.append(frame.stamp_ms)
        for one in to_del:
            del self.trail_data.lidar_frame_dict[one]
        log.info("lidar: 剔除条件 out data_scope 剔除数", len(to_del))

    def remove_out_data_scope_frame(self):
        if self.data_scope == "":
            log.info("self.data_scope 未设置，不基于 data_scope 剔除")
            return
        data_scope_poly = shapely.wkt.loads(self.data_scope)
        min_stamp_ms = 1e18
        max_stamp_ms = 0
        for frame in self.trail_data.frame_dict.values():
            if data_scope_poly.intersects(Point(frame.ins_t_world_veh[0:2])):
                if frame.stamp_ms < min_stamp_ms:
                    min_stamp_ms = frame.stamp_ms
                if frame.stamp_ms > max_stamp_ms:
                    max_stamp_ms = frame.stamp_ms
        to_del = []
        log.warning(min_stamp_ms, max_stamp_ms)
        for frame in self.trail_data.frame_dict.values():
            if frame.stamp_ms > min_stamp_ms and frame.stamp_ms < max_stamp_ms:
                pass
            else:
                to_del.append(frame.stamp_ms)
        for one in to_del:
            del self.trail_data.frame_dict[one]
        log.info("剔除条件 out data_scope 剔除数", len(to_del))

    def compute_curvature(self, x, y):
        """
        快速计算离散点的曲率。
        :param x: x 坐标数组
        :param y: y 坐标数组
        :return: 曲率数组
        """
        n = len(x)
        curvature = np.zeros(n)
        
        for i in range(1, n - 1):
            dx1 = x[i] - x[i - 1]
            dy1 = y[i] - y[i - 1]
            dx2 = x[i + 1] - x[i]
            dy2 = y[i + 1] - y[i]
            
            numerator = np.abs(dx1 * dy2 - dx2 * dy1)
            denominator = (dx1**2 + dy1**2)**1.5
            
            if denominator > 1e-6:
                curvature[i] = numerator / denominator
        
        return curvature

    def remove_range_by_length(self, all_ins_t, remove_length, init_index):
        min_index = init_index
        max_index = init_index
        all_len = len(all_ins_t)

        length = 0.0
        for i in range(init_index, 0, -1):
            length += np.linalg.norm(all_ins_t[i][:2] - all_ins_t[min_index][:2])
            if length < remove_length:
                min_index = i
            else:
                break

        length = 0.0
        for i in range(init_index, all_len, 1):
            length += np.linalg.norm(all_ins_t[i][:2] - all_ins_t[max_index][:2])
            if length < remove_length:
                max_index = i
            else:
                break
        
        return min_index, max_index

    def remove_return_frame(self):
        all_ins_t = []
        for frame in self.trail_data.frame_dict.values():
            all_ins_t.append(frame.ins_t_world_veh)
        if len(all_ins_t) < 10:
            return

        all_ins_t = np.array(all_ins_t)
        curvature = self.compute_curvature(all_ins_t[:,0], all_ins_t[:,1])
        
        if(curvature.max() > 0.15):# 0.2 = 1/R (R=5m), 0.15 = 1/R(6m)
            max_init_index = np.argmax(curvature)
            min_index, max_index = self.remove_range_by_length(all_ins_t, 30, max_init_index)
            to_del = []
            index = 0
            mean_curvatue = np.mean(curvature[min_index:max_index])
            log.info("max:", curvature.max(), " index: ", max_init_index, " mean_curvatue:", mean_curvatue)
            if(mean_curvatue > 0.05):
                for frame in self.trail_data.frame_dict.values():
                    if index >= min_index and index <= max_index:
                        to_del.append(frame.stamp_ms)
                    index += 1
                for one in to_del:
                    del self.trail_data.frame_dict[one]
                log.info("剔除条件 remove_return_frame 剔除数", len(to_del))

    def bing_frame_to_link(self):
        if len(self.binged_link_dict) == 0:
            log.info("self.link_dict 未设置，不基于 link 剔除")
            return

        def bing_frame_and_link(frame: FrameData, link_info: LinkInfo):
            frame_heading = util.qab_to_euler_fix_yaw_deg(frame.ins_q_world_veh)
            for i in range(len(link_info.link_nodes)):
                node = link_info.link_nodes[i]
                xy_dis = np.linalg.norm([node[0]-frame.ins_t_world_veh[0],
                                        node[1]-frame.ins_t_world_veh[1]])
                if link_info.link_direction == 3:
                    heading_dis = 0
                else:
                    heading_dis = abs(util.angle_distance_deg(link_info.link_nodes_heading[i], frame_heading))
                if (xy_dis < 25 and (heading_dis < 60)):
                    frame.link_id_list.append(link_info.link_id)
                    return

        def bing_frame_and_all_link(frame: FrameData):
            for link_info in self.binged_link_dict.values():
                bing_frame_and_link(frame, link_info)
            if len(frame.link_id_list) == 0:
                min_xy_dis = 10000
                min_link_id = 0
                frame_heading = util.qab_to_euler_fix_yaw_deg(frame.ins_q_world_veh)
                for link_info in self.binged_link_dict.values():
                    for i in range(len(link_info.link_nodes)):
                        node = link_info.link_nodes[i]
                        xy_dis = np.linalg.norm([node[0]-frame.ins_t_world_veh[0],
                                                 node[1]-frame.ins_t_world_veh[1]])
                        heading_dis = abs(util.angle_distance_deg(link_info.link_nodes_heading[i],
                                                                  frame_heading))
                        if xy_dis > 10:
                            continue
                        if link_info.link_direction == 1 and heading_dis > 10:
                            continue
                        if link_info.link_direction == 3 and heading_dis > 10 and heading_dis < 170:
                            continue
                        if xy_dis < min_xy_dis:
                            min_xy_dis = xy_dis
                            min_link_id = link_info.link_id
                if min_link_id != 0:
                    frame.link_id_list.append(link_info.link_id)

        for frame in self.trail_data.frame_dict.values():
            bing_frame_and_all_link(frame)

    def remove_near_frame(self):
        to_del = []
        t_world_last_frame = np.array([-1e12, -1e12, -1e12])
        world_last_heading_deg = 0
        for frame in self.trail_data.frame_dict.values():
            this_heading_deg = util.qab_to_euler_fix_yaw_deg(frame.ins_q_world_veh)
            line_diss = np.linalg.norm(frame.ins_t_world_veh-t_world_last_frame)
            heading_diss = abs(util.angle_distance_deg(world_last_heading_deg, this_heading_deg))
            if line_diss < 2 and heading_diss < 10:
                to_del.append(frame.stamp_ms)
            else:
                t_world_last_frame = frame.ins_t_world_veh
                world_last_heading_deg = this_heading_deg
        for one in to_del:
            del self.trail_data.frame_dict[one]

    def remove_abnormal_frame(self):
        to_del = []
        world_last_heading_deg = 0
        heading_diss_list = []

        for frame in self.trail_data.frame_dict.values():
            this_heading_deg = util.qab_to_euler_fix_pitch_deg(frame.ins_q_world_veh)
            heading_diss = abs(util.angle_distance_deg(world_last_heading_deg, this_heading_deg))
            heading_diss_list.append(heading_diss)

        mean_diss = np.mean(heading_diss_list)
        std_diss = np.std(heading_diss_list)

        upper_threshold = mean_diss + 1*std_diss
        lower_threshold = mean_diss - 3*std_diss
        # log.info(f"mean_diss:{mean_diss}, std_diss:{std_diss}, upper_threshold:{upper_threshold}, lower_threshold:{lower_threshold}")
            
        for frame in self.trail_data.frame_dict.values():
            this_heading_deg = util.qab_to_euler_fix_pitch_deg(frame.ins_q_world_veh)
            heading_diss = abs(util.angle_distance_deg(world_last_heading_deg, this_heading_deg))    
            
            if heading_diss > upper_threshold or heading_diss < lower_threshold:
                to_del.append(frame.stamp_ms)
            else:
                world_last_heading_deg = this_heading_deg
        for one in to_del:
            del self.trail_data.frame_dict[one]

    def read_trail_info_and_densify(self):  # 读 trail 信息，增密 link 节点
        def make_link_info(one):
            link_info: LinkInfo = LinkInfo()
            link_info.link_id = int(one["link_id"])
            link_info.link_nodes = densify_geometry(shapely.wkt.loads(one["line_string"]), 0.5).coords[:]
            link_info.link_direction = one["link_direction"]
            for i in range(0, len(link_info.link_nodes)-1):
                node1 = link_info.link_nodes[i]
                node2 = link_info.link_nodes[i+1]
                heading = np.arctan2(node2[1]-node1[1], node2[0]-node1[0])*57.29578
                link_info.link_nodes_heading.append(heading)
            link_info.link_nodes_heading.append(link_info.link_nodes_heading[-1])
            return link_info

        trail_info = util.json_file_to_dict(os.path.join(self.data_folder, "trail_info.json"))
        self.trail_data.trail_id = str(trail_info["trail_id"])
        self.trail_data.tile_id = int(trail_info["download_tile_id"])
        for one in trail_info["binded_links"]:
            link_info: LinkInfo = make_link_info(one)
            self.binged_link_dict[link_info.link_id] = link_info
            self.trail_data.link_id_list.append(link_info.link_id)
        self.data_scope = trail_info["data_scope"]

    def remove_discontinuous_part(self):
        parts_frame = []  # 记录片段[[起始时间戳,终止时间戳,长度m],[起始时间戳,终止时间戳,长度m],...]
        if len(self.trail_data.frame_dict.values()) < 3:
            return
        first_frame: FrameData = list(self.trail_data.frame_dict.values())[0]
        parts_frame.append([first_frame, first_frame, 0])
        for frame in self.trail_data.frame_dict.values():
            last_frame: FrameData = parts_frame[-1][1]
            diff_ms = frame.stamp_ms-last_frame.stamp_ms
            T1 = util.qab_tab_to_Tab(frame.ins_q_world_veh, frame.ins_t_world_veh)
            T2 = util.qab_tab_to_Tab(last_frame.ins_q_world_veh, last_frame.ins_t_world_veh)
            delta_T = util.Tab_to_Tba(T1).dot(T2)
            delta_t = util.Tab_to_tab(delta_T)
            delta_q = util.Tab_to_qab(delta_T)
            diff_xyz = np.linalg.norm(np.array(delta_t))
            diff_z = delta_t[2]
            diff_y = delta_t[1]
            diff_deg = util.qab_to_euler_fix_yaw_deg(delta_q)
            data_break = False
            if (diff_ms > 2000 and (diff_xyz > 2 or diff_deg > 10)):
                data_break = True
            elif (diff_ms < 2000 and diff_z > 2):
                data_break = True
            elif (diff_ms < 2000 and diff_y > 2):
                data_break = True
            if data_break:
                log.warning(f"trail 数据有断点: diff_ms:{diff_ms} ms, diff_xyz:{diff_xyz} m, diff_z:{diff_z} m, diff_y:{diff_y} m, diff_deg:{diff_deg} deg, trail id: {self.trail_data.trail_id}, stamp: {frame.stamp_ms}")
                parts_frame.append([frame, frame, 0])
            else:
                parts_frame[-1][1] = frame
                parts_frame[-1][2] += diff_xyz
        best_part = parts_frame[0]
        for one_part in parts_frame:
            if one_part[2] > best_part[2]:
                best_part = one_part
        min_stamp_ms = best_part[0].stamp_ms
        max_stamp_ms = best_part[1].stamp_ms
        to_del = []
        for frame in self.trail_data.frame_dict.values():
            if frame.stamp_ms >= min_stamp_ms and frame.stamp_ms <= max_stamp_ms:
                pass
            else:
                to_del.append(frame.stamp_ms)
        for one in to_del:
            del self.trail_data.frame_dict[one]

    def add_bev_label_to_trail_data_qzc(self):
        add_folder_if_no_exist(os.path.join(self.data_folder, "bev_label_vis"))
        add_folder_if_no_exist(os.path.join(self.data_folder, "bev_label_txt"))

        # cmd = "python3 {} --bev_label_folder={} --bev_label_vis_folder={} --bev_label_txt_folder={}".format(
        #     os.path.join(lidar_mapping_root_folder, "preprocess/script/bev_label_vis.py"),
        #     os.path.join(self.data_folder, "bev_label"),
        #     os.path.join(self.data_folder, "bev_label_vis"),
        #     os.path.join(self.data_folder, "bev_label_txt"))
        # util.run_a_process(cmd)

        bev_label_dict: Dict[int, str] = {}
        all_bev_label_path = sorted(util.get_files_path_with_suffix(os.path.join(self.data_folder, "bev_label"), '.json'))
        if all_bev_label_path == []:
            return

        if len(self.trail_data.frame_dict) == 0:
            for bev_label_path in all_bev_label_path:
                stamp_us = int(util.get_file_name_in_file_path(bev_label_path).split(".")[0])
                stamp_ms = int(round(stamp_us/1000))

                frame_data: FrameData = FrameData()
                frame_data.stamp_ms = stamp_ms
                frame_data.bev_label_path = bev_label_path
                self.trail_data.frame_dict[frame_data.stamp_ms] = frame_data
        else:
            for bev_label_path in all_bev_label_path:
                stamp_us = int(util.get_file_name_in_file_path(bev_label_path).split(".")[0])
                stamp_ms = int(round(stamp_us/1000))
                bev_label_dict[stamp_ms] = bev_label_path
            bev_label_dict = {key: bev_label_dict[key] for key in sorted(bev_label_dict.keys())}
            bev_label_stamp_list: List = list(bev_label_dict.keys())

            frame: FrameData
            for frame in self.trail_data.frame_dict.values():
                frame_stamp_ms = frame.stamp_ms
                loc = bisect.bisect_left(bev_label_stamp_list, frame_stamp_ms)
                if loc == 0 or loc == len(bev_label_stamp_list):
                    frame.flag.append("no_bev_label")
                    continue
                frame.bev_label_path = bev_label_dict[bev_label_stamp_list[loc]]

    def add_lidar_label_to_trail_data_qzc(self):
        # add_folder_if_no_exist(os.path.join(self.data_folder, "bev_label_vis"))
        # add_folder_if_no_exist(os.path.join(self.data_folder, "bev_label_txt"))

        # cmd = "python3 {} --bev_label_folder={} --bev_label_vis_folder={} --bev_label_txt_folder={}".format(
        #     os.path.join(lidar_mapping_root_folder, "preprocess/script/bev_label_vis.py"),
        #     os.path.join(self.data_folder, "bev_label"),
        #     os.path.join(self.data_folder, "bev_label_vis"),
        #     os.path.join(self.data_folder, "bev_label_txt"))
        # util.run_a_process(cmd)

        lidar_label_dict: Dict[int, str] = {}
        lidar_label_dict = util.json_file_to_dict(os.path.join(self.data_folder, "lidar_label", "all_pcd_path.json"))
        if lidar_label_dict == {} or "pcd_files" not in lidar_label_dict:
            return

        all_lidar_label_path = lidar_label_dict["pcd_files"]
        if all_lidar_label_path == []:
            return

        if len(self.trail_data.lidar_frame_dict) == 0:
            for lidar_label_path in all_lidar_label_path:
                stamp_us = int(util.get_file_name_in_file_path(lidar_label_path).split(".")[0])
                stamp_ms = int(round(stamp_us/1000))

                frame_data: FrameData = FrameData()
                frame_data.stamp_ms = stamp_ms
                frame_data.lidar_label_path = lidar_label_path
                self.trail_data.lidar_frame_dict[frame_data.stamp_ms] = frame_data
        else:
            for lidar_label_path in all_lidar_label_path:
                stamp_us = int(util.get_file_name_in_file_path(lidar_label_path).split(".")[0])
                stamp_ms = int(round(stamp_us/1000))
                lidar_label_dict[stamp_ms] = lidar_label_path
            lidar_label_dict = {key: lidar_label_dict[key] for key in sorted(lidar_label_dict.keys())}
            lidar_label_stamp_list: List = list(lidar_label_dict.keys())

            frame: FrameData
            for frame in self.trail_data.lidar_frame_dict.values():
                frame_stamp_ms = frame.stamp_ms
                loc = bisect.bisect_left(lidar_label_stamp_list, frame_stamp_ms)
                if loc == 0 or loc == len(lidar_label_stamp_list):
                    frame.flag.append("no_bev_label")
                    continue
                frame.lidar_label_path = lidar_label_dict[lidar_label_stamp_list[loc]]

    def add_bev_label_to_trail_data(self):
        cmd = "python3 {} --bev_label_folder={} --bev_label_vis_folder={} --bev_label_txt_folder={}".format(
            os.path.join(lidar_mapping_root_folder, "preprocess/script/bev_label_vis.py"),
            os.path.join(self.data_folder, "bev_label"),
            os.path.join(self.data_folder, "bev_label_vis"),
            os.path.join(self.data_folder, "bev_label_txt"))
        util.run_a_process(cmd)

        bev_label_dict: Dict[int, str] = {}
        all_bev_label_path = sorted(util.get_files_path_with_suffix(os.path.join(self.data_folder, "bev_label"), '.json'))
        if all_bev_label_path == []:
            return

        for bev_label_path in all_bev_label_path:
            stamp_us = int(util.get_file_name_in_file_path(bev_label_path).split(".")[0])
            stamp_ms = int(round(stamp_us/1000))
            bev_label_dict[stamp_ms] = bev_label_path
        bev_label_dict = {key: bev_label_dict[key] for key in sorted(bev_label_dict.keys())}
        bev_label_stamp_list: List = list(bev_label_dict.keys())
        frame: FrameData
        for frame in self.trail_data.frame_dict.values():
            frame_stamp_ms = frame.stamp_ms
            loc = bisect.bisect_left(bev_label_stamp_list, frame_stamp_ms)
            if loc == 0 or loc == len(bev_label_stamp_list):
                frame.flag.append("no_bev_label")
                continue
            frame.bev_label_path = bev_label_dict[bev_label_stamp_list[loc]]

    def stage_1(self, data_folder: str, utm_num: int = 0, t_utm_world: list = [], data_type: str = DataType.BYD_BEV.name, sparse_data: bool = True):
        self.data_folder = data_folder
        self.trail_data.utm_num = self.utm_num = utm_num
        self.trail_data.t_utm_world = self.t_utm_world = t_utm_world
        self.trail_data.data_type = data_type

        self.read_trail_info_and_densify()
        # TODO:qzc
        # self.read_calib_to_trail_data()

        # self.add_scan_to_trail_data()
        # self.add_image_to_trail_data()
        # self.add_perception_raw_to_trail_data()
        ## self.add_bev_label_to_trail_data()
        self.add_lidar_label_to_trail_data_qzc()
        self.add_bev_label_to_trail_data_qzc()

        # self.add_cloud_line_seg_to_trail_data()
        # self.add_cloud_pano_seg_to_trail_data()

        # self.read_nav_ins_to_trail_data()
        self.read_nav_ins_to_trail_data_qzc()
        self.read_ndm_loc_to_trail_data()
        self.read_odom_to_trail_data()
        self.read_odom3d_to_trail_data()

        # TODO:qzc use utm instead of opt_q_world_veh 
        # self.read_opt_pose_to_trail_data_qzc()

        # self.read_pp_laneline_file()  # 读取车端感知后处理车道线结果
        # self.add_pp_laneline_to_trail_data()  # 将感知结果与trail绑定

        # self.read_imu_to_csv()
        # self.read_veh_status_to_csv()

        self.trail_data.output_veh_pose("odom", os.path.join(self.data_folder, "debug/veh_pose_raw_odom.txt"))
        self.trail_data.output_veh_pose("ins", os.path.join(self.data_folder, "debug/veh_pose_raw_ins.txt"))
        self.trail_data.output_veh_pose("loc", os.path.join(self.data_folder, "debug/veh_pose_raw_loc.txt"))

        log.info("[DataSetMaker] 原始帧数:", len(self.trail_data.frame_dict.values()))
        self.remove_incomplete_frame()
        log.info("[DataSetMaker] 剔除不完整帧后帧数:", len(self.trail_data.frame_dict.values()))
        # self.remove_abnormal_frame()
        # log.info("[DataSetMaker] 剔除异常帧后帧数:", len(self.trail_data.frame_dict.values()))
        self.remove_out_data_scope_frame_qzc()
        self.remove_return_frame()

        if sparse_data:
            self.remove_out_data_scope_frame()
            log.info("[DataSetMaker] 剔除任务范围外帧后帧数:", len(self.trail_data.frame_dict.values()))
            self.remove_discontinuous_part()  # 只保留最长连续片段
            log.info("[DataSetMaker] 剔除不连续片段后帧数:", len(self.trail_data.frame_dict.values()))
            self.bing_frame_to_link()

        self.trail_data.to_json(os.path.join(self.data_folder, "raw_rate_data_set.json"))
        if sparse_data:
            self.remove_near_frame()
            log.info("[DataSetMaker] 剔除过近帧后帧数:", len(self.trail_data.frame_dict.values()))
        else:
            log.warning("不进行抽帧")

        # self.trail_data.to_pp_laneline_info_json(os.path.join(self.data_folder, "pp_laneline"))  # 将感知结果以json文件格式输出
        self.trail_data.to_json(os.path.join(self.data_folder, "stage_1_data_set.json"))

    def stage_2(self, data_folder: str, utm_num: int = 0, t_utm_world: list = []):
        # TODO: qzc
        self.data_folder = data_folder
        self.trail_data.utm_num = self.utm_num = utm_num
        self.trail_data.t_utm_world = self.t_utm_world = t_utm_world
        self.trail_data.from_json(os.path.join(self.data_folder, "stage_1_data_set.json"))

        self.trail_data.to_json(os.path.join(self.data_folder, "data_set.json"))  # 替换原来的json
        self.trail_data.output_veh_pose("odom", os.path.join(self.data_folder, "debug/veh_pose_odom.txt"))
        self.trail_data.output_veh_pose("ins", os.path.join(self.data_folder, "debug/veh_pose_ins.txt"))
        self.trail_data.output_veh_pose("loc", os.path.join(self.data_folder, "debug/veh_pose_loc.txt"))
        return 
    
        self.data_folder = data_folder
        self.trail_data.utm_num = self.utm_num = utm_num
        self.trail_data.t_utm_world = self.t_utm_world = t_utm_world
        self.trail_data.from_json(os.path.join(self.data_folder, "stage_1_data_set.json"))

        preprocess_cmd = "{} --data_set_path={} --output_folder={} ".format(
            os.path.join(lidar_mapping_root_folder, "preprocess/pointcloudrender/bin/scan_render"),
            os.path.join(self.data_folder, "stage_1_data_set.json"),
            self.data_folder)
        run_a_process(preprocess_cmd, 3600)
        time.sleep(3)

        un_image_param = util.json_file_to_dict(os.path.join(self.data_folder, "un_image_param.json"))
        if (un_image_param == {}):
            log.warning("scan_render 似乎未调用成功，第一次重试...")
            run_a_process(preprocess_cmd, 3600)
            time.sleep(3)
            un_image_param = util.json_file_to_dict(os.path.join(self.data_folder, "un_image_param.json"))
            if (un_image_param == {}):
                log.warning("scan_render 似乎未调用成功，第二次重试...")
                run_a_process(preprocess_cmd, 3600)
                time.sleep(3)
                un_image_param = util.json_file_to_dict(os.path.join(self.data_folder, "un_image_param.json"))
                if (un_image_param == {}):
                    log.warning("scan_render 似乎未调用成功，放弃重试,打印 preprocess 文件夹下所有文件：")
                    all_file = util.get_files_path_recursion(os.path.join(lidar_mapping_root_folder, "preprocess"))
                    log.warning(all_file)

        # self.get_render_result()
        self.get_render_result_qzc()

        self.trail_data.to_json(os.path.join(self.data_folder, "data_set.json"))  # 替换原来的json
        self.trail_data.output_veh_pose("odom", os.path.join(self.data_folder, "debug/veh_pose_odom.txt"))
        self.trail_data.output_veh_pose("ins", os.path.join(self.data_folder, "debug/veh_pose_ins.txt"))
        self.trail_data.output_veh_pose("loc", os.path.join(self.data_folder, "debug/veh_pose_loc.txt"))

    def stage_all(self, data_folder: str, utm_num: int = 0, t_utm_world: list = [], data_type: str = DataType.BYD_BEV.name, sparse_data: bool = True):
        self.stage_1(data_folder, utm_num, t_utm_world, data_type, sparse_data)
        self.stage_2(data_folder, utm_num, t_utm_world)


parser = argparse.ArgumentParser()
parser.add_argument('--data_folder', type=str, default="/home/test/lidar_mapping_ws/1234567890/data/1659476486")
parser.add_argument('--utm_num', type=str, default='50')
parser.add_argument('--t_utm_world', type=str, default='[473194,4446707,34]')
parser.add_argument('--data_type', type=str, default=DataType.BYD_BEV.name)
parser.add_argument('--sparse_data', type=lambda x: x.lower() == 'true', default=False)

parser.add_argument('--stage', type=str, default='')  # 不填则执行全部阶段

if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.data_folder, f"log/data_set_maker_{util.get_folder_name_in_path(args.data_folder)}.log"))
    log.warning("[DataSerMaker] data_folder: ", args.data_folder)
    log.warning("[DataSerMaker] utm_num: ", args.utm_num)
    log.warning("[DataSerMaker] t_utm_world: ", args.t_utm_world)
    log.warning("[DataSerMaker] data_type: ", args.data_type)
    log.warning("[DataSerMaker] sparse_data: ", args.sparse_data)
    log.warning("[DataSerMaker] stage: ", args.stage)

    data_set_maker: DataSetMaker = DataSetMaker()
    if args.stage == "1":  # 阶段1，基础数据处理
        data_set_maker.stage_1(data_folder=args.data_folder,
                               utm_num=int(args.utm_num),
                               t_utm_world=json.loads(args.t_utm_world),
                               data_type=args.data_type,
                               sparse_data=args.sparse_data)
    elif args.stage == "2":  # 阶段2，点云数据处理
        data_set_maker.stage_2(data_folder=args.data_folder,
                               utm_num=int(args.utm_num),
                               t_utm_world=json.loads(args.t_utm_world))
    else:
        data_set_maker.stage_all(data_folder=args.data_folder,
                                 utm_num=int(args.utm_num),
                                 t_utm_world=json.loads(args.t_utm_world),
                                 data_type=args.data_type,
                                 sparse_data=args.sparse_data)
