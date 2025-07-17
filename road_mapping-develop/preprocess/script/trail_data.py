
import numpy as np
import os
import json
from scipy.spatial.transform import Rotation
from typing import Dict


def Tab_to_Tba(Tab: np.ndarray) -> np.ndarray:
    Rba = Tab[:3, :3].T
    tba = -Rba.dot(Tab[:3, 3])
    return np.vstack([np.hstack([Rba, tba.reshape(-1, 1)]), [0, 0, 0, 1]])


def write_lines_to_file_override(file_path: str, lines: list):
    folder = os.path.dirname(file_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    s = ''
    for l in lines:
        s += (str(l).strip('\n') + '\n')
    open(file_path, 'w').write(s.strip('\n'))


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


def qab_tab_to_Tab(qab: np.ndarray, tab: np.ndarray) -> np.ndarray:
    if (len(qab) == 0) or (len(tab) == 0):
        # print("qab_tab_to_Tab 输入长度为0")
        return np.array([])
    Rab: np.ndarray = Rotation.from_quat(qab).as_matrix()
    Tab: np.ndarray = np.c_[Rab, tab]
    Tab: np.ndarray = np.r_[Tab, np.array([[0, 0, 0, 1]])]
    return Tab


def tab_to_Tab(tab: np.ndarray) -> np.ndarray:
    Tab: np.ndarray = np.eye(4)
    Tab[0, 3] = tab[0]
    Tab[1, 3] = tab[1]
    Tab[2, 3] = tab[2]
    return Tab


def Tab_to_qab(Tab: np.ndarray) -> np.ndarray:
    return np.array(Rotation.from_matrix(Tab[0:3, 0:3]).as_quat())


def Tab_to_tab(Tab: np.ndarray) -> np.ndarray:
    return Tab[0:3, 3:4].reshape(3)


def Tab_pbc_to_pac(Tab: np.ndarray, pbc: np.ndarray) -> np.ndarray:
    Rab: np.ndarray = Tab[0:3, 0:3]
    tab: np.ndarray = Tab[0:3, 3:4].reshape(3)
    return Rab.dot(pbc) + tab


def fxycxy_to_K(fxycxy):
    if len(fxycxy) == 0:
        return np.array([])
    K = np.array([[fxycxy[0], 0, fxycxy[2]],
                  [0, fxycxy[1], fxycxy[3]],
                  [0, 0, 1]])
    return K


class FrameData:
    def __init__(self) -> None:
        self.trail_id: int = 0
        self.link_id_list: list = []
        self.stamp_ms: int = 0
        self.scan_path: str = ""
        self.scan_rgb_path: str = ""
        self.veh_scan_path: str = ""
        self.front_image_path: str = ""
        self.cloud_line_seg_path: str = ""
        self.cloud_pano_seg_path: str = ""
        self.bev_label_path: str = ""
        self.lidar_label_path: str = ""
        self.un_image_path: str = ""
        self.un_image_seg_path: str = ""
        self.key_frame_id: int = 0
        self.perception_data_path: str = ""

        self.t_veh_lidar: np.ndarray = np.array([])
        self.q_veh_lidar: np.ndarray = np.array([])
        self.T_veh_lidar: np.ndarray = np.array([])

        self.t_veh_cam: np.ndarray = np.array([])
        self.q_veh_cam: np.ndarray = np.array([])
        self.T_veh_cam: np.ndarray = np.array([])
        self.cam_width: int = 0
        self.cam_height: int = 0
        self.cam_fxycxy: list = []
        self.cam_K: np.ndarray = np.array([])
        self.cam_k1234: list = []

        self.un_cam_width: int = 0
        self.un_cam_height: int = 0
        self.un_cam_fxycxy: list = []
        self.un_cam_K: np.ndarray = np.array([])

        self.ins_q_world_veh: np.ndarray = np.array([])
        self.ins_t_world_veh: np.ndarray = np.array([])
        self.ins_T_world_veh: np.ndarray = np.array([])
        self.ins_position_type: str = ""

        self.lio_q_world_veh: np.ndarray = np.array([])
        self.lio_t_world_veh: np.ndarray = np.array([])
        self.lio_T_world_veh: np.ndarray = np.array([])

        self.reloc_q_world_veh: np.ndarray = np.array([])
        self.reloc_t_world_veh: np.ndarray = np.array([])
        self.reloc_T_world_veh: np.ndarray = np.array([])

        self.loc_q_world_veh: np.ndarray = np.array([])
        self.loc_t_world_veh: np.ndarray = np.array([])
        self.loc_T_world_veh: np.ndarray = np.array([])

        self.odom_q_odom_veh: np.ndarray = np.array([])
        self.odom_t_odom_veh: np.ndarray = np.array([])
        self.odom_T_odom_veh: np.ndarray = np.array([])

        self.odom3d_q_odom_veh: np.ndarray = np.array([])
        self.odom3d_t_odom_veh: np.ndarray = np.array([])
        self.odom3d_T_odom_veh: np.ndarray = np.array([])

        # 图优化多躺融合后的车体位姿
        self.opt_q_world_veh: np.ndarray = np.array([])
        self.opt_t_world_veh: np.ndarray = np.array([])
        self.opt_T_world_veh: np.ndarray = np.array([])
        self.opt_cnt = 0

        self.good_bev_line_id: list = []
        self.flag: list = []
        self.submap_id_list = []

        # 感知后处理结果
        self.pp_laneline_stamp = 0
        self.pp_laneline_points = []
        self.pp_laneline_coeffs = []
        self.pp_laneline_types = []
        self.pp_laneline_start_end_pts = []


class TrailData:
    @classmethod
    def init_from_json(cls, json_path: str):
        d = cls()
        d.from_json(json_path)
        return d

    def __init__(self) -> None:
        self.trail_id: str = ""  # trail id 保持为字符串格式
        self.tile_id: int = 0  # tile id 保持为 int 格式
        self.link_id_list: list = []  # link id 保持为 int 格式

        self.data_type: str = "BYD_BEV"

        self.utm_num: int = 0
        self.t_utm_world: np.ndarray = np.array([])

        self.t_veh_lidar: np.ndarray = np.array([])
        self.q_veh_lidar: np.ndarray = np.array([])
        self.T_veh_lidar: np.ndarray = np.array([])

        self.t_veh_cam: np.ndarray = np.array([])
        self.q_veh_cam: np.ndarray = np.array([])
        self.T_veh_cam: np.ndarray = np.array([])
        self.cam_width: int = 0
        self.cam_height: int = 0
        self.cam_fxycxy: list = []
        self.cam_K: np.ndarray = np.array([])
        self.cam_k1234: list = []
        self.cam_k12p12k3456: np.ndarray = np.array([])
        self.cam_dist_type: str = ""

        self.un_cam_width: int = 0
        self.un_cam_height: int = 0
        self.un_cam_fxycxy: list = []
        self.un_cam_K: np.ndarray = np.array([])

        self.frame_dict: Dict[int, FrameData] = {}
        self.lidar_frame_dict: Dict[int, FrameData] = {}
        self.tag = []
        self.reloc_state: int = 0
        self.comment = ""

    def calc_opt_ins_diff(self):
        diff_sum = 0
        diff_cnt = 0
        frame: FrameData
        for frame in self.frame_dict.values():
            if (len(frame.ins_t_world_veh) == 3 and len(frame.opt_t_world_veh) == 3):
                T_ins_opt = Tab_to_Tba(frame.ins_T_world_veh).dot(frame.opt_T_world_veh)
                diff_sum += np.linalg.norm(np.abs(T_ins_opt[:3, 3]))
                diff_cnt += 1
        return diff_sum, diff_cnt

    def calc_opt_ins_diff_xyz_sum_cnt(self):
        diff_xyz_sum = 0
        diff_cnt = 0
        frame: FrameData
        for frame in self.frame_dict.values():
            if (len(frame.ins_t_world_veh) == 3 and len(frame.opt_t_world_veh) == 3):
                T_ins_opt = Tab_to_Tba(frame.ins_T_world_veh).dot(frame.opt_T_world_veh)
                diff_xyz_sum += np.abs(T_ins_opt[:3, 3])
                diff_cnt += 1
        if diff_cnt == 0:
            return 0, 0
        else:
            return diff_xyz_sum, diff_cnt

    def calc_opt_ins_diff_xyz_mean(self):
        diff_xyz_sum = np.array([0., 0., 0.])
        diff_cnt = 0
        frame: FrameData
        for frame in self.frame_dict.values():
            if (len(frame.ins_t_world_veh) == 3 and len(frame.opt_t_world_veh) == 3):
                T_ins_opt = Tab_to_Tba(frame.ins_T_world_veh).dot(frame.opt_T_world_veh)
                diff_xyz_sum += np.abs(T_ins_opt[:3, 3])
                diff_cnt += 1
        if diff_cnt == 0:
            return np.array([100000, 100000, 100000])
        else:
            return diff_xyz_sum/diff_cnt

    def output_ins_lidar_pose_one_line_file(self, output_folder):
        frame: FrameData
        frame_id = 0
        for frame in self.frame_dict.values():
            ins_T_world_veh = qab_tab_to_Tab(frame.ins_q_world_veh, frame.ins_t_world_veh)
            ins_T_world_lidar = ins_T_world_veh.dot(self.T_veh_lidar)
            ins_t_world_lidar = Tab_to_tab(ins_T_world_lidar)
            ins_t_utm_lidar = ins_t_world_lidar+self.t_utm_world
            ins_q_world_lidar = Tab_to_qab(ins_T_world_lidar)
            pose_file_line = [f"{frame_id} {frame.stamp_ms/1000} {ins_t_utm_lidar[0]} {ins_t_utm_lidar[1]} {ins_t_utm_lidar[2]} {ins_q_world_lidar[0]} {ins_q_world_lidar[1]} {ins_q_world_lidar[2]} {ins_q_world_lidar[3]}"]
            frame_id += 1
            file_path = os.path.join(output_folder, f"{frame.stamp_ms/1000}.pose")
            write_lines_to_file_override(file_path, pose_file_line)

    def output_cc_veh_pose(self, type, to_path):
        out_list = ["x y z r g b"]
        frame: FrameData
        for frame in self.frame_dict.values():
            veh_c = []
            T_world_veh = []
            if type == "loc":
                veh_c = frame.loc_t_world_veh
                if len(veh_c) < 3:
                    continue
                T_world_veh = qab_tab_to_Tab(frame.loc_q_world_veh, frame.loc_t_world_veh)
            elif type == "ins":
                veh_c = frame.ins_t_world_veh
                if len(veh_c) < 3:
                    continue
                T_world_veh = qab_tab_to_Tab(frame.ins_q_world_veh, frame.ins_t_world_veh)
            elif type == "odom":
                veh_c = frame.odom_t_odom_veh
                if len(veh_c) < 3:
                    continue
                T_world_veh = qab_tab_to_Tab(frame.odom_q_odom_veh, frame.odom_t_odom_veh)
            elif type == "opt":
                veh_c = frame.opt_t_world_veh
                if len(veh_c) < 3:
                    continue
                T_world_veh = qab_tab_to_Tab(frame.opt_q_world_veh, frame.opt_t_world_veh)
            elif type == "lio":
                veh_c = frame.lio_t_world_veh
                if len(veh_c) < 3:
                    continue
                T_world_veh = qab_tab_to_Tab(frame.lio_q_world_veh, frame.lio_t_world_veh)
            elif type == "reloc":
                veh_c = frame.reloc_t_world_veh
                if len(veh_c) < 3:
                    continue
                T_world_veh = qab_tab_to_Tab(frame.reloc_q_world_veh, frame.reloc_t_world_veh)

            if len(veh_c) < 3:
                continue
            out_list.append(f"{veh_c[0]} {veh_c[1]} {veh_c[2]} 255 255 255")
            for i in range(10):
                veh_x = Tab_pbc_to_pac(T_world_veh, np.array([i*0.1, 0, 0]))
                out_list.append(f"{veh_x[0]} {veh_x[1]} {veh_x[2]} 255 0 0")
                veh_y = Tab_pbc_to_pac(T_world_veh, np.array([0, i*0.1, 0]))
                out_list.append(f"{veh_y[0]} {veh_y[1]} {veh_y[2]} 0 255 0")
                veh_z = Tab_pbc_to_pac(T_world_veh, np.array([0, 0, i*0.1]))
                out_list.append(f"{veh_z[0]} {veh_z[1]} {veh_z[2]} 0 0 255")
        write_lines_to_file_override(to_path, out_list)

    def output_veh_pose(self, type, to_path):
        out_list = ["stamp_ms x y z qx qy qz qw"]
        frame: FrameData
        for frame in self.frame_dict.values():
            veh_c = []
            if type == "loc":
                veh_c = frame.loc_t_world_veh
                veh_q = frame.loc_q_world_veh
            elif type == "ins":
                veh_c = frame.ins_t_world_veh
                veh_q = frame.ins_q_world_veh
            elif type == "odom":
                veh_c = frame.odom_t_odom_veh
                veh_q = frame.odom_q_odom_veh
            elif type == "opt":
                veh_c = frame.opt_t_world_veh
                veh_q = frame.opt_q_world_veh
            elif type == "lio":
                veh_c = frame.lio_t_world_veh
                veh_q = frame.lio_q_world_veh
            elif type == "reloc":
                veh_c = frame.reloc_t_world_veh
                veh_q = frame.reloc_q_world_veh
            if len(veh_c) < 3:
                continue
            out_list.append(f"{frame.stamp_ms} {veh_c[0]} {veh_c[1]} {veh_c[2]} {veh_q[0]} {veh_q[1]} {veh_q[2]} {veh_q[3]}")
        write_lines_to_file_override(to_path, out_list)

    def to_json(self, to_json_path: str):
        trail_json = {}
        trail_json["header"] = {
            "trail_id": self.trail_id,
            "utm_num": self.utm_num,
            "t_utm_world": self.t_utm_world,
            "tile_id": self.tile_id,
            "link_id_list": self.link_id_list,
            "data_type": self.data_type,
            "q_veh_lidar": self.q_veh_lidar,
            "t_veh_lidar": self.t_veh_lidar,
            "q_veh_cam": self.q_veh_cam,
            "t_veh_cam": self.t_veh_cam,
            "cam_width": self.cam_width,
            "cam_height": self.cam_height,
            "cam_fxycxy": self.cam_fxycxy,
            "cam_k1234": self.cam_k1234,
            "cam_k12p12k3456": self.cam_k12p12k3456,
            "cam_dist_type": self.cam_dist_type,
            "un_cam_width": self.un_cam_width,
            "un_cam_height": self.un_cam_height,
            "un_cam_fxycxy": self.un_cam_fxycxy,
            "reloc_state": self.reloc_state,
            "tag": self.tag,
            "comment": self.comment
        }
        trail_json["lidar_data"] = []
        frame: FrameData
        for frame in self.lidar_frame_dict.values():
            frame_json = {}
            frame_json["stamp_ms"] = frame.stamp_ms
            frame_json["link_id_list"] = frame.link_id_list
            frame_json["scan_path"] = frame.scan_path
            frame_json["scan_rgb_path"] = frame.scan_rgb_path
            frame_json["veh_scan_path"] = frame.veh_scan_path
            frame_json["front_image_path"] = frame.front_image_path
            frame_json["perception_data_path"] = frame.perception_data_path
            frame_json["cloud_line_seg_path"] = frame.cloud_line_seg_path
            frame_json["cloud_pano_seg_path"] = frame.cloud_pano_seg_path
            frame_json["lidar_label_path"] = frame.lidar_label_path
            frame_json["un_image_path"] = frame.un_image_path
            frame_json["un_image_seg_path"] = frame.un_image_seg_path
            frame_json["key_frame_id"] = frame.key_frame_id
            frame_json["ins_position_type"] = frame.ins_position_type
            frame_json["ins_q_world_veh"] = frame.ins_q_world_veh
            frame_json["ins_t_world_veh"] = frame.ins_t_world_veh
            frame_json["lio_q_world_veh"] = frame.lio_q_world_veh
            frame_json["lio_t_world_veh"] = frame.lio_t_world_veh
            frame_json["reloc_q_world_veh"] = frame.reloc_q_world_veh
            frame_json["reloc_t_world_veh"] = frame.reloc_t_world_veh
            frame_json["loc_q_world_veh"] = frame.loc_q_world_veh
            frame_json["loc_t_world_veh"] = frame.loc_t_world_veh
            frame_json["odom_q_odom_veh"] = frame.odom_q_odom_veh
            frame_json["odom_t_odom_veh"] = frame.odom_t_odom_veh
            frame_json["odom3d_q_odom_veh"] = frame.odom3d_q_odom_veh
            frame_json["odom3d_t_odom_veh"] = frame.odom3d_t_odom_veh
            frame_json["opt_q_world_veh"] = frame.opt_q_world_veh
            frame_json["opt_t_world_veh"] = frame.opt_t_world_veh
            frame_json["flag"] = frame.flag
            frame_json["good_bev_line_id"] = frame.good_bev_line_id
            frame_json["submap_id_list"] = frame.submap_id_list
            trail_json["lidar_data"].append(frame_json)
        trail_json["data"] = []
        for frame in self.frame_dict.values():
            frame_json = {}
            frame_json["stamp_ms"] = frame.stamp_ms
            frame_json["link_id_list"] = frame.link_id_list
            frame_json["scan_path"] = frame.scan_path
            frame_json["scan_rgb_path"] = frame.scan_rgb_path
            frame_json["veh_scan_path"] = frame.veh_scan_path
            frame_json["front_image_path"] = frame.front_image_path
            frame_json["perception_data_path"] = frame.perception_data_path
            frame_json["cloud_line_seg_path"] = frame.cloud_line_seg_path
            frame_json["cloud_pano_seg_path"] = frame.cloud_pano_seg_path
            frame_json["bev_label_path"] = frame.bev_label_path
            frame_json["un_image_path"] = frame.un_image_path
            frame_json["un_image_seg_path"] = frame.un_image_seg_path
            frame_json["key_frame_id"] = frame.key_frame_id
            frame_json["ins_position_type"] = frame.ins_position_type
            frame_json["ins_q_world_veh"] = frame.ins_q_world_veh
            frame_json["ins_t_world_veh"] = frame.ins_t_world_veh
            frame_json["lio_q_world_veh"] = frame.lio_q_world_veh
            frame_json["lio_t_world_veh"] = frame.lio_t_world_veh
            frame_json["reloc_q_world_veh"] = frame.reloc_q_world_veh
            frame_json["reloc_t_world_veh"] = frame.reloc_t_world_veh
            frame_json["loc_q_world_veh"] = frame.loc_q_world_veh
            frame_json["loc_t_world_veh"] = frame.loc_t_world_veh
            frame_json["odom_q_odom_veh"] = frame.odom_q_odom_veh
            frame_json["odom_t_odom_veh"] = frame.odom_t_odom_veh
            frame_json["odom3d_q_odom_veh"] = frame.odom3d_q_odom_veh
            frame_json["odom3d_t_odom_veh"] = frame.odom3d_t_odom_veh
            frame_json["opt_q_world_veh"] = frame.opt_q_world_veh
            frame_json["opt_t_world_veh"] = frame.opt_t_world_veh
            frame_json["flag"] = frame.flag
            frame_json["good_bev_line_id"] = frame.good_bev_line_id
            frame_json["submap_id_list"] = frame.submap_id_list
            trail_json["data"].append(frame_json)

        dict_to_json_file(to_json_path, trail_json)
        print("完成输出 dataset.json to ", to_json_path)

    def from_json(self, from_json_path: str):
        trail_json: dict = json_file_to_dict(from_json_path)
        if trail_json == {}:
            print("[SubmapMapping] data set json 为空:", from_json_path)
            return
        self.trail_id = trail_json["header"]["trail_id"]
        self.utm_num = trail_json["header"]["utm_num"]
        self.t_utm_world = np.array(trail_json["header"]["t_utm_world"])
        self.q_veh_lidar = np.array(trail_json["header"]["q_veh_lidar"])
        self.t_veh_lidar = np.array(trail_json["header"]["t_veh_lidar"])
        self.T_veh_lidar = qab_tab_to_Tab(self.q_veh_lidar, self.t_veh_lidar)
        self.q_veh_cam = trail_json["header"]["q_veh_cam"]
        self.t_veh_cam = trail_json["header"]["t_veh_cam"]
        self.T_veh_cam = qab_tab_to_Tab(self.q_veh_cam, self.t_veh_cam)
        self.cam_width = trail_json["header"]["cam_width"]
        self.cam_height = trail_json["header"]["cam_height"]
        self.cam_fxycxy = trail_json["header"]["cam_fxycxy"]
        self.cam_k1234 = trail_json["header"]["cam_k1234"]
        self.cam_k12p12k3456 = trail_json["header"]["cam_k12p12k3456"]
        self.cam_dist_type = trail_json["header"]["cam_dist_type"]
        self.cam_K = fxycxy_to_K(self.cam_fxycxy)
        self.tag = trail_json["header"]["tag"]
        self.reloc_state = trail_json["header"]["reloc_state"]
        self.tile_id = trail_json["header"]["tile_id"]
        self.link_id_list = trail_json["header"]["link_id_list"]
        self.data_type = trail_json["header"]["data_type"]
        self.un_cam_width = trail_json["header"]["un_cam_width"]
        self.un_cam_height = trail_json["header"]["un_cam_height"]
        self.un_cam_fxycxy = trail_json["header"]["un_cam_fxycxy"]
        self.un_cam_K = fxycxy_to_K(self.un_cam_fxycxy)

        for frame_json in trail_json["lidar_data"]:
            frame: FrameData = FrameData()
            frame.stamp_ms = frame_json["stamp_ms"]
            frame.trail_id = self.trail_id
            frame.scan_path = frame_json["scan_path"]
            frame.scan_rgb_path = frame_json["scan_rgb_path"]
            frame.veh_scan_path = frame_json["veh_scan_path"]
            frame.front_image_path = frame_json["front_image_path"]
            frame.cloud_line_seg_path = frame_json["cloud_line_seg_path"]
            frame.key_frame_id = frame_json["key_frame_id"]
            frame.perception_data_path = frame_json["perception_data_path"]
            frame.lidar_label_path = frame_json["lidar_label_path"]
            frame.cloud_pano_seg_path = frame_json["cloud_pano_seg_path"]
            frame.link_id_list = frame_json["link_id_list"]
            frame.un_image_path = frame_json["un_image_path"]
            frame.un_image_seg_path = frame_json["un_image_seg_path"]
            frame.ins_position_type = frame_json["ins_position_type"]
            frame.q_veh_lidar = self.q_veh_lidar
            frame.t_veh_lidar = self.t_veh_lidar
            frame.T_veh_lidar = self.T_veh_lidar
            frame.q_veh_cam = self.q_veh_cam
            frame.t_veh_cam = self.t_veh_cam
            frame.T_veh_cam = self.T_veh_cam
            frame.cam_width = self.cam_width
            frame.cam_height = self.cam_height
            frame.cam_fxycxy = self.cam_fxycxy
            frame.cam_k1234 = self.cam_k1234
            frame.cam_K = self.cam_K

            frame.un_cam_width: int = self.un_cam_width
            frame.un_cam_height: int = self.un_cam_height
            frame.un_cam_fxycxy: list = self.un_cam_fxycxy
            frame.un_cam_K: np.ndarray = self.un_cam_K

            frame.ins_q_world_veh = np.array(frame_json["ins_q_world_veh"])
            frame.ins_t_world_veh = np.array(frame_json["ins_t_world_veh"])
            frame.ins_T_world_veh = qab_tab_to_Tab(frame.ins_q_world_veh, frame.ins_t_world_veh)
            if (len(np.array(frame_json["lio_q_world_veh"])) == 4):
                frame.lio_q_world_veh = np.array(frame_json["lio_q_world_veh"])
                frame.lio_t_world_veh = np.array(frame_json["lio_t_world_veh"])
                frame.lio_T_world_veh = qab_tab_to_Tab(frame.lio_q_world_veh, frame.lio_t_world_veh)
            if (len(np.array(frame_json["reloc_q_world_veh"])) == 4):
                frame.reloc_q_world_veh = np.array(frame_json["reloc_q_world_veh"])
                frame.reloc_t_world_veh = np.array(frame_json["reloc_t_world_veh"])
                frame.reloc_T_world_veh = qab_tab_to_Tab(frame.reloc_q_world_veh, frame.reloc_t_world_veh)
            if (len(np.array(frame_json["loc_q_world_veh"])) == 4):
                frame.loc_q_world_veh = np.array(frame_json["loc_q_world_veh"])
                frame.loc_t_world_veh = np.array(frame_json["loc_t_world_veh"])
                frame.loc_T_world_veh = qab_tab_to_Tab(frame.loc_q_world_veh, frame.loc_t_world_veh)
            frame.odom_q_odom_veh = np.array(frame_json["odom_q_odom_veh"])
            frame.odom_t_odom_veh = np.array(frame_json["odom_t_odom_veh"])
            frame.odom_T_odom_veh = qab_tab_to_Tab(frame.odom_q_odom_veh, frame.odom_t_odom_veh)
            if (len(np.array(frame_json["odom3d_q_odom_veh"])) == 4):
                frame.odom3d_q_odom_veh = np.array(frame_json["odom3d_q_odom_veh"])
                frame.odom3d_t_odom_veh = np.array(frame_json["odom3d_t_odom_veh"])
                frame.odom3d_T_odom_veh = qab_tab_to_Tab(frame.odom3d_q_odom_veh, frame.odom3d_t_odom_veh)
            if (len(np.array(frame_json["opt_q_world_veh"])) == 4):
                frame.opt_q_world_veh = np.array(frame_json["opt_q_world_veh"])
                frame.opt_t_world_veh = np.array(frame_json["opt_t_world_veh"])
                frame.opt_T_world_veh = qab_tab_to_Tab(frame.opt_q_world_veh, frame.opt_t_world_veh)
            frame.flag = frame_json["flag"]
            frame.good_bev_line_id = frame_json["good_bev_line_id"] if "good_bev_line_id" in frame_json.keys() else []
            # frame.submap_id_list = frame_json["submap_id_list"] if "submap_id_list" in frame_json.keys() else []
            self.lidar_frame_dict[frame.stamp_ms] = frame

        for frame_json in trail_json["data"]:
            frame: FrameData = FrameData()
            frame.stamp_ms = frame_json["stamp_ms"]
            frame.trail_id = self.trail_id
            frame.scan_path = frame_json["scan_path"]
            frame.scan_rgb_path = frame_json["scan_rgb_path"]
            frame.veh_scan_path = frame_json["veh_scan_path"]
            frame.front_image_path = frame_json["front_image_path"]
            frame.cloud_line_seg_path = frame_json["cloud_line_seg_path"]
            frame.key_frame_id = frame_json["key_frame_id"]
            frame.perception_data_path = frame_json["perception_data_path"]
            frame.bev_label_path = frame_json["bev_label_path"]
            frame.cloud_pano_seg_path = frame_json["cloud_pano_seg_path"]
            frame.link_id_list = frame_json["link_id_list"]
            frame.un_image_path = frame_json["un_image_path"]
            frame.un_image_seg_path = frame_json["un_image_seg_path"]
            frame.ins_position_type = frame_json["ins_position_type"]
            frame.q_veh_lidar = self.q_veh_lidar
            frame.t_veh_lidar = self.t_veh_lidar
            frame.T_veh_lidar = self.T_veh_lidar
            frame.q_veh_cam = self.q_veh_cam
            frame.t_veh_cam = self.t_veh_cam
            frame.T_veh_cam = self.T_veh_cam
            frame.cam_width = self.cam_width
            frame.cam_height = self.cam_height
            frame.cam_fxycxy = self.cam_fxycxy
            frame.cam_k1234 = self.cam_k1234
            frame.cam_K = self.cam_K

            frame.un_cam_width: int = self.un_cam_width
            frame.un_cam_height: int = self.un_cam_height
            frame.un_cam_fxycxy: list = self.un_cam_fxycxy
            frame.un_cam_K: np.ndarray = self.un_cam_K

            frame.ins_q_world_veh = np.array(frame_json["ins_q_world_veh"])
            frame.ins_t_world_veh = np.array(frame_json["ins_t_world_veh"])
            frame.ins_T_world_veh = qab_tab_to_Tab(frame.ins_q_world_veh, frame.ins_t_world_veh)
            if (len(np.array(frame_json["lio_q_world_veh"])) == 4):
                frame.lio_q_world_veh = np.array(frame_json["lio_q_world_veh"])
                frame.lio_t_world_veh = np.array(frame_json["lio_t_world_veh"])
                frame.lio_T_world_veh = qab_tab_to_Tab(frame.lio_q_world_veh, frame.lio_t_world_veh)
            if (len(np.array(frame_json["reloc_q_world_veh"])) == 4):
                frame.reloc_q_world_veh = np.array(frame_json["reloc_q_world_veh"])
                frame.reloc_t_world_veh = np.array(frame_json["reloc_t_world_veh"])
                frame.reloc_T_world_veh = qab_tab_to_Tab(frame.reloc_q_world_veh, frame.reloc_t_world_veh)
            if (len(np.array(frame_json["loc_q_world_veh"])) == 4):
                frame.loc_q_world_veh = np.array(frame_json["loc_q_world_veh"])
                frame.loc_t_world_veh = np.array(frame_json["loc_t_world_veh"])
                frame.loc_T_world_veh = qab_tab_to_Tab(frame.loc_q_world_veh, frame.loc_t_world_veh)
            frame.odom_q_odom_veh = np.array(frame_json["odom_q_odom_veh"])
            frame.odom_t_odom_veh = np.array(frame_json["odom_t_odom_veh"])
            frame.odom_T_odom_veh = qab_tab_to_Tab(frame.odom_q_odom_veh, frame.odom_t_odom_veh)
            if (len(np.array(frame_json["odom3d_q_odom_veh"])) == 4):
                frame.odom3d_q_odom_veh = np.array(frame_json["odom3d_q_odom_veh"])
                frame.odom3d_t_odom_veh = np.array(frame_json["odom3d_t_odom_veh"])
                frame.odom3d_T_odom_veh = qab_tab_to_Tab(frame.odom3d_q_odom_veh, frame.odom3d_t_odom_veh)
            if (len(np.array(frame_json["opt_q_world_veh"])) == 4):
                frame.opt_q_world_veh = np.array(frame_json["opt_q_world_veh"])
                frame.opt_t_world_veh = np.array(frame_json["opt_t_world_veh"])
                frame.opt_T_world_veh = qab_tab_to_Tab(frame.opt_q_world_veh, frame.opt_t_world_veh)
            frame.flag = frame_json["flag"]
            frame.good_bev_line_id = frame_json["good_bev_line_id"] if "good_bev_line_id" in frame_json.keys() else []
            # frame.submap_id_list = frame_json["submap_id_list"] if "submap_id_list" in frame_json.keys() else []
            self.frame_dict[frame.stamp_ms] = frame

    def to_pp_laneline_info_json(self, to_json_floder: str):
        if not os.path.exists(to_json_floder):
            os.makedirs(to_json_floder)

        frame: FrameData
        for frame in self.frame_dict.values():
            stamp_ms: str = str(int(frame.pp_laneline_stamp)*1000)
            to_json_file = os.path.join(to_json_floder, stamp_ms+".json")
            # log.info("准备输出：", to_json_file)
            frame_json = {}
            frame_json["stamp_ms"] = stamp_ms

            # 将结果分为5大要素-lane fence barrier diversion other
            frame_json["LaneBoundary"] = []
            frame_json["RoadBoundary"] = []
            # print(frame_json["stamp_ms"])
            for i in range(len(frame.pp_laneline_types)):
                lane_dict = {}
                lane_dict["points"] = frame.pp_laneline_points[i]
                # print(lane_dict["points"])
                lane_dict["coeffs"] = frame.pp_laneline_coeffs[i]
                lane_dict["start_end_pts"] = frame.pp_laneline_start_end_pts[i]
                # 属性的写入
                type = int(frame.pp_laneline_types[i])
                lane_dict["pp_type"] = str(format(type, "b"))

                #             message RoadBoundary { //道路边界
                # enum RoadBoundaryType { // 道路边界类型
                #     UNKNOWN_BOUNDARY = 0; //未知类型
                #     //LANELINE = 1;
                #     CURB = 2;//路缘石
                #     //CENTER = 3;
                #     GUARDRAIL = 4;//防护栏
                #     CONCRETE_BARRIER = 5;//混凝土防护栏（新泽西防护栏）
                #     FENCE = 6;//栅栏
                #     WALL = 7;//保护墙
                #     CANOPY = 8;//遮棚
                #     PAVE = 9;//自然边界,铺设未铺设边界
                #     DITCH = 10;//沟渠
                #     PUNCHEON = 11;//离散型障碍物（包括短柱:可表达石墩\短柱等无法穿越的障碍物）
                # }
                # 包装roadboundary
                if (type >> 16 & 1):
                    geo_type = 0
                    if (type >> 19 & 1):
                        geo_type = 5
                    lane_dict["sub_type"] = geo_type
                    frame_json["RoadBoundary"].append(lane_dict)
                else:
                    # color proto
                    # enum Color { // 颜色
                    #     UNKNOWN_COLOR = 0;//未确认
                    #     WHITE = 1;//白色
                    #     YELLOW = 2;//黄色
                    #     ORANGE = 3;//橙色
                    #     BLUE = 4;//蓝色
                    #     GREEN = 5;//绿色
                    #     GRAY = 6;//灰色
                    #     LEFT_GRAY_RIGHT_YELLOW = 7;//左灰右黄
                    #     LEFT_YELLOW_RIGHT_WHITE = 8;//左黄右白
                    #     LEFT_WHITE_RIGHT_YELLOW = 9;//左白右黄
                    # }

                    White: bool = bool(type >> 12 & 1)
                    Yellow: bool = bool(type >> 13 & 1)
                    Blue: bool = bool(type >> 14 & 1)
                    Green: bool = bool(type >> 15 & 1)
                    # print("white:{}-yellow:{}-blue:{}-green:{}".format(White, Yellow, Blue, Green))
                    # print("white:{}-yellow:{}-blue:{}-green:{}-b:type:{}-int:type:{}".format(type >> 12 & 1, type >> 13 & 1, type >> 14 & 1, type >> 15 & 1,lane_dict["pp_type"],int(type)))
                    color_type: int = 0
                    if (White and Yellow):
                        color_type = 8  # 根据远哥需求白和黄同时存在，认定为左黄右白8
                    elif (White and (not Yellow)):
                        color_type = 1
                    elif ((not White) and Yellow):
                        color_type = 2
                    elif (Blue):
                        color_type = 4
                    elif (Green):
                        color_type = 5

                    lane_dict["color_type"] = color_type
                    # LineMarking type proto
                    # enum MarkingType { //车道标线线型
                    #     UNKNOWN = 0; //未确认
                    #     SOLID = 1; //单实线
                    #     DASHED = 2;//单虚线
                    #     SHORT_DASHED = 3;//短粗虚线
                    #     DOUBLE_SOLID = 4;//双实线
                    #     DOUBLE_DASHED = 5;//双虚线
                    #     LEFT_SOLID_RIGHT_DASHED = 6;//左实右虚
                    #     LEFT_DASHED_RIGHT_SOLID = 7;//左虚右实
                    #     SHADED_AREA = 8;//表达没有明确边界的场景，如导流区等分割的车道
                    #     VIRTUAL_LANE = 9;//表达除了路口内，其他场景的虚拟车道标线
                    #     VIRTUAL_JUNCTION = 10;//表达路口内的虚拟车道标线
                    #     //VIRTUAL_CURB = 11;
                    #     //UNCLOSED_ROAD = 12;
                    #     //VIRTUAL_ROAD = 13;
                    #     //OTHER = 99;
                    # };
                    ramp: bool = bool(type >> 8 & 1)
                    double: bool = bool(type >> 9 & 1)
                    dash: bool = bool(type >> 10 & 1)
                    solid = bool(type >> 11 & 1)
                    marking_type: int = 0

                    if (double):
                        if (dash and solid):
                            marking_type = 6
                        elif (solid):
                            marking_type = 4
                        elif (dash):
                            marking_type = 5
                        else:
                            marking_type = 4
                    elif (dash and solid):
                        marking_type = 6  # 根据远哥虚实同时存在，认定为左实右虚
                    elif (dash):
                        marking_type = 2
                    elif (solid):
                        marking_type = 1

                    lane_dict["geo_type"] = marking_type
                    isDiversion = 0
                    if ((type >> 18 & 1) or (type >> 20 & 1)):
                        isDiversion = 1
                    lane_dict["isDiversion"] = isDiversion
                    frame_json["LaneBoundary"].append(lane_dict)

            dict_to_json_file(to_json_file, frame_json)
        print("完成输出 pp_laneline_info to ", to_json_floder)
