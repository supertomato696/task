import argparse
from trail_data import TrailData
import util
from util import *
from util import log
import os
import sys

lidar_mapping_root_folder = os.path.abspath(__file__+"/../../")
project_root_folder = os.path.abspath(__file__+"/../../../")
# task_type_name_dict = {"zl": "连续直路", "lk2": "离散路口", "zb_manual": "众包人工", "zb_auto": "众包自动", "zb_auto_zc": "自采自动"}
sys.path.append(project_root_folder)
from data_io.script.bev_label_map import DataType


class PreProcess:
    def __init__(self) -> None:
        self.task_folder: str = ""
        self.task_data_folder: str = ""
        self.tile_download_folder: str = ""
        self.output_folder: str = ""
        self.utm_num: int = 0
        self.t_utm_world: list = []
        self.trail_dict: Dict[int, TrailData] = {}
        self.data_check_error_log = {}
        self.preprocess_result = {}
        self.sparse_data: bool = True

    def make_link(self, fake_link, true_link):
        if not os.path.exists(true_link):
            return
        util.add_folder_in_file_path_if_no_exist(fake_link)
        rel_path = util.get_relpath(true_link, fake_link)
        log.info(rel_path)
        rel_path = rel_path.replace("../../../", "../../")
        # 删除可能占据虚拟文件的东西
        if os.path.exists(fake_link):
            return
        try:
            util.del_folder_if_exist(fake_link)
            util.del_file_if_exist(fake_link)
            os.unlink(fake_link)
        except Exception as e:
            pass
        # 创建虚拟链接文件
        os.symlink(rel_path, fake_link)

    def make_data_link(self):
        if not os.path.exists(self.tile_download_folder):
            log.info("不存在", self.tile_download_folder, "不执行数据复制")
            return
        all_download_tile_folder = util.get_folders_path_only_number_name(self.tile_download_folder)
        all_download_trail_folder = []
        for one_tile_folder in all_download_tile_folder:
            all_download_trail_folder.extend(util.get_folders_path_only_number_name(one_tile_folder))
        for one_trail_folder in all_download_trail_folder:
            trail_id = util.get_folder_name_in_path(one_trail_folder)
            if trail_id in self.data_check_error_log.keys():
                trail_id = trail_id+"_bad"
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "lidar"),
                           os.path.join(one_trail_folder, "lidar"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "label_raw"),
                           os.path.join(one_trail_folder, "label_raw"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "lidar_label"),
                           os.path.join(one_trail_folder, "lidar_label"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "front_camera_120"),
                           os.path.join(one_trail_folder, "f_ca_120"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "perception_raw"),
                           os.path.join(one_trail_folder, "perception_raw"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "calib_cam_params.json"),
                           os.path.join(one_trail_folder, "calib_cam_params.json"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "calib_lidar_params.json"),
                           os.path.join(one_trail_folder, "calib_lidar_params.json"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "nav_gnss.txt"),
                           os.path.join(one_trail_folder, "nav_gnss.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "nav_imu.txt"),
                           os.path.join(one_trail_folder, "nav_imu.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "nav_ins.txt"),
                           os.path.join(one_trail_folder, "nav_ins.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "ndm_location.txt"),
                           os.path.join(one_trail_folder, "ndm_location.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "odometry.txt"),
                           os.path.join(one_trail_folder, "odometry.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "odometry3d.txt"),
                           os.path.join(one_trail_folder, "odometry3d.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "opt_pose_veh.txt"),
                           os.path.join(one_trail_folder, "opt_pose_veh.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "perception_laneline.txt"),
                           os.path.join(one_trail_folder, "perception_laneline.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "vehicle_status.txt"),
                           os.path.join(one_trail_folder, "vehicle_status.txt"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "bev_label"),
                           os.path.join(one_trail_folder, "bev_label"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "bevlane_f_ca_120"),
                           os.path.join(one_trail_folder, "bevlane_f_ca_120"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "bevlane_semantic_mask_f_ca_120"),
                           os.path.join(one_trail_folder, "semantic_mask"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "bevlane_semantic_mask_f_ca_120"),
                           os.path.join(one_trail_folder, "bevlane_semantic_mask_f_ca_120"))
            self.make_link(os.path.join(self.task_data_folder, str(trail_id), "bevlane_perception_f_ca_120"),
                           os.path.join(one_trail_folder, "bevlane_perception_f_ca_120"))

    def remove_bad_scan(self):
        all_trail_folder = util.get_folders_path_only_number_name(self.task_data_folder)
        for trail_folder in all_trail_folder:
            lidar_folder = os.path.join(trail_folder, "lidar")
            all_bad_path = util.get_files_path_with_suffix(lidar_folder, ".bad")
            for one_bad_path in all_bad_path:
                name = one_bad_path.split("/")[-1].split(".")[0]
                pcd_path = os.path.join(lidar_folder, name + ".pcd")
                util.rename_file(one_bad_path, pcd_path)
            all_pcd_path = util.get_files_path_with_suffix(lidar_folder, ".pcd")
            for one_pcd_path in all_pcd_path:
                this_size = util.get_filesize_B(one_pcd_path)
                if this_size < 500*1024:
                    util.rename_file(one_pcd_path, one_pcd_path+".bad")

    def get_firmware_ins_info(self):
        try:
            bad_ins_cnt = 0
            all_ins_cnt = 0
            new_lidar_firmware = False
            trail_folders_path: list = util.get_folders_path_only_number_name(self.task_data_folder)
            for one_trail_folder in trail_folders_path:
                firmware_ins_param_json = util.json_file_to_dict(os.path.join(one_trail_folder, "firmware_ins_param_json.json"))
                new_lidar_firmware = firmware_ins_param_json["firmware"]
                bad_ins_cnt += firmware_ins_param_json["bad_ins"]
                all_ins_cnt += firmware_ins_param_json["all_ins"]

            if all_ins_cnt > 0:
                bad_ins_rate = f"{(round(100*bad_ins_cnt/all_ins_cnt, 2))}%"
            else:
                bad_ins_rate = ""
            lidar_firmware = "新" if (new_lidar_firmware == True) else "旧"

            return lidar_firmware, bad_ins_rate
        except Exception as e:
            return "", ""

    def run(self, task_folder: str, utm_num: int, t_utm_world: list, data_type: str, sparse_data: bool) -> int:
        self.task_folder = task_folder
        self.utm_num = utm_num
        self.t_utm_world = t_utm_world
        self.data_type = data_type
        self.sparse_data = sparse_data
        self.task_data_folder = os.path.join(self.task_folder, "data")
        self.tile_download_folder = os.path.join(self.task_folder, "tile_download")
        self.output_folder = os.path.join(self.task_folder, "output")

        print("--------------------------------------------")
        print("                建立数据软链接                ")
        print("--------------------------------------------")
        self.make_data_link()

        print("--------------------------------------------")
        print("            补充 trail info 文件             ")
        print("--------------------------------------------")
        data_meta = util.json_file_to_dict(os.path.join(self.task_folder, "data_meta.json"))
        for one_trail in data_meta["trail_dict"].values():
            trail_info = {"trail_id": one_trail["trail_id"],
                          "download_tile_id": one_trail["download_tile_id"],
                          "download_branch": one_trail["download_branch"],
                          "data_scope": data_meta["task_geom_local"],
                          "binded_links": [],
                          "all_links": []}
            if "link_dict" in data_meta.keys():
                for link_json in data_meta["link_dict"].values():
                    trail_info["all_links"].append(link_json)
                    if link_json["link_id"] in one_trail["link_id_list"]:
                        trail_info["binded_links"].append(link_json)
            if os.path.exists(os.path.join(self.task_data_folder, str(one_trail["trail_id"]))):
                util.dict_to_json_file(os.path.join(self.task_data_folder, str(one_trail["trail_id"]), "trail_info.json"), trail_info)

        print("--------------------------------------------")
        print("                剔除lidar坏数据               ")
        print("--------------------------------------------")
        # self.remove_bad_scan()

        print("--------------------------------------------")
        print("            data_set_maker 阶段1             ")
        print("--------------------------------------------")
        all_trail_folder: list = util.get_folders_path_only_number_name(self.task_data_folder)
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["数据预处理-阶段1"])
        params = []
        for one_trail_folder in all_trail_folder:
            trail_id = util.get_folder_name_in_path(one_trail_folder)
            entry = os.path.join(lidar_mapping_root_folder, "script/data_set_maker_qzc.py")
            cmd = "python3 {} --data_folder={} --utm_num={} --t_utm_world={} --data_type={} --sparse_data={} --stage=1".format(
                entry,
                os.path.join(self.task_folder, "data", str(trail_id)),
                self.utm_num,
                json.dumps(self.t_utm_world).replace(' ', ''),
                self.data_type,
                str(self.sparse_data))
            params.append([run_a_process, cmd, 3600])
        thread_pool(min(30, cpu_count()), params)

        print("--------------------------------------------")
        print("            data_set_maker 阶段2             ")
        print("--------------------------------------------")
        all_trail_folder: list = util.get_folders_path_only_number_name(self.task_data_folder)
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["数据预处理-阶段2"])
        params = []
        for one_trail_folder in all_trail_folder:
            trail_id = util.get_folder_name_in_path(one_trail_folder)
            entry = os.path.join(lidar_mapping_root_folder, "script/data_set_maker_qzc.py")
            cmd = "python3 {} --data_folder={} --utm_num={} --t_utm_world={} --data_type={} --stage=2".format(
                entry,
                os.path.join(self.task_folder, "data", str(trail_id)),
                self.utm_num,
                json.dumps(self.t_utm_world).replace(' ', ''),
                self.data_type)
            params.append([run_a_process, cmd, 3600])
        thread_pool(8, params)

        # error_code = 0
        # all_valid_frames_num = 0
        # for one_trail_folder in all_trail_folder:
        #     veh_scan_folder = os.path.join(one_trail_folder, "veh_scan")
        #     all_valid_frames_num += len(util.get_files_path_recursion(veh_scan_folder))
        # if all_valid_frames_num == 0:
        #     error_code = 205  # 筛选为空
        # if all_valid_frames_num < 3:
        #     error_code = 206  # 筛选<3

        # lidar_firmware, bad_ins_rate = self.get_firmware_ins_info()
        lidar_firmware = ''
        bad_ins_rate = ''

        # TODO:qzc,force set true
        error_code = 0

        preprocess_result = {"error_code": error_code,
                             "lidar_firmware": lidar_firmware,
                             "bad_ins_rate": bad_ins_rate}
        log.info("preprocess_result", preprocess_result)
        util.dict_to_json_file(os.path.join(self.task_data_folder, "preprocess_result.json"), preprocess_result)


default_task_folder = "/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0224/new_lidar_bev2/lidar_byd/503601/auto_label/output_bev_lidar"
default_utm_num = '50'
default_t_utm_world = '[226409.49436410138,2510825.143938201,0]'
# default_data_type = DataType.BYD_BEV.name
default_data_type = DataType.BYD_LIDAR_BEV_B.name
parser = argparse.ArgumentParser()

parser.add_argument('--task_folder', type=str, default=default_task_folder)
parser.add_argument('--utm_num', type=str, default=default_utm_num)
parser.add_argument('--t_utm_world', type=str, default=default_t_utm_world)
parser.add_argument('--data_type', type=str, default=default_data_type)
parser.add_argument('--sparse_data', type=lambda x: x.lower() == 'true', default=False)

if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.task_folder, f"log/data_preprocess.log"))
    preprocess: PreProcess = PreProcess()
    log.warning("PreProcess sparse_data:", args.sparse_data)
    preprocess.run(task_folder=args.task_folder,
                   utm_num=int(args.utm_num),
                   t_utm_world=json.loads(args.t_utm_world),
                   data_type=args.data_type,
                   sparse_data=args.sparse_data)
