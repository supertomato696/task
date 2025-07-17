import argparse
import os
import util
from util import *
from util import log

mapping_root_folder = os.path.abspath(__file__ + "/../../")
fsd_map_root_folder = os.path.abspath(__file__ + "/../../../")

sys.path.append(fsd_map_root_folder)
from preprocess.script.trail_data import TrailData, FrameData
# from trail_data import TrailData, FrameData

class BEVMapping:
    def __init__(self) -> None:
        self.task_folder = ""
        self.task_data_folder = ""
        self.trail_dict: Dict[str, TrailData] = {}
        self.info_json_path = ""
        self.data_type = ""
        self.debug_lidar_bev_mapping = 0

    def load_data_set(self):
        for one_trail_folder in util.get_folders_path_only_number_name(os.path.join(self.task_folder, "data")):
            data_set_json_path = os.path.join(one_trail_folder, "data_set.json")
            if not os.path.exists(data_set_json_path):
                log.warning("[BEVMapping] dataset.json 不存在，跳过:", data_set_json_path)
                continue
            trail: TrailData = TrailData.init_from_json(data_set_json_path)
            self.trail_dict[trail.trail_id] = trail
        for k, v in self.trail_dict.items():
            log.info("[BEVMapping] 完成载入dataset", k)

    def bev_line_tracking(self):
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["BEV_Track"])
        params = []
        trail: TrailData
        for trail in self.trail_dict.values():
            data_trail_folder = os.path.join(self.task_data_folder, f"{trail.trail_id}")
            bev_trail_folder = os.path.join(self.bev_mappping_folder, f"bev_line_track/{trail.trail_id}")
            bev_mapper_cmd = "python3 {} --data_trail_folder={} --bev_trail_folder={} --info_json_path={}".format(
                os.path.join(mapping_root_folder, "script/bev_line_track.py"),
                data_trail_folder, bev_trail_folder, self.info_json_path)
            util.write_lines_to_file_override(os.path.join(bev_trail_folder, f"bev_line_track_cmd.sh"), ["export DEBUG_LIDAR_MAPPING=YES", bev_mapper_cmd])
            params.append([run_a_process, bev_mapper_cmd, 3600])
        thread_pool(int(cpu_count()*0.8), params)

    def make_flat_scan(self):
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["Flat_Scan"])
        log.info("make_flat_scan")
        params = []
        trail: TrailData
        for trail in self.trail_dict.values():
            one_trail_folder = os.path.join(self.task_data_folder, f"{trail.trail_id}")
            data_set_json_path = os.path.join(one_trail_folder, "data_set.json")
            if not os.path.exists(data_set_json_path):
                continue
            entry = os.path.join(fsd_map_root_folder, "preprocess/pointcloudrender/bin/flat_scan_maker")
            cmd = "{} --data_set_path={} --output_folder={}".format(
                entry,
                data_set_json_path,
                os.path.join(one_trail_folder, "flat_scan"))
            params.append([run_a_process, cmd, 3600])
        thread_pool(8, params)
        log.info("finish make_flat_scan")

    def flat_mapping(self):
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["Flat建图"])
        log.info("flat_mapping")
        ff_mapping_info: dict = {"header": {}, "utm_num": {}, "t_utm_world": [], "lidar_data": [], "data": []}
        # 0 处理utm_zone 和 地图原点
        with open(self.info_json_path, 'r') as f:
            info_data = json.load(f)
        f.close

        ff_mapping_info["utm_num"] = info_data.get('utm_num')
        ff_mapping_info["t_utm_world"] = info_data.get('t_utm_world')

        # 1 处理lidar
        for trail in self.trail_dict.values():
            for frame in trail.lidar_frame_dict.values():
                if (len(frame.opt_q_world_veh) != 4):
                    continue
                if (len(frame.opt_t_world_veh) != 3):
                    continue

                flat_scan_path = frame.lidar_label_path
                ff_mapping_info["lidar_data"].append({"cloud_path": flat_scan_path, "t": frame.opt_t_world_veh, "q": frame.opt_q_world_veh})

        # 2 处理bev
        for trail in self.trail_dict.values():
            for frame in trail.frame_dict.values():
                if (len(frame.opt_q_world_veh) != 4):
                    continue
                if (len(frame.opt_t_world_veh) != 3):
                    continue

                # TODO:qzc
                # if (frame.veh_scan_path == ""):
                #     continue
                # pos = frame.veh_scan_path.rfind('/veh_scan/')
                # flat_scan_path = frame.veh_scan_path[:pos] + '/flat_scan/' + frame.veh_scan_path[pos + len('/veh_scan/'):]

                flat_scan_path = os.path.join(self.task_data_folder, frame.trail_id, "flat_scan", str(frame.stamp_ms)+".pcd")
                ff_mapping_info["data"].append({"cloud_path": flat_scan_path, "bev_json_path": frame.bev_label_path, "t": frame.opt_t_world_veh, "q": frame.opt_q_world_veh})
        ff_mapper_cmd = "{} --input={} --output={} --data_type={} --debug_lidar_bev_mapping={}".format(
            os.path.join(mapping_root_folder, "simple_mapper/bin/flat_mapper"),
            util.dict_to_json_file(os.path.join(self.bev_mappping_folder, f"flat_mapping/flat_mapping_info.json"), ff_mapping_info),
            os.path.join(self.bev_mappping_folder, "flat_mapping"),
            self.data_type,
            self.debug_lidar_bev_mapping)
        util.write_lines_to_file_override(os.path.join(self.bev_mappping_folder, f"flat_mapping/flat_mapping_cmd.sh"), [ff_mapper_cmd])
        run_a_process(ff_mapper_cmd)
        # util.copy_file(os.path.join(self.bev_mappping_folder, "flat_mapping/ground.pcd"), os.path.join(self.task_folder, "model_pcd/flat_global_cloud.pcd"))
        # util.copy_file(os.path.join(self.bev_mappping_folder, "flat_mapping/final_map.pcd"), os.path.join(self.task_folder, "model_pcd/global_cloud.pcd"))
        if os.path.exists(os.path.join(self.bev_mappping_folder, "flat_mapping/ground_bev.pcd")):    
            util.move_file(os.path.join(self.bev_mappping_folder, "flat_mapping/ground_bev.pcd"), os.path.join(self.task_folder, "model_pcd/global_cloud_bev.pcd"))
        if os.path.exists(os.path.join(self.bev_mappping_folder, "flat_mapping/ground_lidar.pcd")):      
            util.move_file(os.path.join(self.bev_mappping_folder, "flat_mapping/ground_lidar.pcd"), os.path.join(self.task_folder, "model_pcd/global_cloud_lidar.pcd"))
            
        util.move_file(os.path.join(self.bev_mappping_folder, "flat_mapping/ground.pcd"), os.path.join(self.task_folder, "model_pcd/global_cloud.pcd"))
        util.move_file(os.path.join(self.bev_mappping_folder, "flat_mapping/pointcloud_semantic_pure_lidar_gcj02.pcd"), os.path.join(self.task_folder, "model_pcd/pointcloud_semantic_pure_lidar_gcj02.pcd"))
        util.move_file(os.path.join(self.bev_mappping_folder, "flat_mapping/pointcloud_semantic_gcj02.pcd"), os.path.join(self.task_folder, "model_pcd/pointcloud_semantic_gcj02.pcd"))
        util.move_file(os.path.join(self.bev_mappping_folder, "flat_mapping/sm.pcd"), os.path.join(self.task_folder, "model_pcd/sm.pcd"))
        log.info("finish flat_mapping")

    def run(self, task_folder: str, info_json_path: str, data_type: str, debug_lidar_bev_mapping: int):
        self.task_folder = task_folder
        self.task_data_folder = os.path.join(self.task_folder, "data")
        self.bev_mappping_folder = os.path.join(self.task_folder, "bev_mapping")
        self.info_json_path = info_json_path
        self.data_type = data_type
        self.debug_lidar_bev_mapping = debug_lidar_bev_mapping

        log.info("====== 数据载入 ======")
        self.load_data_set()

        log.info("====== link tracking ======")
        self.bev_line_tracking()

        log.info("====== make_flat_scan ======")
        self.make_flat_scan()

        log.info("====== flat_mapping ======")
        self.flat_mapping()


# base_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross/"
# default_task_folder = os.path.join(base_dir, "11002417915690/PL0169_event_trafficlight_status_event_20240902-101706_0/223126")
default_task_folder = "/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0224/new_lidar_bev2/lidar_byd/503601/auto_label/output_bev_lidar"
default_info_json_path = "/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0224/new_lidar_bev2/lidar_byd/503601/auto_label/task_info_demo.json"
default_debug_ldiar_bev_mapping = 0 # 0：不生成， 1: 在task type = 3 时，且存在bev资料，则用激光pose+BEV资料生成bev地图, 2: 融合前生成纯激光的地图， 3：分别生成bev 和 激光地图
parser = argparse.ArgumentParser()
parser.add_argument('--task_folder', type=str, default=default_task_folder)
parser.add_argument('--info_json_path', type=str, default=default_info_json_path)
parser.add_argument('--data_type', type=str, default="", help="数据类型")
parser.add_argument('--debug_lidar_bev_mapping', type=int, default=default_debug_ldiar_bev_mapping, help="task_type=3时,用激光pose+BEV建图")
if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.task_folder, f"log/run_bev_mapping.log"))
    error_code = 0
    try:
        lidar_pose_optim: BEVMapping = BEVMapping()
        lidar_pose_optim.run(args.task_folder, args.info_json_path, args.data_type, args.debug_lidar_bev_mapping)
    except Exception as e:
        log.error("error:", e)
        error_code = 412  # "位姿优化_异常退出"
    pose_optim_result = {"error_code": error_code}
    util.dict_to_json_file(os.path.join(args.task_folder, "bev_mapping/bev_mapping_result.json"), pose_optim_result)  # 保存mapping环境结果文件
