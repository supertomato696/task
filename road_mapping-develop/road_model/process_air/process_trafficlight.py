import argparse
import os
import util
from util import *
from util import log

road_model_root_folder = os.path.abspath(__file__ + "/../../")
fsd_map_root_folder = os.path.abspath(__file__ + "/../../../")

sys.path.append(fsd_map_root_folder)
from preprocess.script.trail_data import TrailData, FrameData

class TrafficlightProcessing:
    def __init__(self) -> None:
        self.task_folder = ""
        self.task_data_folder = ""
        self.trail_dict: Dict[str, TrailData] = {}

    def load_data_set(self):
        for one_trail_folder in util.get_folders_path_only_number_name(os.path.join(self.task_folder, "data")):
            data_set_json_path = os.path.join(one_trail_folder, "data_set.json")
            if not os.path.exists(data_set_json_path):
                log.warning("[TrafficlightProcessing] dataset.json 不存在，跳过:", data_set_json_path)
                continue
            trail: TrailData = TrailData.init_from_json(data_set_json_path)
            self.trail_dict[trail.trail_id] = trail
        # for k, v in self.trail_dict.items():
        #     log.info("[TrafficlightProcessing] 完成载入dataset metainfo", k)

    def bev_trafficlight_tracking(self):
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["trafficlight_tracking"])
        params = []
        for trail in self.trail_dict.values():
            ff_mapping_info: dict = {"header": {}, "data": []}
            if len(trail.frame_dict.values()) == 0:
                continue
            for frame in trail.frame_dict.values():
                if (len(frame.opt_q_world_veh) != 4):
                    continue
                if (len(frame.opt_t_world_veh) != 3):
                    continue

                # bev_label_path = os.path.join(self.task_data_folder, frame.trail_id, "bev_label", str(frame.stamp_ms)+".json")
                ff_mapping_info["header"] = {"trail_id": frame.trail_id}
                ff_mapping_info["data"].append({"json_path": frame.bev_label_path, "timestamp": frame.stamp_ms, "utm_num": trail.utm_num, "t_utm_world": trail.t_utm_world, "t": frame.opt_t_world_veh, "q": frame.opt_q_world_veh})

            entry = os.path.join(road_model_root_folder, "process_air/process_tl_tracking.py")
            tl_tracking_cmd = "python3 {}  --task_folder={} --input={} --output={}".format(
                entry,
                self.task_folder,
                util.dict_to_json_file(os.path.join(self.bev_mappping_folder, f"bev_label_info_frame_{trail.trail_id}.json"), ff_mapping_info),
                os.path.join(self.bev_mappping_folder, f"trafficlight_frame_{trail.trail_id}.shp"))
            params.append([run_a_process, tl_tracking_cmd, 3600])
        thread_pool(16, params)
        log.info("finish bev_trafficlight_tracking")

    def bev_trafficlight_merging(self):
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["trafficlight_merging"])
        tl_tracking_files = ''
        for trail in self.trail_dict.values():
            # tl_tracking_path = os.path.join(self.bev_mappping_folder, "trafficlight_processing", f"trafficlight_frame_{trail.trail_id}.shp")
            # tl_tracking_files.append(tl_tracking_path)
            if tl_tracking_files == '':
                tl_tracking_files += str(trail.trail_id)
            else:
                tl_tracking_files += '-' + str(trail.trail_id)
        
        entry = os.path.join(road_model_root_folder, "process_air/process_tl_merging.py")
        trafficlight_merging_cmd = "python3 {}  --task_folder={} --input={} --output={}".format(
            entry,
            self.task_folder,
            tl_tracking_files,
            os.path.join(self.bev_mappping_folder, "trafficlight_processing/final_trafficlight.shp"),
            # os.path.join(self.bev_mappping_folder, "trafficlight_processing/final_trafficlight.pcd")
        )
        run_a_process(trafficlight_merging_cmd)
        # util.copy_file(os.path.join(self.bev_mappping_folder, "trafficlight_processing/final_trafficlight.pcd"), os.path.join(self.task_folder, "model_pcd/trafficlight_cloud.pcd"))
        log.info("finish trafficlight_merging")
    
    def make_trafficlight_topology(self):
        util.write_lines_to_file_override(os.path.join(self.task_folder, "debug/sys_monitor_stage.txt"), ["trafficlight_topology"])
        
        entry = os.path.join(road_model_root_folder, "process_air/process_tl_topology.py")
        trafficlight_merging_cmd = "python3 {} --task_folder={}".format(
            entry,
            self.task_folder,
            # os.path.join(self.bev_mappbuild_folder, "export_to_origin_shp"),
            # os.path.join(self.bev_mappbuild_folder, "export_to_origin_shp/traffic_light.shp")
        )
        run_a_process(trafficlight_merging_cmd)
        # util.copy_file(os.path.join(self.bev_mappbuild_folder, "export_to_origin_shp/traffic_light.shp"), os.path.join(self.bev_mappbuild_folder, "export_to_shp/traffic_light.shp"))
        log.info("finish make_trafficlight_topology")

    def run(self, task_folder: str):
        self.task_folder = task_folder
        self.task_data_folder = os.path.join(self.task_folder, "data")
        self.bev_mappping_folder = os.path.join(self.task_folder, "trafficlight_processing")
        self.bev_mappbuild_folder = os.path.join(self.task_folder, "bev_mapbuild_out")

        log.info("====== 数据载入 ======")
        self.load_data_set()

        log.info("====== bev_trafficlight_tracking ======")
        self.bev_trafficlight_tracking()

        log.info("====== bev_trafficlight_merging ======")
        self.bev_trafficlight_merging()

        log.info("====== make_trafficlight_topology ======")
        self.make_trafficlight_topology()


base_dir = "/mnt/d/01_code/02_dataset/crowd_source/new/site_11002458234858/model"
default_task_folder = os.path.join(base_dir, "auto_label/output")
parser = argparse.ArgumentParser()
parser.add_argument('--task_folder', type=str, default=default_task_folder)
if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.task_folder, f"log/run_bev_trafficlight.log"))
    error_code = 0
    try:
        bev_tl_proc: TrafficlightProcessing = TrafficlightProcessing()
        bev_tl_proc.run(args.task_folder)
    except Exception as e:
        log.error("error:", e)
        error_code = 732  # "交通灯建模_异常退出"
    tl_proc_result = {"error_code": error_code}
    util.dict_to_json_file(os.path.join(args.task_folder, "trafficlight_processing/trafficlight_processing_result.json"), tl_proc_result)  # 保存mapping环境结果文件
