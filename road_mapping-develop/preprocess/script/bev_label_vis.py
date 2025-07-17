from typing import List, Dict
import numpy as np
import argparse
import os
import cv2
import json
import matplotlib
from shapely.geometry import LineString
import time


TYPE_DICT = {0: 'other_unset_type', 1: "other_solid", 2: "other_dashed", 3: 'other_double_solid', 4: 'other_double_dashed',
             5: 'other_left_dashed_right_solid', 6: 'other_left_solid_right_dashed', 7: 'white_unset_type', 8: 'white_solid', 9: 'white_dashed',
             10: 'white_double_solid', 11: 'white_double_dashed', 12: 'white_left_dashed_right_solid', 13: 'white_left_solid_right_dashed', 14: 'yellow_unset_type',
             15: 'yellow_solid', 16: 'yellow_dashed', 17: 'yellow_double_solid', 18: 'yellow_double_dashed', 19: 'yellow_left_dashed_right_solid',
             20: 'yellow_left_solid_right_dashed', 21: 'unknow_road_boundary', 22: 'road_boundary_curb', 23: 'road_boundary_fence', 24: 'road_boundary_wall',
             25: 'road_boundary_surface', 26: 'road_boundary_ditch', 27: 'road_boundary_obstacle', 28: 'stopline', 29: 'crosswalk',
             30: 'polygon_unknow_kind', 31: 'polygon_flowerbed', 32: 'polygon_sentry_box', 33: 'polygon_physical_safe_island', 34: 'polygon_linear_safe_island',
             35: 'polygon_6', 36: 'lane_center', 37: 'junction', 38: 'virtual_lane_center', 39: 'arrow_unknown',
             40: 'arrow_left', 41: 'arrow_forward', 42: 'arrow_right', 43: 'arrow_left_and_forward', 44: 'arrow_right_and_forward',
             45: 'arrow_left_and_right', 46: 'arrow_u_turn', 47: 'arrow_u_turn_and_forward', 48: 'arrow_u_turn_and_left', 49: 'arrow_merge_left',
             50: 'arrow_merge_right', 51: 'crosswalk_notice', 52: 'speed_limit_low', 53: 'speed_limit_high', 54: 'arrow_no_left_turn',
             55: 'arrow_no_right_turn', 56: 'arrow_no_u_turn', 57: 'arrow_forward_and_left_and_right', 58: 'arrow_forward_and_u_turn_and_left', 59: 'arrow_right_and_u_turn',
             60: 'marking_text', 61: 'marking_time', 62: 'check_following_distance', 63: 'stopto_give_way', 64: 'slowdown_to_give_way',
             65: 'marking_nets', 66: "trafficlight_NONE", 67: "trafficlight_ROUND", 68: "trafficlight_CROSS", 69: "trafficlight_PEDESTRIAN",
             70: "trafficlight_BICYCLE", 71: "trafficlight_ARROW", 72: "trafficlight_TIME", }


class log:
    def __init__(self) -> None: pass
    @staticmethod
    def time_str(): return time.strftime('%d %H:%M:%S', time.localtime())
    @classmethod
    def info(cls, *args): print(cls.time_str(), *args)
    @classmethod
    def debug(cls, *args): print("\033[92m" + cls.time_str(), *args, "\033[00m")
    @classmethod
    def warning(cls, *args): print("\033[93m" + cls.time_str(), *args, "\033[00m")
    @classmethod
    def error(cls, *args): print("\033[91m" + cls.time_str(), *args, "\033[00m")


def json_file_to_dict(json_path: str) -> dict:
    if not os.path.exists(json_path):
        log.info("🔴 error 路径不存在", json_path)
        return {}
    data = {}
    with open(json_path, 'r') as f:
        try:
            data = json.load(f)
        except Exception as e:
            log.info('Reason: ' + str(e))
            return {}
    return data


def write_image(to_path: str, image: np.ndarray):
    folder = os.path.dirname(to_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    cv2.imwrite(to_path, image)


def get_files_path_with_suffix(folder_path='', suffix='') -> List[str]:
    if not os.path.exists(folder_path):
        log.info('get_files_path_with_suffix(), no {}'.format(folder_path))
        return []
    files_path = []
    for name in os.listdir(folder_path):
        if name.endswith(suffix):
            fp = os.path.join(folder_path, name)
            if os.path.isfile(fp):
                files_path.append(fp)
    return sorted(files_path)


def write_lines_to_file_override(file_path: str, lines: list):
    folder = os.path.dirname(file_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    s = ''
    for l in lines:
        s += (str(l).strip('\n') + '\n')
    open(file_path, 'w').write(s.strip('\n'))


class BEVLine:
    def __init__(self, line_id: int, type_id: int, bev_score: float, points: list) -> None:
        self.line_id: int = line_id
        self.type_id: int = type_id
        self.bev_score: float = bev_score
        self.type_name: str = TYPE_DICT[type_id] if type_id in TYPE_DICT.keys() else "NO_DEFINE"
        self.points: np.ndarray = np.array(points)
        self.line: LineString = LineString([(x, y) for x, y in self.points[:, :2]])
        self.line_length = self.line.length
        self.tag = []

    def to_image_points(self, max_x, min_x, max_y, min_y, pixel_size) -> list:
        width = int(round((max_y-min_y)/pixel_size))
        height = int(round((max_x-min_x)/pixel_size))
        points = []
        for point in self.points:
            x = int(round((max_y-point[1])/pixel_size))
            y = int(round((max_x-point[0])/pixel_size))
            points.append([x, y])
            if (x < 0 or y < 0 or x >= width or y >= height):
                continue
        return points

    def info_str(self) -> str:
        tag = ""
        for one in self.tag:
            tag += f"{one}|"
        s = f"{self.line_id}|{self.type_id}-{self.type_name}|s{round(self.bev_score,1)}|{round(self.line_length,1)}m {tag}"
        s.strip("|")
        return s


class BEVFrame:
    def __init__(self, stamp, bev_label_path) -> None:
        self.stamp = stamp
        self.frame_id = str(stamp)
        self.bev_label_path = bev_label_path
        self.line_list: List[BEVLine] = []
        self.load_frame_bev_label()

    def load_frame_bev_label(self):
        bev_label_info = json_file_to_dict(self.bev_label_path)
        instance_list = bev_label_info["detail"]["instance_list"]
        instance_n = len(instance_list)
        for line_id in range(0, instance_n):
            one_instance = instance_list[line_id]
            if "points" in one_instance["data"].keys() and len(one_instance["data"]["points"]) > 0:
                bev_line: BEVLine = BEVLine(line_id, one_instance["attrs"]["type"], one_instance["attrs"]["score"], one_instance["data"]["points"])
                self.line_list.append(bev_line)
            elif "3dbox" in one_instance["data"].keys() and len(one_instance["data"]["3dbox"]) > 0:
                points = [one_instance["data"]["3dbox"][0],
                          one_instance["data"]["3dbox"][1],
                          one_instance["data"]["3dbox"][2],
                          one_instance["data"]["3dbox"][3],
                          one_instance["data"]["3dbox"][6],
                          one_instance["data"]["3dbox"][7],
                          one_instance["data"]["3dbox"][4],
                          one_instance["data"]["3dbox"][5],
                          one_instance["data"]["3dbox"][0]]
                bev_line: BEVLine = BEVLine(line_id, one_instance["attrs"]["type"], one_instance["attrs"]["score"], points)
                self.line_list.append(bev_line)

    def output_frame_bev_txt(self, txt_path: str):
        lines = ["type x y z"]
        for good_line in self.line_list:
            for one_point in good_line.points:
                if (len(one_point) == 2):
                    lines.append(f"{good_line.type_id} {one_point[0]} {one_point[1]} 0")
                elif (len(one_point) == 3):
                    lines.append(f"{good_line.type_id} {one_point[0]} {one_point[1]} {one_point[2]}")
        write_lines_to_file_override(txt_path, lines)

    def output_frame_bev_png(self, png_path: str):
        max_x, min_x = 32, -32
        max_y, min_y = 60, -60
        pixel_size = 0.1
        width = int(round((max_y-min_y)/pixel_size))
        height = int(round((max_x-min_x)/pixel_size))
        img = np.zeros((height, width, 3), np.uint8)
        veh_center_x = int(round(max_y/pixel_size))
        veh_center_y = int(round(max_x/pixel_size))
        cv2.rectangle(img, pt1=(veh_center_x+10, veh_center_y+20),  pt2=(veh_center_x-10, veh_center_y-20), color=(100, 100, 100),  thickness=1)
        cv2.putText(img, f"{self.frame_id}", (0, 10), cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 255], 1)

        def draw(one_line: BEVLine):
            points = np.array(one_line.to_image_points(max_x, min_x, max_y, min_y, pixel_size))
            cv2.putText(img, f"{one_line.line_id}", (points[0][0], points[0][1]+(one_line.line_id % 3)*10), cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)
            cv2.putText(img, f"{one_line.line_id}", (points[-1][0], points[-1][1]-(one_line.line_id % 2)*10), cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)
            if (one_line.line_id <= 55):
                cv2.putText(img, one_line.info_str(), [0, 25+one_line.line_id*11], cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)
            else:
                cv2.putText(img, one_line.info_str(), [800, (one_line.line_id-55)*11], cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)
            cv2.polylines(img, [points], False, rgb, thickness=1)
        for good_line in self.line_list:
            rgb = [int(255 * x) for x in matplotlib.cm.jet(good_line.type_id / len(TYPE_DICT))[:3]]
            draw(good_line)
        write_image(png_path, img)


class BEVLabelVis:
    def __init__(self) -> None:
        self.bev_label_folder = ""
        self.bev_label_vis_folder = ""
        self.bevframe_dict: Dict[str, BEVFrame] = {}

    def load_bev_label_json(self):
        all_json_path = get_files_path_with_suffix(self.bev_label_folder, ".json")
        for one_json_path in all_json_path:
            file_name = os.path.basename(one_json_path)
            stamp_str = file_name.split(".")[0]
            stamp = int(stamp_str)
            self.bevframe_dict[stamp] = BEVFrame(stamp, one_json_path)
        log.info(f"共载入{len(self.bevframe_dict)}个 frame")

    def run(self, bev_label_folder, bev_label_vis_folder, bev_label_txt_folder):
        self.bev_label_folder = bev_label_folder
        self.bev_label_vis_folder = bev_label_vis_folder
        self.bev_label_txt_folder = bev_label_txt_folder
        log.info("[BEVLabelVis] 载入数据 from:", bev_label_folder)
        self.load_bev_label_json()
        bevframe: BEVFrame
        # for bevframe in self.bevframe_dict.values():
        #     bevframe.output_frame_bev_png(os.path.join(self.bev_label_vis_folder, f"{bevframe.stamp}.png"))
        #     bevframe.output_frame_bev_txt(os.path.join(self.bev_label_txt_folder, f"{bevframe.stamp}.txt"))
        # log.info("[BEVLabelVis] Finish 可视化:", bev_label_folder)


parser = argparse.ArgumentParser()
parser.add_argument('--bev_label_folder', type=str)
parser.add_argument('--bev_label_vis_folder', type=str)
parser.add_argument('--bev_label_txt_folder', type=str)

if __name__ == "__main__":
    args = parser.parse_args()
    log.debug("==============================================================")
    log.debug("=========            Hello bev_label_vis              ========")
    log.debug("==============================================================")
    bev_label_vis: BEVLabelVis = BEVLabelVis()
    bev_label_vis.run(args.bev_label_folder, args.bev_label_vis_folder, args.bev_label_txt_folder)


#  python3 /home/test/code/fsd_map_bev_develop/preprocess/script/bev_label_vis.py --bev_label_folder=/home/test/lidar_mapping_ws/fs_10001561/data/1685243432460/bev_label --bev_label_vis_folder=/home/test/lidar_mapping_ws/fs_10001561/data/1685243432460/bev_label_vis --bev_label_txt_folder=/home/test/lidar_mapping_ws/fs_10001561/data/1685243432460/bev_label_txt
