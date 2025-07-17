# -*- coding: utf-8 -*-
# @Time: 2024/10/28 17:07
# @Author: sheng.guangwei
# @Team: BYD

import math
import json
import numpy as np
from pyproj import Proj
from collections import defaultdict
from shapely.geometry import Polygon

import os,sys
project_root_folder = os.path.abspath(__file__+"/../../../")
sys.path.append(project_root_folder)

from data_io.script.bev_label_map import *


def get_width_length(polygon: Polygon):
    rotated_rectangle = polygon.minimum_rotated_rectangle
    minx, miny, maxx, maxy = rotated_rectangle.bounds
    length = maxx - minx
    width = maxy - miny
    length1 = length
    width1 = width
    if length1 < width1:
        length = width1
        width = length1

    return length, width, rotated_rectangle.area

def calculate_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = math.degrees(math.atan2(dy, dx))
    if angle < 0 :
        angle += 360
        angle = -angle
    return angle

def cal_distance(p1, p2):
    return math.sqrt((p2[0]-p1[0]) ** 2 + (p2[1]-p1[1]) ** 2)

def get_angle(polygon: Polygon):
    rotated_rectangle = polygon.minimum_rotated_rectangle
    exterior_coords = rotated_rectangle.exterior.coords


    max_length = 0
    longest_edge_start = None
    longest_edge_end = None
    for i in range(len(exterior_coords) -1):
        p1 = exterior_coords[i]
        p2 = exterior_coords[i+1]
        length = cal_distance(p1, p2)
        if length > max_length:
            max_length = length
            longest_edge_start = p1
            longest_edge_end = p2
    
    orientation_angle = calculate_angle(longest_edge_start, longest_edge_end)
    
    return orientation_angle

def _get_gauss_kruger_proj(lon_list):
    """获取指定经度list的投影坐标系定义"""
    lon_min = np.min(lon_list)
    lon_max = np.max(lon_list)
    lon_center = float(np.mean((lon_min, lon_max)))
    lon_center = str(round(lon_center, 2))
    proj_str = "+proj=tmerc +lat_0=0 +lon_0=" + lon_center + " +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs"
    # proj_str = "+proj=tmerc +lat_0=0 +lon_0=114 +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs"
    # proj_gk = Proj(proj_crs, preserve_units=False)

    return proj_str

# def _get_frame_par(bev_frame_data):
#     f_meta = bev_frame_data['meta']
#     if 'egopose_relevant_frame' in f_meta.keys():
#         if isinstance(f_meta['egopose_relevant_frame']['timestamp_us'], int):
#             bev_frame = f_meta['egopose_relevant_frame']['sequence']
#             bev_timestamp = f_meta['egopose_relevant_frame']['timestamp_us']
#         else:
#             bev_frame = ""  # todo: json支持输出meta.sequence
#             bev_timestamp = f_meta['sensor_timestamp_us']
#     else:
#         bev_frame = ""  # todo: json支持输出meta.sequence
#         bev_timestamp = f_meta['sensor_timestamp_us']

#     return bev_frame, bev_timestamp

class BevReader(object):
    def get_ins_dict_qzc(self, egopose_path, save_egopose_txt):
        pass

    def get_egopose_dict_qzc(self, egopose_path, save_egopose_txt, use_opt_pose):
        pass

    def get_lane_center_qzc(self, bev_frame_data):
        pass

    def get_lane_boundary_qzc(self, bev_frame_data):
        pass

    def get_road_boundary_qzc(self, bev_frame_data):
        pass

    def get_stopline_qzc(self, bev_frame_data):
        pass

    def get_arrow_qzc(self, bev_frame_data):
        pass

    def get_cross_walk_qzc(self, bev_frame_data):
        pass

    def get_traffic_light_qzc(self, bev_frame_data):
        pass

    def get_traffic_light_bulb_qzc(self, bev_frame_data):
        pass


class BevReaderMmtRc(BevReader):
    # def get_frame_and_time(self, bev_frame_data):
    #     return _get_frame_par(bev_frame_data)

    def get_ins_dict_qzc(self, egopose_path, save_egopose_txt):
        """将mmt的egopose信息（mla-egopose.json）读取到dict中"""

        with open(egopose_path, 'r', encoding='utf-8') as file:
            egopose_data = json.load(file)

        # data.append([j["header"]["frame_id"],
        #             j["header"]["stamp"],
        #             j["ts"],
        #             j["position"]["x"],
        #             j["position"]["y"],
        #             j["position"]["z"],
        #             j["quaternion"]["w"],
        #             j["quaternion"]["x"],
        #             j["quaternion"]["y"],
        #             j["quaternion"]["z"],
        #             j["speed"]])
        save_lines = ""
        for egopose in egopose_data:
            rt_timestamp = float(egopose['meta']['timestamp_us'])
            rt_frame = egopose['meta']['seq']
            q_w = float(egopose['orientation']['quaternion_global']['w'])
            q_x = float(egopose['orientation']['quaternion_global']['x'])
            q_y = float(egopose['orientation']['quaternion_global']['y'])
            q_z = float(egopose['orientation']['quaternion_global']['z'])
            lat = float(egopose['position']['position_global']['latitude'])
            lon = float(egopose['position']['position_global']['longitude'])
            height = float(egopose['position']['position_global']['altitude'])

            one_pose = {
                "ts": rt_timestamp,
                "header":{
                    "frame_id": rt_frame,
                    "stamp": rt_timestamp
                },
                "position":{
                    "longitude": lon,
                    "latitude": lat,
                    "height": height,
                },
                "quaternion":{
                    "x": q_x,
                    "y": q_y,
                    "z": q_z,
                    "w": q_w,
                },
                "speed": 0,
                "position_type": 0
            }
            save_lines += json.dumps(one_pose)
            save_lines += '\n'

        with open(save_egopose_txt, 'w', encoding='utf-8') as out_file:
            out_file.writelines(save_lines)

    def get_egopose_dict_qzc(self, egopose_path, save_egopose_txt, use_opt_pose):
        """将mmt的egopose信息（mla-egopose.json）读取到dict中"""
        
        egopose_data = None
        with open(egopose_path, 'r', encoding='utf-8') as file:
            egopose_data = json.load(file)

        # data.append([j["header"]["frame_id"],
        #             j["header"]["stamp"],
        #             j["ts"],
        #             j["position"]["x"],
        #             j["position"]["y"],
        #             j["position"]["z"],
        #             j["quaternion"]["w"],
        #             j["quaternion"]["x"],
        #             j["quaternion"]["y"],
        #             j["quaternion"]["z"],
        #             j["speed"]])
        save_lines = ""
        for egopose in egopose_data:
            rt_timestamp = float(egopose['meta']['timestamp_us'])
            rt_frame = egopose['meta']['seq']
            if use_opt_pose: # local: enu
                q_w = float(egopose['orientation']['quaternion_local']['w'])
                q_x = float(egopose['orientation']['quaternion_local']['x'])
                q_y = float(egopose['orientation']['quaternion_local']['y'])
                q_z = float(egopose['orientation']['quaternion_local']['z'])
                x = float(egopose['position']['position_local']['x'])
                y = float(egopose['position']['position_local']['y'])
                z = float(egopose['position']['position_local']['z'])
            else:
                q_w = float(egopose['orientation']['quaternion_local']['w'])
                q_x = float(egopose['orientation']['quaternion_local']['x'])
                q_y = float(egopose['orientation']['quaternion_local']['y'])
                q_z = float(egopose['orientation']['quaternion_local']['z'])
                x = float(egopose['position']['position_local']['x'])
                y = float(egopose['position']['position_local']['y'])
                z = float(egopose['position']['position_local']['z'])

            one_pose = {
                "ts": rt_timestamp,
                "header":{
                    "frame_id": rt_frame,
                    "stamp": rt_timestamp
                },
                "position":{
                    "x": x,
                    "y": y,
                    "z": z,
                },
                "quaternion":{
                    "x": q_x,
                    "y": q_y,
                    "z": q_z,
                    "w": q_w,
                },
                "speed": 0
            }
            save_lines += json.dumps(one_pose)
            save_lines += '\n'

        with open(save_egopose_txt, 'w', encoding='utf-8') as out_file:
            out_file.writelines(save_lines)

        if len(egopose_data) == 0:
            print("failed read bev pose: ", egopose_path)
            return -1
        
        return 0

    def get_lane_center_qzc(self, bev_frame_data):
        """将mmt的车道中心线感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        lane_centers = bev_frame_data['landmarks']['lane_centers']
        for i in range(len(lane_centers)):  # 逐个对象处理
            line_object = lane_centers[i]
            if line_object['score'] == None or line_object['lane_types'][0] == None or \
                line_object['track_id']['id'] == None :
                continue

            if line_object['score'] < 0.8:
                continue
        
            line_points = list([list(d.values()) for d in line_object['line_points']])  # 将线坐标转换为list
            if len(line_points) < 2:
                continue

            raw_bev_ids = []
            for type_value in line_object['lane_types']:
                raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_LANE_CENTER', 3, type_value)
                raw_bev_ids.append(raw_bev_id)
            # if len(line_object['lane_types']) > 1:
            #     print("line_center组合属性:", line_object['lane_types'])

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "type": line_object['lane_types'][0], # 车道类型
                # "big_type": 3, # 用于跟踪使用
                "type": raw_bev_ids, # 车道类型
                "score": line_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = line_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "center line"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_lane_boundary_qzc(self, bev_frame_data):
        """将mmt的车道边界感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        lane_boundaries = bev_frame_data['landmarks']['lane_boundaries']
        for i in range(len(lane_boundaries)):  # 逐个对象处理
            line_object = lane_boundaries[i]
            if line_object['score'] == None or line_object['type'] == None or \
                line_object['track_id']['id'] == None :
                continue
        
            if line_object['score'] < 0.8:
                continue

            line_points = list([list(d.values()) for d in line_object['line_points']])  # 将线坐标转换为list
            if len(line_points) < 2:
                continue

            raw_bev_ids = []
            raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_LANE_LINE', line_object['color'], line_object['type'])
            raw_bev_ids.append(raw_bev_id)

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 1, # 用于跟踪使用
                "type": raw_bev_ids, # 车道线类型
                # "type": line_object['type'], # 车道类型
                # "color": line_object['color'], # 标线颜色
                "is_bold": line_object['width_type'], # 标线是否加粗
                "score": line_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = line_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "lane boundary"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_road_boundary_qzc(self, bev_frame_data):
        """将mmt的道路边界感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        road_boundaries = bev_frame_data['landmarks']['road_boundaries']
        for i in range(len(road_boundaries)):  # 逐个对象处理
            line_object = road_boundaries[i]
            if line_object['score'] == None or line_object['type'] == None or \
                line_object['track_id']['id'] == None :
                continue
        
            if line_object['score'] < 0.8:
                continue

            line_points = list([list(d.values()) for d in line_object['line_points']])  # 将线坐标转换为list
            if len(line_points) < 2:
                continue

            raw_bev_ids = []
            raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_BARRIER', line_object['type'], 0)
            raw_bev_ids.append(raw_bev_id)

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 4, # 用于跟踪使用
                # "type": line_object['type'], # 车道类型
                "type": raw_bev_ids, # 道路边界类型
                "score": line_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = line_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "road boundary"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_stopline_qzc(self, bev_frame_data):
        """将mmt的停止位置感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        stop_locations = bev_frame_data['landmarks']['stop_lines']
        for i in range(len(stop_locations)):  # 逐个对象处理
            line_object = stop_locations[i]

            if line_object['score'] == None or line_object['type'] == None or \
                line_object['track_id']['id'] == None :
                continue

            if line_object['score'] < 0.9:
                continue

            line_points = list([list(d.values()) for d in line_object['line_points']])  # 将线坐标转换为list
            if len(line_points) < 2:
                continue

            raw_bev_ids = []
            raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_OBJECT', 8, line_object['type'])
            raw_bev_ids.append(raw_bev_id)

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 2, # 用于跟踪使用
                # "type": line_object['type'], # 车道类型
                "type": raw_bev_ids, # 停止线类型
                "score": line_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = line_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "stop line"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_arrow_qzc(self, bev_frame_data):
        """将mmt的road mark位置感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        road_marks = bev_frame_data['landmarks']['road_marks']
        for i in range(len(road_marks)):  # 逐个对象处理
            polygon_object = road_marks[i]

            if polygon_object['score'] == None or polygon_object['types'] == None or \
                polygon_object['track_id']['id'] == None :
                continue

            if any(type_value > 7 for type_value in polygon_object['types']): # 过滤导流带
                continue
        
            polygon_points = list([list(d.values()) for d in polygon_object['polygon_points']])  # 将线坐标转换为list
            if len(polygon_points) < 3:
                continue
            
            raw_bev_ids = []
            for type_value in polygon_object['types']:
                raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_OBJECT', 7, type_value)
                raw_bev_ids.append(raw_bev_id)
            # if len(polygon_object['types']) > 1:
            #     print("箭头组合属性:", polygon_object['types'])

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 7, # 用于跟踪使用
                # "type": polygon_object['types'], # 车道类型
                "type": raw_bev_ids, # 箭头类型
                "score": polygon_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": polygon_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = polygon_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "road mark"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_cross_walk_qzc(self, bev_frame_data):
        """将mmt的cross walk位置感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        cross_walks = bev_frame_data['landmarks']['cross_walks']
        for i in range(len(cross_walks)):  # 逐个对象处理
            polygon_object = cross_walks[i]

            if polygon_object['score'] == None or \
                polygon_object['track_id']['id'] == None :
                continue

            if polygon_object['score'] < 0.9:
                continue

            polygon_points = list([list(d.values()) for d in polygon_object['center_line_points']])  # 将线坐标转换为list
            if len(polygon_points) < 3:
                continue
        
            raw_bev_ids = []
            raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_OBJECT', 10, 0)
            raw_bev_ids.append(raw_bev_id)

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 5, # 用于跟踪使用
                "type": raw_bev_ids, # 人行横道类型
                "score": polygon_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": polygon_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = polygon_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "cross_walk"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list
    
    def get_junction_qzc(self, bev_frame_data):
        """将mmt的junction位置感知结果（ddld-landmark.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        # 获取当前帧的感知结果
        aois = bev_frame_data['landmarks']['aois']
        for i in range(len(aois)):  # 逐个对象处理
            polygon_object = aois[i]

            if polygon_object['score'] == None or \
                polygon_object['type'] == None:
                continue
        
            polygon_points = list([list(d.values()) for d in polygon_object['polygon_points']])  # 将线坐标转换为list
            
            raw_bev_ids = []
            raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_JUNCTION', 1, 0)
            raw_bev_ids.append(raw_bev_id)

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 6, # 用于跟踪使用
                "type": raw_bev_ids, # 路口区域 junction
                "score": polygon_object['score'] # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": polygon_points,
                "3dbox": []
            }
            # tmp_bev_frame_dict2["track_id"] = polygon_object['track_id']['id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "junction"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_traffic_light_qzc(self, bev_frame_data):
        """将mmt的交通灯感知结果（_worldmodel_traffic_light.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []

        traffic_light_result = bev_frame_data.get('traffic_light_perception_result')
        if not traffic_light_result:
            return

        # 获取当前帧的感知结果
        traffic_lights = traffic_light_result.get('physical_traffic_light_single_frame', [])

        for point_object in traffic_lights:
            score = point_object.get('score')
            track_id = point_object.get('track_id')
            if score is None or track_id is None:
                continue

            points = []
            location_bv = point_object.get('location_bv', {})
            if isinstance(location_bv, dict):  # 检查是否为字典
                points.append(list(location_bv.values()))  # 将坐标dict转换为list

            raw_bev_ids = []
            raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_TRAFFICLIGHT', point_object['group_shape_type'], 0)
            raw_bev_ids.append(raw_bev_id)

            bulb_list = []
            for bulb_info in point_object.get('bulb', []):
                bulb = {
                    "attrs": {
                        "type":bulb_info.get('type'), # 灯泡类型
                        "pattern":bulb_info.get('pattern'),# 灯芯类型
                        "status": bulb_info.get('status'), # 灯芯状态
                        "onoff": bulb_info.get('onoff'), # ON/OFF状态
                    }
                }
                bulb_list.append(bulb)

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {                
                # "group_shape_type": point_object['group_shape_type'],
                "bulb_num": point_object.get('bulb_num', 0),
                "bulb": bulb_list,
                "heading_theta_bv": point_object['heading_theta_bv'],
                # "big_type": 0, # 用于跟踪使用
                "type": raw_bev_ids, # 交通灯类型
                "score": score # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": points,
                "3dbox": [] # TODO:qzc, yanxin need to fill this field
            }
            tmp_bev_frame_dict2["track_id"] = track_id  # 追踪id
            tmp_bev_frame_dict2["name"] = "traffic_light"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    def get_traffic_light_bulb_qzc(self, bev_frame_data):
        """将mmt的交通灯灯芯感知结果（_worldmodel_traffic_light.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        if 'traffic_light_perception_result' not in bev_frame_data:
            return

        # 获取当前帧的感知结果
        traffic_lights = bev_frame_data['traffic_light_perception_result']['physical_traffic_light_single_frame']
        for i in range(len(traffic_lights)):
            point_object = traffic_lights[i]
            if point_object['score'] == None or \
                point_object['track_id'] == None :
                continue
        
            points = list(point_object['location_bv'].values())  # 将坐标dict转换为list

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {                
                "bulb_pattern": point_object['bulb'][0]['pattern'],
                "bulb_status": point_object['bulb'][0]['status'],
                "bulb_onoff": point_object['bulb'][0]['onoff'],
                # "big_type": 0, # 用于跟踪使用
                "type": 82, # 车道类型
                "score": point_object['score'] # 置信度

            }
            tmp_bev_frame_dict2["data"] = {
                "points": [],
                "3dbox": [] # TODO:qzc, yanxin need to fill this field
            }
            tmp_bev_frame_dict2["track_id"] = point_object['track_id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "traffic_light_bulb"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list


class BevReaderByd(BevReader):
    def get_ins_dict_qzc(self, egopose_path, save_egopose_txt):
        """将mmt的egopose信息（mla-egopose.json）读取到dict中"""

        with open(egopose_path, 'r', encoding='utf-8') as file:
            egopose_data = json.load(file)

        save_lines = ""
        for egopose in egopose_data:
            rt_timestamp = float(egopose['meta']['timestamp_us'])
            rt_frame = egopose['meta']['seq']
            q_w = float(egopose['orientation']['quaternion_global']['w'])
            q_x = float(egopose['orientation']['quaternion_global']['x'])
            q_y = float(egopose['orientation']['quaternion_global']['y'])
            q_z = float(egopose['orientation']['quaternion_global']['z'])
            lat = float(egopose['position']['position_global']['latitude'])
            lon = float(egopose['position']['position_global']['longitude'])
            height = float(egopose['position']['position_global']['altitude'])

            one_pose = {
                "ts": rt_timestamp,
                "header":{
                    "frame_id": rt_frame,
                    "stamp": rt_timestamp
                },
                "position":{
                    "longitude": lon,
                    "latitude": lat,
                    "height": height,
                },
                "quaternion":{
                    "x": q_x,
                    "y": q_y,
                    "z": q_z,
                    "w": q_w,
                },
                "speed": 0,
                "position_type": 0
            }
            save_lines += json.dumps(one_pose)
            save_lines += '\n'

        with open(save_egopose_txt, 'w', encoding='utf-8') as out_file:
            out_file.writelines(save_lines)

    def get_egopose_dict_qzc(self, egopose_path, save_egopose_txt, use_opt_pose):
        """将mmt的egopose信息（mla-egopose.json）读取到dict中"""

        egopose_data = None
        with open(egopose_path, 'r', encoding='utf-8') as file:
            egopose_data = json.load(file)

        save_lines = ""
        for egopose in egopose_data:
            rt_timestamp = float(egopose['meta']['timestamp_us'])
            rt_frame = egopose['meta']['seq']
            if use_opt_pose: # local: enu
                q_w = float(egopose['orientation']['quaternion_local']['w'])
                q_x = float(egopose['orientation']['quaternion_local']['x'])
                q_y = float(egopose['orientation']['quaternion_local']['y'])
                q_z = float(egopose['orientation']['quaternion_local']['z'])
                x = float(egopose['position']['position_local']['x'])
                y = float(egopose['position']['position_local']['y'])
                z = float(egopose['position']['position_local']['z'])
            else:
                q_w = float(egopose['orientation']['quaternion_local']['w'])
                q_x = float(egopose['orientation']['quaternion_local']['x'])
                q_y = float(egopose['orientation']['quaternion_local']['y'])
                q_z = float(egopose['orientation']['quaternion_local']['z'])
                x = float(egopose['position']['position_local']['x'])
                y = float(egopose['position']['position_local']['y'])
                z = float(egopose['position']['position_local']['z'])

            one_pose = {
                "ts": rt_timestamp,
                "header":{
                    "frame_id": rt_frame,
                    "stamp": rt_timestamp
                },
                "position":{
                    "x": x,
                    "y": y,
                    "z": z,
                },
                "quaternion":{
                    "x": q_x,
                    "y": q_y,
                    "z": q_z,
                    "w": q_w,
                },
                "speed": 0
            }
            save_lines += json.dumps(one_pose)
            save_lines += '\n'

        with open(save_egopose_txt, 'w', encoding='utf-8') as out_file:
            out_file.writelines(save_lines)

        if len(egopose_data) == 0:
            print("failed read bev pose: ", egopose_path)
            return -1
        
        return 0
    
    def get_lane_sm_qzc(self, bev_frame_data):
        mse_score = 0.15
        # other_scripts/msg/environment_model_msgs/local_map_info.proto : message Lane
        # other_scripts/msg/environment_model_msgs/map_common.proto : enum LaneType
        """将 byd 的车道中心线感知结果（ _perception_detection_bev_lane_track.json ）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        if 'lanes' not in bev_frame_data:
            return bev_frame_list

        # 获取当前帧的感知结果
        lane_dict = {}
        split_ids = []
        merge_ids = []
        lane_centers = bev_frame_data['lanes']
        for i in range(len(lane_centers)):  # 逐个对象处理
            bev_obj = lane_centers[i]
            if bev_obj['track_id'] == None :
                continue
            lane_dict[bev_obj['track_id']] = bev_obj

            if bev_obj['connect_type'] == None:
                continue
            if bev_obj['connect_type'] == 'SPLIT':
                split_ids.append(bev_obj['track_id'])
            if bev_obj['connect_type'] == 'MERGE':
                merge_ids.append(bev_obj['track_id'])

        line_points = []
        for split_id in split_ids:
            if 'previous_lane_id' in lane_dict[split_id]:
                parent_lane_id = lane_dict[split_id]['previous_lane_id'][0]
                if parent_lane_id in lane_dict:
                    xy = lane_dict[parent_lane_id]['end_point']
                    line_points.append([xy['x'], xy['y'], 0])

        for merge_id in merge_ids:
            if 'previous_lane_id' in lane_dict[merge_id]:
                parent_lane_id = lane_dict[merge_id]['next_lane_id'][0]
                if parent_lane_id in lane_dict:
                    xy = lane_dict[parent_lane_id]['end_point']
                    line_points.append([xy['x'], xy['y'], 0])

        if len(line_points) > 0:
            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "type": bev_obj['lane_types'][0], # 车道类型
                # "big_type": 3, # 用于跟踪使用
                "type": [150], # 车道类型
                "score": 0 # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            # tmp_bev_frame_dict2["track_id"] = bev_obj['track_id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "split merge point"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    # 1. 车道中心线
    def get_lane_center_qzc(self, bev_frame_data):
        mse_score = 0.15
        # other_scripts/msg/environment_model_msgs/local_map_info.proto : message Lane
        # other_scripts/msg/environment_model_msgs/map_common.proto : enum LaneType
        """将 byd 的车道中心线感知结果（ _perception_detection_bev_lane_track.json ）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        if 'lanes' not in bev_frame_data:
            return bev_frame_list
    
        # 获取当前帧的感知结果
        lane_centers = bev_frame_data['lanes']
        for i in range(len(lane_centers)):  # 逐个对象处理
            bev_obj = lane_centers[i]
            if bev_obj['quality'] == None or bev_obj['type'] == None or \
                bev_obj['track_id'] == None :
                continue
            
            score = bev_obj['quality']*0.01
            if score > 1:
                continue
            # if score < 0.3:
            #     continue
        
            line_points_origin = list([list(d.values()) for d in bev_obj['polyline']])  # 将线坐标转换为list
            mses = list([d for d in bev_obj['mse']])  # 将线坐标转换为list
            pnt_size = len(line_points_origin)
            mse_size = len(mses)
            if pnt_size < 2 or pnt_size != mse_size:
                continue
            
            line_points = []
            for i in range(pnt_size):
                line_points_origin[i].append(0) # 增加 z 坐标
                if(mses[i] > mse_score):
                    continue
                line_points.append(line_points_origin[i])
            
            if len(line_points) < 2:
                continue
            
            raw_bev_ids = []
            type_value = get_lane_type(bev_obj['type'])
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_LANE_CENTER', 3, type_value.value)
            if raw_bev_id == None:
                # TODO:qzc ziyan 
                print("ELEMENT_LANE_CENTER cannot found:", type_value, " type: ", 3, " value: ", type_value.value) # LaneTypeByd.LANE_TYPE_BLOCKED
                continue
            raw_bev_ids.append(raw_bev_id)

            # for type_value in bev_obj['lane_types']:
            #     raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_LANE_CENTER', 3, type_value)
            #     raw_bev_ids.append(raw_bev_id)
            # if len(bev_obj['lane_types']) > 1:
            #     print("line_center组合属性:", bev_obj['lane_types'])

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "type": bev_obj['lane_types'][0], # 车道类型
                # "big_type": 3, # 用于跟踪使用
                "type": raw_bev_ids, # 车道类型
                "score": score # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = bev_obj['track_id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "center line"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list

    # 2. 道路边界线
    def get_road_boundary_qzc(self, bev_frame_data):
        """将 byd 的道路边界感知结果,读取到dict中"""
        mse_score = 0.15

        # other_scripts/msg/environment_model_msgs/local_map_info.proto : message Lane
        # other_scripts/msg/environment_model_msgs/map_common.proto : enum RoadEdgeType
        # 获取当前帧的frame和timestamp
        bev_frame_list = []

        # 获取当前帧的感知结果
        if 'road_edges' not in bev_frame_data:
            return bev_frame_list

        lane_centers = bev_frame_data['road_edges']
        for i in range(len(lane_centers)):  # 逐个对象处理
            bev_obj = lane_centers[i]
            if bev_obj['quality'] == None or bev_obj['type'] == None or \
                bev_obj['track_id'] == None :
                continue
            
            score = bev_obj['quality']*0.01
            if score > 1:
                continue
            # if score < 0.3:
            #     continue
        
            line_points_origin = list([list(d.values()) for d in bev_obj['polyline']])  # 将线坐标转换为list
            mses = list([d for d in bev_obj['mse']])  # 将线坐标转换为list
            pnt_size = len(line_points_origin)
            mse_size = len(mses)
            if pnt_size < 2 or pnt_size != mse_size:
                continue
            
            line_points = []
            for i in range(pnt_size):
                line_points_origin[i].append(0) # 增加 z 坐标
                if(mses[i] > mse_score):
                    continue
                line_points.append(line_points_origin[i])

            if len(line_points) < 2:
                continue

            raw_bev_ids = []
            type_value = get_road_edge_type(bev_obj['type'])
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_BARRIER', 0, type_value.value)
            if raw_bev_id == None:
                print("ELEMENT_BARRIER cannot found:",type_value, " type: ", 0, " value: ", type_value.value)
                continue
            raw_bev_ids.append(raw_bev_id)

            # for type_value in bev_obj['lane_types']:
            #     raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_BARRIER', 0, type_value)
            #     raw_bev_ids.append(raw_bev_id)
            # if len(bev_obj['lane_types']) > 1:
            #     print("line_center组合属性:", bev_obj['lane_types'])

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {
                # "big_type": 4, # 用于跟踪使用
                # "type": bev_obj['type'], # 车道类型
                "type": raw_bev_ids, # 道路边界类型
                "score": score # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": line_points,
                "3dbox": []
            }
            tmp_bev_frame_dict2["track_id"] = bev_obj['track_id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "road boundary"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list
    
    # 3. 路面其他要素都在这里分发读取
    def get_lane_marker_qzc(self, bev_frame_data, data_type: str):
        bev_frame_list = []

        if 'lane_markers' not in bev_frame_data:
            return bev_frame_list

        lane_markers = bev_frame_data['lane_markers']
        for i in range(len(lane_markers)):
            bev_obj = lane_markers[i]

            obj_type = get_lane_marker_type(bev_obj['type'])
            if obj_type == LaneMarkerTypeByd.LM_TYPE_UNKNOWN:
                continue
            
            if obj_type in (LaneMarkerTypeByd.LM_TYPE_SOLID, LaneMarkerTypeByd.LM_TYPE_DASHED, \
                            LaneMarkerTypeByd.LM_TYPE_DASHED_SOLID, LaneMarkerTypeByd.LM_TYPE_SOLID_DASHED, \
                            LaneMarkerTypeByd.LM_TYPE_DOUBLE_DASHED, LaneMarkerTypeByd.LM_TYPE_DOUBLE_SOLID, \
                            LaneMarkerTypeByd.LM_TYPE_WIDEDASHED,
                            ):
                new_obj = self.get_lane_boundary_qzc(bev_obj)
                if new_obj != None:
                    bev_frame_list.append(new_obj)
            elif obj_type in (LaneMarkerTypeByd.LM_TYPE_FISHBONE,):
                continue
            elif obj_type in (LaneMarkerTypeByd.RM_TYPE_CROSSWALK,):
                if data_type != DataType.BYD_LIDAR_BEV_B.name:
                    new_obj = self.get_cross_walk_qzc(bev_obj)
                    if new_obj != None:
                        bev_frame_list.append(new_obj)
            elif obj_type in (LaneMarkerTypeByd.RM_TYPE_STRAIGHT,LaneMarkerTypeByd.RM_TYPE_LEFT,\
                              LaneMarkerTypeByd.RM_TYPE_RIGHT, LaneMarkerTypeByd.RM_TYPE_TURNING, \
                              LaneMarkerTypeByd.RM_TYPE_STRAIGHT_LEFT, LaneMarkerTypeByd.RM_TYPE_STRAIGHT_RIGHT, \
                              LaneMarkerTypeByd.RM_TYPE_STRAIGHT_LEFT_RIGHT, LaneMarkerTypeByd.RM_TYPE_LEFT_RIGHT, \
                              LaneMarkerTypeByd.RM_TYPE_STRAIGHT_TURNING, LaneMarkerTypeByd.RM_TYPE_LEFT_TURNING, \
                              ):
                if data_type != DataType.BYD_LIDAR_BEV_B.name:
                    new_obj = self.get_arrow_qzc(bev_obj)
                    if new_obj != None:
                        bev_frame_list.append(new_obj)
            elif obj_type in (LaneMarkerTypeByd.RM_TYPE_STOPLINE,):
                if data_type != DataType.BYD_LIDAR_BEV_B.name:
                    new_obj = self.get_stopline_qzc(bev_obj)
                    if new_obj != None:
                        bev_frame_list.append(new_obj)
            else:
                continue

        return bev_frame_list
    
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

    def get_lane_boundary_qzc(self, bev_obj):
        """将 byd 的车道边界感知结果（_perception_detection_bev_lane_track.json）读取到dict中"""
        mse_score = 0.15

        obj_type = get_lane_marker_type(bev_obj['type'])
        score = bev_obj['quality'] * 0.01
        color = get_lane_color(bev_obj['color'])
        track_id = bev_obj['track_id']
        if score == None or obj_type == None or track_id == None :
            return None

        # if track_id == "41" and score == 0.88:
        #     print("qzc")

        if score > 1:
            return None
        # if score < 0.3:
        #     return None

        if 'segment_visibility' not in bev_obj or 'type_seg' not in bev_obj:
            return None
        segment_visibility = bev_obj['segment_visibility']
        type_seg = bev_obj['type_seg']
        polyline = bev_obj['polyline']
        if (all(segment_visibility) == False):
            # method 1
            # min_index = -1
            # max_index = len(polyline)
            # for i in range(len(segment_visibility)):
            #     if segment_visibility[i]:
            #         min_index = type_seg[i]['start_index']
            #         max_index = type_seg[i]['end_index']
            #     else:
            #         break
            # if min_index == -1 or max_index == len(polyline):
            #     return None
            # polyline = polyline[:max_index+1]

            # method2:
            line_points_origin = np.array([np.array(list(d.values())) for d in polyline])  # 将线坐标转换为list
            curvature = self.compute_curvature(line_points_origin[:,0], line_points_origin[:,1])
            if np.any(curvature > 0.02):
                return None

        # if (all(segment_visibility) == False):
        #     return None

        line_points_origin = list([list(d.values()) for d in polyline])  # 将线坐标转换为list
        mses = list([d for d in bev_obj['mse']])  # 将线坐标转换为list
        pnt_size = len(line_points_origin)
        mse_size = len(mses)
        if pnt_size < 2 or pnt_size != mse_size:
            return None
        
        line_points = []
        for i in range(pnt_size):
            line_points_origin[i].append(0) # 增加 z 坐标
            if(mses[i] > mse_score):
                continue
            line_points.append(line_points_origin[i])

        if len(line_points) < 2:
            return None
        
        raw_bev_ids = []
        raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_LANE_LINE', color.value, obj_type.value)
        if raw_bev_id == None:
            print("ELEMENT_LANE_LINE cannot found:",obj_type, " color: ", color.value, " type: ", obj_type.value)
            return None
        raw_bev_ids.append(raw_bev_id)
        
        tmp_bev_frame_dict2 = defaultdict(list)
        tmp_bev_frame_dict2["attrs"] = {
            # "big_type": 1, # 用于跟踪使用
            "type": raw_bev_ids, # 车道线类型
            # "type": bev_obj['type'], # 车道类型
            # "color": bev_obj['color'], # 标线颜色
            "is_bold": 1, # 标线是否加粗, 0:未知，1：0.15m，2：0.45m
            "score": score # 置信度
        }
        tmp_bev_frame_dict2["data"] = {
            "points": line_points,
            "3dbox": []
        }
        tmp_bev_frame_dict2["track_id"] = track_id  # 追踪id
        tmp_bev_frame_dict2["name"] = "lane boundary"

        return tmp_bev_frame_dict2

    def get_stopline_qzc(self, bev_obj):
        """将 byd 的停止位置感知结果, 读取到dict中"""

        obj_type = get_lane_marker_type(bev_obj['type'])
        score = bev_obj['quality'] * 0.01
        track_id = bev_obj['track_id']
        if score == None or obj_type == None or track_id == None :
            return None

        # if score > 1:
        #     return None
        # if score < 0.5:
        #     return None
             
        line_points = list([list(d.values()) for d in bev_obj['polyline']])  # 将线坐标转换为list
        if len(line_points) < 2:
            return None
        for i in range(len(line_points)):
            line_points[i].append(0)
        
        raw_bev_ids = []
        raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 8, obj_type.value)
        if raw_bev_id == None:
            print("ELEMENT_OBJECT cannot found:",obj_type, "type: ", 8, " value: ", obj_type.value)
            return None
        raw_bev_ids.append(raw_bev_id)

        tmp_bev_frame_dict2 = defaultdict(list)
        tmp_bev_frame_dict2["attrs"] = {
            # "big_type": 5, # 用于跟踪使用
            "type": raw_bev_ids, # 人行横道类型
            "score": score # 置信度
        }
        tmp_bev_frame_dict2["data"] = {
            "points": line_points,
            "3dbox": []
        }
        tmp_bev_frame_dict2["track_id"] = track_id  # 追踪id
        tmp_bev_frame_dict2["name"] = "stop line"

        return tmp_bev_frame_dict2

    def get_arrow_qzc(self, bev_obj):
        """将 byd 的road mark位置感知结果, 读取到dict中"""

        obj_type = get_lane_marker_type(bev_obj['type'])
        score = bev_obj['quality'] * 0.01
        track_id = bev_obj['track_id']
        if score == None or obj_type == None or track_id == None :
            return None

        # if score > 1:
        #     return None
        if score < 0.3:
            return None
         
        line_points = list([list(d.values()) for d in bev_obj['polyline']])  # 将线坐标转换为list
        if len(line_points) < 2:
            return None
        
        line_poly = []
        for i in range(len(line_points)):
            line_points[i].append(0)
            line_poly.append((line_points[i][0], line_points[i][1]))
        
        line_poly = Polygon(line_poly)
        # rect_angle = get_angle(line_poly)
        # if math.fabs(rect_angle-180) > 10 :
        #     return None
        
        length, width, area = get_width_length(line_poly)
        if length > 0 and width > 0  and (
            width / length > 0.6 or 
            (length < 3 and width < 3) or
            area < 1.0 # min: 1.35
            ):
            return None
        
        raw_bev_ids = []
        # RM_TYPE_STRAIGHT = 11;
        # RM_TYPE_LEFT = 12;
        # RM_TYPE_RIGHT = 13;
        # RM_TYPE_TURNING = 14; LaneMarkerTypeByd.RM_TYPE_TURNING
        # RM_TYPE_STRAIGHT_LEFT = 15;
        # RM_TYPE_STRAIGHT_RIGHT = 16;
        # RM_TYPE_STRAIGHT_LEFT_RIGHT = 17;
        # RM_TYPE_LEFT_RIGHT = 18;
        # RM_TYPE_STRAIGHT_TURNING = 19;
        # RM_TYPE_LEFT_TURNING = 20;
        if obj_type == LaneMarkerTypeByd.RM_TYPE_STRAIGHT_LEFT:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_STRAIGHT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_LEFT.value)
            raw_bev_ids.append(raw_bev_id)
        elif obj_type == LaneMarkerTypeByd.RM_TYPE_STRAIGHT_RIGHT:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_STRAIGHT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_RIGHT.value)
            raw_bev_ids.append(raw_bev_id)
        elif obj_type == LaneMarkerTypeByd.RM_TYPE_STRAIGHT_LEFT_RIGHT:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_STRAIGHT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_LEFT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_RIGHT.value)
            raw_bev_ids.append(raw_bev_id)
        elif obj_type == LaneMarkerTypeByd.RM_TYPE_LEFT_RIGHT:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_LEFT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_RIGHT.value)
            raw_bev_ids.append(raw_bev_id)
        elif obj_type == LaneMarkerTypeByd.RM_TYPE_STRAIGHT_TURNING:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_STRAIGHT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_TURNING.value)
            raw_bev_ids.append(raw_bev_id)
        elif obj_type == LaneMarkerTypeByd.RM_TYPE_LEFT_TURNING:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_LEFT.value)
            raw_bev_ids.append(raw_bev_id)
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, LaneMarkerTypeByd.RM_TYPE_TURNING.value)
            raw_bev_ids.append(raw_bev_id)
        else:
            raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, obj_type.value)
            if raw_bev_id == None:
                print("ELEMENT_OBJECT2 cannot found:",obj_type, " value: ", obj_type.value)
                return None
            raw_bev_ids.append(raw_bev_id)
        # raw_bev_ids = []
        # for type_value in obj_type:
        #     raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 7, type_value)
        #     raw_bev_ids.append(raw_bev_id)
        # # if len(polygon_object['types']) > 1:
        # #     print("箭头组合属性:", polygon_object['types'])

        tmp_bev_frame_dict2 = defaultdict(list)
        tmp_bev_frame_dict2["attrs"] = {
            # "big_type": 5, # 用于跟踪使用
            "type": raw_bev_ids, # 人行横道类型
            "score": score # 置信度
        }
        tmp_bev_frame_dict2["data"] = {
            "points": line_points,
            "3dbox": []
        }
        tmp_bev_frame_dict2["track_id"] = track_id  # 追踪id
        tmp_bev_frame_dict2["name"] = "road mark"

        return tmp_bev_frame_dict2
    

    def get_cross_walk_qzc(self, bev_obj):
        """将 byd 的cross walk位置感知结果, 读取到dict中"""

        obj_type = get_lane_marker_type(bev_obj['type'])
        score = bev_obj['quality'] * 0.01
        track_id = bev_obj['track_id']
        if score == None or obj_type == None or track_id == None :
            return None

        # if score > 1:
        #     return None
        if score < 0.3:
            return None
        
        line_points = list([list(d.values()) for d in bev_obj['polyline']])  # 将线坐标转换为list
        if len(line_points) < 2:
            return None
        
        line_poly = []
        for i in range(len(line_points)):
            line_points[i].append(0)
            line_poly.append((line_points[i][0], line_points[i][1]))
        
        line_poly = Polygon(line_poly)

        # rect_angle = get_angle(line_poly)
        # if math.fabs(rect_angle-90) > 10: # TODO:qzc right turn
        #     return None

        length, width, area = get_width_length(line_poly)
        if length > 0 and width > 0  and (
            # width / length > 0.6 or 
            (length < 3 and width < 3) or
            area < 1.3
            ):
            return None

        raw_bev_ids = []
        raw_bev_id = get_raw_bev_id(g_type_map_byd, 'ELEMENT_OBJECT', 10, 0)
        raw_bev_ids.append(raw_bev_id)

        tmp_bev_frame_dict2 = defaultdict(list)
        tmp_bev_frame_dict2["attrs"] = {
            # "big_type": 5, # 用于跟踪使用
            "type": raw_bev_ids, # 人行横道类型
            "score": score # 置信度
        }
        tmp_bev_frame_dict2["data"] = {
            "points": line_points,
            "3dbox": []
        }
        tmp_bev_frame_dict2["track_id"] = track_id  # 追踪id
        tmp_bev_frame_dict2["name"] = "cross_walk"

        return tmp_bev_frame_dict2
    
    def get_junction_qzc(self, bev_frame_data):
        pass

    def get_traffic_light_qzc(self, bev_frame_data):
        """将 byd 的交通灯感知结果（_worldmodel_traffic_light.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []

        objects_result = bev_frame_data.get('objects')
        if not objects_result:
            return bev_frame_list
        
        # 获取当前帧的感知结果
        for point_object in objects_result:
            score = point_object.get('exist_score', 0)   #实际没用到
            track_id = point_object.get('id', 0)  #实际没用到
            object_type = point_object.get('type')
            if object_type != 'TRAFFIC_LIGHT':
                continue
            # if score is None or track_id is None:
            #     continue

            points = []
            location_bv = point_object.get('position', {})
            if isinstance(location_bv, dict):  # 检查是否为字典
                points.append(list(location_bv.values()))  # 将坐标dict转换为list
            # print(f"point1: {points}")

            # traffic_value = get_trafficLight_direction_type(point_object['type'])
            
            #TODO: 这里不确定
            raw_bev_ids = []
            # raw_bev_id = get_raw_bev_id(g_type_map_mmt_rc, 'ELEMENT_TRAFFICLIGHT', point_object['group_shape_type'], 0)  #实际没用到
            raw_bev_id = 79
            raw_bev_ids.append(raw_bev_id)  # 实际后面没用到

            bulb_list = []
            for bulb_info in point_object.get('bulb', []):
                bulb = {
                    "attrs": {
                        "type":bulb_info.get('type', ''), # 灯泡类型       #实际没用到
                        "pattern":bulb_info.get('pattern', ['']),# 灯芯类型     #实际没用到
                        "status": bulb_info.get('status', ['']), # 灯芯状态     #实际没用到
                        "onoff": bulb_info.get('onoff', ['']), # ON/OFF状态   #实际没用到
                    }
                }
                bulb_list.append(bulb)

            angle = 0
            direction = point_object['attributes'].get('traffic_light_direction', '')
            if direction == 'TLD_RIGHT':
                angle = 90/180* math.pi
            elif direction == 'TLD_LEFT':
                angle = 270/180* math.pi
            elif direction == 'TLD_UP':
                angle = 0
            elif direction == 'TLD_DOWN':
                angle = 180/180* math.pi

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {                
                "bulb_num": point_object['attributes'].get('traffic_light_num', 0),
                "bulb": bulb_list,
                "heading_theta_bv": angle ,
                # "heading_theta_bv": point_object['imu_box']['heading'],
                # "big_type": 0, # 用于跟踪使用
                "type": raw_bev_ids, # 交通灯类型 # 没用到
                "score": score # 置信度
            }
            tmp_bev_frame_dict2["data"] = {
                "points": points,
                "3dbox": [] # TODO:qzc, yanxin need to fill this field
            }
            tmp_bev_frame_dict2["track_id"] = track_id  # 追踪id
            tmp_bev_frame_dict2["name"] = "traffic_light"

            bev_frame_list.append(tmp_bev_frame_dict2)

            #TODO:yanxin 用上朝向 和颜色等信息，输出shp

        return bev_frame_list

    def get_traffic_light_bulb_qzc(self, bev_frame_data):
        """将mmt的交通灯灯芯感知结果（_worldmodel_traffic_light.json）读取到dict中"""
        # 获取当前帧的frame和timestamp
        bev_frame_list = []
        # bev_frame_dict['bev_frame'], bev_frame_dict['bev_timestamp'] = _get_frame_par(bev_frame_data)

        if 'traffic_light_perception_result' not in bev_frame_data:
            return

        # 获取当前帧的感知结果
        traffic_lights = bev_frame_data['traffic_light_perception_result']['physical_traffic_light_single_frame']
        for i in range(len(traffic_lights)):
            point_object = traffic_lights[i]
            if point_object['score'] == None or \
                point_object['track_id'] == None :
                continue
        
            points = list(point_object['location_bv'].values())  # 将坐标dict转换为list

            tmp_bev_frame_dict2 = defaultdict(list)
            tmp_bev_frame_dict2["attrs"] = {                
                "bulb_pattern": point_object['bulb'][0]['pattern'],
                "bulb_status": point_object['bulb'][0]['status'],
                "bulb_onoff": point_object['bulb'][0]['onoff'],
                # "big_type": 0, # 用于跟踪使用
                "type": 82, # 车道类型
                "score": point_object['score'] # 置信度

            }
            tmp_bev_frame_dict2["data"] = {
                "points": [],
                "3dbox": [] # TODO:qzc, yanxin need to fill this field
            }
            tmp_bev_frame_dict2["track_id"] = point_object['track_id']  # 追踪id
            tmp_bev_frame_dict2["name"] = "traffic_light_bulb"

            bev_frame_list.append(tmp_bev_frame_dict2)

        return bev_frame_list



if __name__ == '__main__':
    bev_reader = BevReader()

    # 车道边界、道路边界、车道中心线、路面标识、人行横道、停止位置
    landmark_path = r'E:\to_kuandeng\20241107\to_kd_sample_3\PL0681_event_trafficlight_status_event_20240924-170447_0\_ddld_landmark.json'
    # with open(landmark_path, 'r', encoding='utf-8') as file:
    #     bev_data = json.load(file)
    # for bev_frame_data in bev_data:
    #     bev_reader.get_cross_walk(bev_frame_data)

    # 交通灯
    tl_path = r'E:\to_kuandeng\20241107\to_kd_sample_3\PL0681_event_trafficlight_status_event_20240924-170447_0\_worldmodel_traffic_light.json'
    with open(tl_path, 'r', encoding='utf-8') as file:
        bev_data = json.load(file)
    for bev_frame_data in bev_data:
        bev_reader.get_traffic_light_bulb(bev_frame_data)
