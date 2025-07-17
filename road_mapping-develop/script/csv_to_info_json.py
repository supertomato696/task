'''
Description:
Author: ccj,qzc
Date: 2024-12-09 20:00:00
Reference:
'''
import sys,os
project_root_folder = os.path.abspath(__file__+"/../../")
sys.path.append(project_root_folder)


import json
import argparse
# import geopandas as gpd
import pandas as pd

from data_io.script.util import lonlat_to_utm_num, lonlatalt_utm_num_to_utm_xyz, gcj2wgs, wgs2gcj
import collections
from itertools import product
import numpy as np
from shapely.geometry import Polygon
import pyproj
import shapely.wkt
from shapely.geometry import Polygon, LineString, GeometryCollection
from common.util.util import *

# from util import *
from preprocess.script.util import log
from data_io.script.bev_label_map import DataType
from common.data.error_code import ErrorCode

def read_csv(file_path):
    return pd.read_csv(file_path, dtype={'lng':str, 'lat':str, 'prj_lng':str, 'prj_lat':str})

def load_json_file(file_path):
    """加载 JSON 文件"""
    with open(file_path, 'r', encoding='utf-8') as file:
        return json.load(file)

def default_dump(obj):
    """Convert numpy classes to JSON serializable objects."""
    if isinstance(obj, (np.integer, np.floating, np.bool_)):
        return obj.item()
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    else:
        return obj
    
def save_json_file(file_path, data):
    """保存 JSON 数据到文件"""
    with open(file_path, 'w', encoding='utf-8') as file:
        json.dump(data, file, ensure_ascii=False, indent=4, default=default_dump)
        # json.dump(data, file, ensure_ascii=False, indent=4)

"""
origin_utm_num: 
utm_x:
utm_y:
geom_poly_wkt: gcj
"""
def expand_polygon(origin_utm_num, utm_x, utm_y, geom_poly_wkt, buffer_len=20):
    # gcj -> utm
    proj_transform = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:326{origin_utm_num}", always_xy=True).transform
    geom_poly_wkt_utm_1 = shapely.ops.transform(proj_transform, geom_poly_wkt)
    geom_poly_wkt_utm = shapely.affinity.translate(geom_poly_wkt_utm_1, -utm_x, -utm_y)

    # expand 50
    geom_poly_wkt_utm_buffer = geom_poly_wkt_utm.buffer(buffer_len)

    # utm -> gcj
    proj_transform_inverse = pyproj.Transformer.from_crs(f"EPSG:326{origin_utm_num}", "EPSG:4326", always_xy=True).transform
    geom_poly_wkt_utm_buffer_1 = shapely.affinity.translate(geom_poly_wkt_utm_buffer, utm_x, utm_y)
    geom_poly_wkt_buffer = shapely.ops.transform(proj_transform_inverse, geom_poly_wkt_utm_buffer_1)

    return geom_poly_wkt_utm_buffer, geom_poly_wkt_buffer

def generate_task_geom_and_site_center(data_folder, task_json_data, template):
    origin_utm_num = template['utm_num']
    t_utm_world = template['t_utm_world']
    proj_transform = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:326{origin_utm_num}", always_xy=True).transform

    polygon_global = task_json_data['polygon']
    if polygon_global != "":
        template['middle']['task_geom_global'] = polygon_global
        
        polygon_global_utm = shapely.ops.transform(proj_transform, shapely.wkt.loads(polygon_global))
        polygon_local = shapely.affinity.translate(polygon_global_utm, -t_utm_world[0], -t_utm_world[1])
        template['middle']['task_geom_local'] = polygon_local.wkt

        buffer_len = 20
        geom_poly_wkt_utm_buffer, geom_poly_wkt_buffer = expand_polygon(origin_utm_num, t_utm_world[0], t_utm_world[1], shapely.wkt.loads(polygon_global), buffer_len)
        template['middle']['task_geom_global_expand'] = geom_poly_wkt_buffer.wkt
        template['middle']['task_geom_local_expand'] = geom_poly_wkt_utm_buffer.wkt

        try:
            # shapely_to_obj(all_link_world_ls, os.path.join(task_arg.task_folder, f"debug/all_link_geom.obj"))
            task_folder = os.path.join(data_folder, "auto_label/output")
            shapely_to_obj(polygon_local, os.path.join(task_folder, f"debug/task_geom.obj"))
            shapely_to_obj(polygon_local, os.path.join(task_folder, f"debug/data_scope_origin.obj"))
            shapely_to_obj(geom_poly_wkt_utm_buffer, os.path.join(task_folder, f"debug/data_scope_buffer20.obj"))
            for one_link in template["middle"]["links"]:
                link_utm_ls = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["link_geom"]))
                link_world_ls = shapely.affinity.translate(link_utm_ls, -t_utm_world[0], -t_utm_world[1])
                shapely_to_obj(link_world_ls, os.path.join(task_folder, f"debug/link-{one_link['link_id']}.obj"))
        except Exception as e:
            log.error("!!!! cannot save: {} !!!!!!".format(e))
    else:
        task_utm_poly: Polygon = Polygon()
        if ("task_geom" in template["middle"].keys()):  # 对于有 task_geom 字段的，使用 task_geom 字段作为任务范围
            task_utm_poly = shapely.ops.transform(proj_transform, shapely.wkt.loads(template["middle"]["task_geom"]))
        else:  # 对于有 links 字段的，使用 links 范围合成任务范围
            for one_link in template["middle"]["links"]:
                new_link_polygon = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["task_geom"])).buffer(0.01)
                task_utm_poly = task_utm_poly.union(new_link_polygon)
        task_utm_poly = task_utm_poly.convex_hull
        task_world_poly = shapely.affinity.translate(task_utm_poly, -t_utm_world[0], -t_utm_world[1])

        all_link_world_ls = LineString()
        for one_link in template["middle"]["links"]:
            link_utm_ls = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["link_geom"]))
            link_world_ls = shapely.affinity.translate(link_utm_ls, -t_utm_world[0], -t_utm_world[1])
            all_link_world_ls = all_link_world_ls.union(link_world_ls)

        all_link_in_task_world_ls = all_link_world_ls.intersection(task_world_poly)


        proj_transform_inverse = pyproj.Transformer.from_crs(f"EPSG:326{origin_utm_num}", "EPSG:4326", always_xy=True).transform
        all_link_in_task_world_ls2 = shapely.affinity.translate(all_link_in_task_world_ls.buffer(50), t_utm_world[0], t_utm_world[1])
        all_link_in_task_world_ls_inverse = shapely.ops.transform(proj_transform_inverse, all_link_in_task_world_ls2)

        task_geom_global = all_link_in_task_world_ls_inverse.wkt # gcj02, "POLYGON ((xxxx))"
        task_geom_local = all_link_in_task_world_ls.buffer(50).wkt # utm local, "POLYGON ((xxxx))"

        def convert_poly_2_multipoly(task_geom_in):
            task_geom_out = "MULTIPOLYGON ((("
            cur_links1 = shapely.wkt.loads(task_geom_in)
            cur_links = shapely.geometry.mapping(cur_links1)
            for wgs in cur_links['coordinates'][0]:
                task_geom_out = task_geom_out + str(wgs[0]) + " " + str(wgs[1]) + ","
            task_geom_out = task_geom_out[:-1] + ")))"
            return task_geom_out
        
        # info:task_geom_global, gcj02
        template["middle"]["task_geom_global"] = convert_poly_2_multipoly(task_geom_global) 
        template["middle"]["task_geom_local"] = convert_poly_2_multipoly(task_geom_local) 

        template['middle']['task_geom_global_expand'] = template["middle"]["task_geom_global"]
        template['middle']['task_geom_local_expand'] = template["middle"]["task_geom_local"]

        try:
            # shapely_to_obj(all_link_world_ls, os.path.join(task_arg.task_folder, f"debug/all_link_geom.obj"))
            task_folder = os.path.join(data_folder, "auto_label/output")
            shapely_to_obj(task_world_poly, os.path.join(task_folder, f"debug/task_geom.obj"))
            shapely_to_obj(all_link_in_task_world_ls, os.path.join(task_folder, f"debug/data_scope_origin.obj"))
            shapely_to_obj(all_link_in_task_world_ls.buffer(50), os.path.join(task_folder, f"debug/data_scope_buffer50.obj"))
            for one_link in template["middle"]["links"]:
                link_utm_ls = shapely.ops.transform(proj_transform, shapely.wkt.loads(one_link["link_geom"]))
                link_world_ls = shapely.affinity.translate(link_utm_ls, -t_utm_world[0], -t_utm_world[1])
                shapely_to_obj(link_world_ls, os.path.join(task_folder, f"debug/link-{one_link['link_id']}.obj"))
        except Exception as e:
            log.error("!!!! cannot save: {} !!!!!!".format(e))

    # 读取site_center，并将其转换到utm坐标系
    if len(task_json_data['site_center']) > 0 and task_json_data['site_center'][0] != '':
        template['middle']['site_center_gcj02'] = task_json_data['site_center'][0]
        
        siter_center_global = shapely.wkt.loads(task_json_data['site_center'][0])
        siter_center_gloabl_utm = shapely.ops.transform(proj_transform, siter_center_global)
        siter_center_local = shapely.affinity.translate(siter_center_gloabl_utm, -t_utm_world[0], -t_utm_world[1])

        template['middle']['site_center'] = list(siter_center_local.coords[0])
        print("adjusted site_center:", template['middle']['site_center'])
    else :
        cur_links1 = shapely.wkt.loads(template["middle"]["task_geom_local"])

        # 计算几何中心
        centroid = cur_links1.centroid
        template["middle"]["site_center"] = [centroid.x, centroid.y]
        print(f"几何中心点坐标: {template['middle']['site_center']}")

# 读取 sd node
def read_sd_node(data_folder, template):
    origin_utm_num = template['utm_num']
    t_utm_world = template['t_utm_world']
    proj_transform = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:326{origin_utm_num}", always_xy=True).transform
    data_scope_wkt = template['middle']['task_geom_global_expand']
    data_scope_global_poly = shapely.wkt.loads(data_scope_wkt)
    data_scope_wkt = template['middle']['task_geom_local_expand']
    data_scope_local_poly = shapely.wkt.loads(data_scope_wkt)

    sd_json_path = os.path.join(data_folder, "auto_label/input/sd_node.geojson")
    with open(sd_json_path,'r',encoding='utf8')as fp:
        sd_json_data = json.load(fp)
    fp.close()

    m_sd_id_to_sd_info_all = collections.OrderedDict()
    inner_link_node = []
    for node in sd_json_data['features']:
        cross_flag = node["properties"]["cross_flag"]
        node_id = node["properties"]["node_id"]
        main_node_id = node["properties"]["main_node_id"]
        sub_node_id = node["properties"]["sub_node_id"].split(',')
        node_link_id = node["properties"]["node_link_id"].split(',')
        cross_link_id = node["properties"]["cross_link_id"].split(',')
        coordinates = node["geometry"]["coordinates"]

        point_wkt = "Point ("+str(coordinates[0]) + " " + str(coordinates[1]) + ")"
        point = shapely.wkt.loads(point_wkt)
        if data_scope_global_poly.contains(point):
            m_sd_id_to_sd_info_all[node_id] = {
                                                "cross_flag": cross_flag,
                                                "node_id": node_id,
                                                "main_node_id": main_node_id,
                                                "sub_node_id": sub_node_id,
                                                "node_link_id": node_link_id,
                                                "cross_link_id": cross_link_id,
                                                "geom": coordinates,
                                            }

    nodes = []
    # nodes_template = template.get('middle', {}).get('nodes', [{}])[0]
    for node_id, node_info in m_sd_id_to_sd_info_all.items():
        # link = nodes_template.copy()
        one_node = {}
        one_node['cross_flag'] = node_info['cross_flag']
        one_node['node_id'] = node_info['node_id']
        one_node['main_node_id'] = node_info['main_node_id']
        one_node['sub_node_id'] = node_info['sub_node_id']
        one_node['node_link_id'] = node_info['node_link_id']
        one_node['cross_link_id'] = node_info['cross_link_id']
        one_node['geom'] = node_info['geom']
        nodes.append(one_node)
    template['middle']['nodes'] = nodes

    # # 如果有 task_geom_global_expand 则读取该字段
    # # 如果没有， 则按照中心点
    # for one_link in template["middle"]["links"]:
    #     start_node_id = one_link["start_node_id"]
    #     end_node_id = one_link["end_node_id"]

def calculate_angle(lon1, lat1, lon2, lat2):
    """
    计算从点1到点2的方位角（角度制），范围为-180到180度。
    
    参数:
    point1: 第一个点的UTM坐标 (x1, y1)
    point2: 第二个点的UTM坐标 (x2, y2)
    
    返回:
    float: 方位角，东为0度，逆时针为正，顺时针为负
    """
    origin_utm_num1 = lonlat_to_utm_num(lon1, lat1)
    [utm_x1, utm_y1, _] = lonlatalt_utm_num_to_utm_xyz(lon1, lat1, 0, origin_utm_num1)

    origin_utm_num2 = lonlat_to_utm_num(lon2, lat2)
    [utm_x2, utm_y2, _] = lonlatalt_utm_num_to_utm_xyz(lon2, lat2, 0, origin_utm_num2)

    x1, y1 = utm_x1, utm_y1
    x2, y2 = utm_x2, utm_y2
    
    # 计算坐标差值
    dx = x2 - x1
    dy = y2 - y1
    
    # 计算方位角（弧度）
    angle_rad = math.atan2(dx, dy)  # atan2(dx, dy) 计算从正北方向逆时针旋转的角度
    
    # 将弧度转换为角度
    angle_deg = math.degrees(angle_rad)
    
    # 确保角度在-180到180度之间
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg <= -180:
        angle_deg += 360
    
    return angle_deg

def refine_no_trajs_link(m_link_id_to_link_info_all, have_trajs_link_id, have_no_trajs_link_id, node_map, link_dir_angle):
    # print("----------------", have_trajs_link_id, have_no_trajs_link_id, node_map, link_dir_angle)

    if have_no_trajs_link_id is None:
        return None

    cluster_links = []
    start_node_index_links, end_node_index_links = node_map[0], node_map[1]
    hash_table = set([])
    for no_trajs_link in have_no_trajs_link_id:
        if no_trajs_link in hash_table:
            continue

        cluster_link = collections.deque([no_trajs_link])
        find_front, find_back = False, False
        total_length = m_link_id_to_link_info_all[no_trajs_link]["link_length"]
        length_threshold = 50.0
        
        # 向前搜索
        index_link = no_trajs_link
        while True:
            index_node = m_link_id_to_link_info_all[index_link]["start_node_id"] if m_link_id_to_link_info_all[index_link]["link_direction"] in [1, 3] \
                    else m_link_id_to_link_info_all[index_link]["end_node_id"]
            if not end_node_index_links[index_node] or len(end_node_index_links[index_node]) > 1:
                break

            cur_link_id = end_node_index_links[index_node][0]
            if cur_link_id not in have_no_trajs_link_id:
                if cur_link_id in have_trajs_link_id:
                    prev_dirs = link_dir_angle[cur_link_id]
                    cur_dirs = link_dir_angle[index_link]
                    for prev_dir, cur_dir in product(prev_dirs, cur_dirs):
                        if abs(prev_dir - cur_dir) < 10:
                            cluster_link.appendleft(cur_link_id)
                            find_front = True
                            break
                break
            
            else:  # 前端link也是未有轨迹覆盖
                prev_dirs = link_dir_angle[cur_link_id]
                cur_dirs = link_dir_angle[index_link]
                is_same_dir = False
                for prev_dir, cur_dir in product(prev_dirs, cur_dirs):
                    if abs(prev_dir - cur_dir) < 10:
                        is_same_dir = True
                        break
                if not is_same_dir:
                    break
                total_length += m_link_id_to_link_info_all[cur_link_id]["link_length"]
                if total_length > length_threshold:
                    break

                hash_table.add(cur_link_id)
                cluster_link.appendleft(cur_link_id)
                index_link = cur_link_id
        
        # 向后搜索
        index_link = no_trajs_link
        while True:
            index_node = m_link_id_to_link_info_all[index_link]["end_node_id"] if m_link_id_to_link_info_all[index_link]["link_direction"] in [1, 3] \
                    else m_link_id_to_link_info_all[index_link]["start_node_id"]
            
            if not start_node_index_links[index_node] or len(start_node_index_links[index_node]) > 1:
                break

            cur_link_id = start_node_index_links[index_node][0]
            if cur_link_id not in have_no_trajs_link_id:
                if cur_link_id in have_trajs_link_id:
                    prev_dirs = link_dir_angle[cur_link_id]
                    cur_dirs = link_dir_angle[index_link]
                    for prev_dir, cur_dir in product(prev_dirs, cur_dirs):
                        if abs(prev_dir - cur_dir) < 10:
                            cluster_link.append(cur_link_id)
                            find_back = True
                            break
                break
            
            else:  # 后端link也是未有轨迹覆盖
                prev_dirs = link_dir_angle[cur_link_id]
                cur_dirs = link_dir_angle[index_link]
                is_same_dir = False
                for prev_dir, cur_dir in product(prev_dirs, cur_dirs):
                    if abs(prev_dir - cur_dir) < 10:
                        is_same_dir = True
                        break
                if not is_same_dir:
                    break
                total_length += m_link_id_to_link_info_all[cur_link_id]["link_length"]
                if total_length > length_threshold:
                    break
                hash_table.add(cur_link_id)
                cluster_link.append(cur_link_id)
                index_link = cur_link_id
            
        if find_front and find_back:
            # print("---------------", cluster_link)
            pre_link = cluster_link.popleft()
            next_link = cluster_link.pop()
            length = 0
            for link_id in cluster_link:
                m_link_id_to_link_info_all[link_id]['trail_ids'] = m_link_id_to_link_info_all[pre_link]['trail_ids']
                m_link_id_to_link_info_all[link_id]['tile_ids'] = m_link_id_to_link_info_all[pre_link]['tile_ids'] 
                m_link_id_to_link_info_all[link_id]['trail_names'] = m_link_id_to_link_info_all[pre_link]['trail_names']
                m_link_id_to_link_info_all[link_id]['in_scope'] = True

                if length > length_threshold/2.0:
                    m_link_id_to_link_info_all[link_id]['trail_ids'] = m_link_id_to_link_info_all[next_link]['trail_ids']
                    m_link_id_to_link_info_all[link_id]['tile_ids'] = m_link_id_to_link_info_all[next_link]['tile_ids'] 
                    m_link_id_to_link_info_all[link_id]['trail_names'] = m_link_id_to_link_info_all[next_link]['trail_names']

            cluster_links.append(cluster_link)
    
    return cluster_links



def fill_template4(template, data_folder, use_opt_pose, data_type, str_run_secific_trajs, debug_model):
    """根据数据填充模板"""
    if template is None:
        return template

    data_folder = os.path.join(data_folder, "..")
    # 1 读取路口
    # 1 读取 10个路口的sdlink.geojson， 根据 m_link_id_to_link_info 筛选出，对应 link id 上的所有 wgs84点
    m_link_id_to_link_info_all = collections.OrderedDict()
    have_trajs_link_id = set([])
    have_no_trajs_link_id = set([])
    node_map = [collections.defaultdict(list), collections.defaultdict(list)]
    link_dir_angle = collections.defaultdict(list)

    info_json_path = os.path.join(data_folder, "auto_label/input/sd_link.geojson")
    with open(info_json_path,'r',encoding='utf8')as fp:
        json_data = json.load(fp)
    fp.close()

    for link in json_data['features']:
        link_id = link["properties"]["link_id"]
        link_direction = link["properties"]["direction"]
        kind = link["properties"]["kind"]
        form = link["properties"]["form"].split(",")
        lanenum_sum = link["properties"]["lanenum_sum"]
        start_node_id = link["properties"]["start_node_id"]
        end_node_id = link["properties"]["end_node_id"]
        link_length = link["properties"]["length"]
        # print("---------------------- link_length: {} ----------------------".format(link_length))

        link_geom = "LINESTRING ("
        task_geom = "POLYGON (("

        cur_links = link['geometry']['coordinates']
        for wgs in cur_links: 
            # TODO:qzc gcj01
            
            # if debug_model == 1:   #联调的原始数据已经是wgs84， 本地调试的是gcj02，需转一次
            #     wgs[0], wgs[1] = wgs2gcj(wgs[0], wgs[1]) # gcj02->wgs84
            
            # wgs[0], wgs[1] = gcj2wgs(wgs[0], wgs[1]) # gcj02->wgs84
            # TODO:qzc gcj01
            link_geom = link_geom + str(wgs[0]) + " " + str(wgs[1]) + ","
            task_geom = task_geom + str(wgs[0]) + " " + str(wgs[1]) + ","

        link_geom = link_geom[:-1] + ")"
        task_geom += str(cur_links[0][0]) + " " + str(cur_links[0][1]) +  "))"  

        m_link_id_to_link_info_all[link_id] = {
                                                "link_geom": link_geom,
                                                "task_geom": task_geom,
                                                "tile_ids": [],
                                                "trail_ids": [],
                                                "trail_names": [],
                                                "in_scope": False,
                                                "link_length": link_length,
                                                "link_direction": link_direction,
                                                "kind": kind,
                                                "form": form,
                                                "lanenum_sum": lanenum_sum,
                                                "start_node_id": start_node_id,
                                                "end_node_id": end_node_id,
                                            }
        have_no_trajs_link_id.add(link_id)

        if link_direction in [1, 3]:
            node_map[0][start_node_id].append(link_id)
            node_map[1][end_node_id].append(link_id)
        if link_direction in [2, 3]:
            node_map[0][end_node_id].append(link_id)
            node_map[1][start_node_id].append(link_id)

        heading = calculate_angle(cur_links[0][0], cur_links[0][1], cur_links[-1][0], cur_links[-1][1])
        if link_direction == 2:
            heading = heading-180 if heading >= 0 else heading+180
        link_dir_angle[link_id].append(heading)
        if link_direction == 3:
            reverse_heading = heading-180 if heading >= 0 else heading+180
            link_dir_angle[link_id].append(reverse_heading)

    # 2 查找当前路径下的所有数据
    run_secific_trajs = str_run_secific_trajs.split(',')

    data_engine_dir = os.path.join(data_folder, "source/data_engine")
    parse_dir = os.path.join(data_folder, "parse")
    match_dir = os.path.join(data_engine_dir, "match")
    match_json = os.path.join(match_dir, "output.json")
    # TODO:qzc use cover:output.json
    
    tasks_json_path = os.path.join(data_folder, "auto_label/input/tasks.json")
    if not os.path.exists(tasks_json_path):
        log.error("!!!! fill_template file not exist: ", tasks_json_path)
        print("File Not Exist !")
        return None
    
    with open(tasks_json_path,'r',encoding='utf8')as fp:
        task_json_data = json.load(fp)
    fp.close()

    trajs = []
    # gps_matched_file = ""
    gps_matched_file = "_mla_egopose_gps_matched.csv"

    # bev_score=task_json_data["bev_score"]
    if data_type == DataType.BYD_LIDAR_B.name or data_type == DataType.BYD_LIDAR_BEV_B.name:
        bag_list=task_json_data["lidar_bag_list"]
    elif data_type in [DataType.BYD_BEV.name, DataType.MMT_RC.name]:
        bag_list=task_json_data["bev_bag_list"]
    else:
        log.error("!!!! fill_template in making info_json: ERROR DATA_TYPE !!!!!!")
        print("ERROR TYPE !")
        return None
    
    for bag in bag_list:
        # parse_traj_dir = os.path.join(parse_dir, bag)
        match_traj_dir = os.path.join(match_dir, bag["bag_id"])
        # 如果有指定运行某几个轨迹，则只跑特定的轨迹
        if run_secific_trajs[0] != '':
            if bag["bag_id"] in run_secific_trajs:
                trajs.append(match_traj_dir)
        else:
            data_file = os.path.join(match_traj_dir, gps_matched_file)
            if os.path.exists(data_file) == False:
                continue
            else:
                try:
                    data = read_csv(data_file)
                except:
                    continue
                trajs.append(match_traj_dir)
                print(match_traj_dir)


    # 3 填充轨迹
    g_tile_id = 0
    g_trail_id = 1

    # 计算中心点
    traj_nums = 0
    avg_proj_lon = 0.0
    avg_proj_lat = 0.0
    for traj in trajs:
        data_file = os.path.join(traj, gps_matched_file)
        data = read_csv(data_file)

        trail_name =  os.path.basename(traj)

        for row_id in range(data.shape[0]):
            link_id = data.loc[row_id]['link_id']

            proj_lon = float(data.loc[row_id]['prj_lng'])
            proj_lat = float(data.loc[row_id]['prj_lat'])
            # TODO:qzc gcj01
            # proj_lon, proj_lat = gcj2wgs(proj_lon, proj_lat) # gcj02->wgs84
            # TODO:qzc gcj01
            if np.isnan(proj_lat) or np.isnan(proj_lon):
                continue
            
            avg_proj_lon += proj_lon
            avg_proj_lat += proj_lat
            traj_nums += 1

            if link_id in m_link_id_to_link_info_all:
                if g_trail_id not in m_link_id_to_link_info_all[link_id]['trail_ids']:
                    m_link_id_to_link_info_all[link_id]['trail_ids'].append(g_trail_id)
                    m_link_id_to_link_info_all[link_id]['tile_ids'].append(g_tile_id)
                    m_link_id_to_link_info_all[link_id]['trail_names'].append(trail_name)

                m_link_id_to_link_info_all[link_id]['in_scope'] = True

                have_trajs_link_id.add(link_id)
                if link_id in have_no_trajs_link_id:
                    have_no_trajs_link_id.remove(link_id)
        
        g_trail_id += 1
        # TODO:qzc
        # g_tile_id += 1 
    
    # print("-----------------  执行否  -----------------------")
    cluster_links = refine_no_trajs_link(m_link_id_to_link_info_all, have_trajs_link_id, have_no_trajs_link_id, node_map, link_dir_angle)
    # print("-----------------", cluster_links)

    links = []
    link_template = template.get('middle', {}).get('links', [{}])[0]
    for k, link_info in m_link_id_to_link_info_all.items():
        if link_info['in_scope'] == False:
            continue

        link = link_template.copy()
        link['link_id'] = k
        link['link_geom'] = link_info['link_geom']
        link['task_geom'] = link_info['task_geom']

        link.update({
            'link_ids': [k],
            'tile_ids': link_info['tile_ids'],
            'tracks': [],
            # 'task_geom': "",
            'link_length': link_info['link_length'],
            'link_direction': link_info['link_direction'],
            'kind': link_info['kind'],
            'form': link_info['form'],
            'lanenum_sum': link_info['lanenum_sum'],
            "start_node_id": link_info['start_node_id'],
            "end_node_id": link_info['end_node_id'],
            'crossing_id': []
        })

        for index in range(len(link_info['trail_ids'])):
            tile_id = link_info['tile_ids'][index]
            trail_id = link_info['trail_ids'][index]
            trail_name = link_info['trail_names'][index]
            link['tracks'].append(
                {
                    "tile_id": tile_id, 
                    "trail_id": trail_id, 
                    "trail_name": trail_name,
                    "branch": "fsd_capture_22q4_P", 
                    "in_pid": 0, 
                    "tags": ["time_day"], 
                    "bev_version": "" 
                }
            )

        links.append(link)

    template['middle']['links'] = links
    template['middle']['sub_crosses'] = []
    

    # 第一个pose 的 wgs84 转换为 utm，num
    # TODO:qzc 读取中心点

    origin_wgs_lon = float(avg_proj_lon/traj_nums)
    origin_wgs_lat = float(avg_proj_lat/traj_nums)
    origin_wgs_alt = 0

    if data_type == DataType.BYD_LIDAR_B.name:
        pos_txt_path = os.path.join(data_folder, "source/lidar_mapping/output/gcj02_origin.txt")
        if os.path.exists(pos_txt_path) == False:
            # TODO:yx 补充错误码
            log.error("!!!! fill_template in making info_json: FILE NOT EXIST: {} !!!!!!".format(pos_txt_path))
            print("file not exist: ", pos_txt_path)
            return None
        else:
            with open(pos_txt_path, 'r') as file:
                line = file.readline().strip()
                while line[0] == '#':
                    line = file.readline().strip()
                    
                try:
                    lla = [float(num) for num in line.split()]
                    origin_wgs_lon, origin_wgs_lat, origin_wgs_alt = lla[0], lla[1], 0
                except Exception as e:
                    log.error("!!!! fill_template in making info_json: LLA NOT NORMAL: {} !!!!!!".format(lla))
                    print("lla not normal: ", lla)
                    return None

    origin_utm_num = lonlat_to_utm_num(origin_wgs_lon, origin_wgs_lat)
    [utm_x, utm_y, alt] = lonlatalt_utm_num_to_utm_xyz(origin_wgs_lon, origin_wgs_lat, origin_wgs_alt, origin_utm_num)

    # template = {}
    template['timestamp_us'] = ""
    template['tile_branch'] = ""  
    template['road_branch'] = ""  
    template.get('tile_id_list', []).clear()
    template['utm_num'] = origin_utm_num
    template['t_utm_world'] = [utm_x, utm_y, 0]

    template['middle']['task_id'] = ""
    template['middle']['frame_id'] = ""        
    template['middle']['data_type'] = data_type        
    template['middle']['task_geom_local'] = ""
    template['middle']['task_geom_global'] = ""
    template['middle']['task_geom_local_expand'] = ""
    template['middle']['task_geom_global_expand'] = ""
    template['middle']['site_center'] = []
    template['middle']['site_center_gcj02'] = []

    template['middle']['task_id'] = task_json_data['task_id']  
    template['middle']['frame_id'] = task_json_data['frame_id']

    generate_task_geom_and_site_center(data_folder, task_json_data, template)

    read_sd_node(data_folder, template)


    # 将log中的每个字段设置为空
    if 'log' in template:
        for key in template['log'].keys():
            if isinstance(template['log'][key], str):
                template['log'][key] = ""  
            elif isinstance(template['log'][key], (int, float)):
                template['log'][key] = 0  
            elif isinstance(template['log'][key], dict):
                template['log'][key] = {}  
            elif isinstance(template['log'][key], list):
                template['log'][key] = []  
            elif isinstance(template['log'][key], bool):
                template['log'][key] = False  


    # 将sys_log字段中每一个字段都置空
    if 'sys_log' in template:
        if 'container' in template['sys_log']:
            for key in template['sys_log']['container'].keys():
                template['sys_log']['container'][key] = 0
        if 'host' in template['sys_log']:
            for key in template['sys_log']['host'].keys():
                template['sys_log']['host'][key] = 0
        template['sys_log']['stages'] = []  

    return template

#####################################################################################################
#####################################################################################################
#####################################################################################################

default_template_file = project_root_folder+"/data/task_info_demo.json"
default_output_file = project_root_folder+"/data/task_info_demo_1.json"

# data_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross/11002417915690/PL0169_event_trafficlight_status_event_20240902-101706_0"
data_dir =  "/mnt/d/01_code/04_dataset/dilabel/crowd_source/new/model"
default_data_file = os.path.join(data_dir, "_mla_egopose_gps_matched.csv")


# default_data_folder= "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2"
default_data_folder= "/mnt/d/01_code/04_dataset/dilabel/crowd_source/new/model/auto_label"
# default_data_folder=""
# default_run_secific_trajs = ""
default_run_secific_trajs = "PL0169_event_ld_dash_solid_event_20240904-135253_0"
default_debug_model=1

parser = argparse.ArgumentParser()
parser.add_argument('--template_file', type=str, default=default_template_file, help='模板文件路径')
parser.add_argument('--data_folder', type=str, default=default_data_folder, help='数据文件路径')
parser.add_argument('--data_file', type=str, default=default_data_file, help='数据文件路径')
parser.add_argument('--output_file', type=str, default=default_output_file, help='输出文件路径')
parser.add_argument('--use_opt_pose', type=int, default=0)
parser.add_argument('--data_type', type=str, default=DataType.MMT_RC.name) # mmt: momenta生产数据, mmt_rc: momenta研采, byd: 比亚迪生产, byd_rc: 比亚迪研采
parser.add_argument('--run_secific_trajs', type=str, default=default_run_secific_trajs, help="运行特定的数据") 
parser.add_argument('--debug_model', type=int, default=default_debug_model, help="是否使用debug模式") 

if __name__ == "__main__":
    args = parser.parse_args()
    template = load_json_file(args.template_file)
    filled_template = fill_template4(template, args.data_folder, args.use_opt_pose, args.data_type, args.run_secific_trajs, args.debug_model)
    if filled_template:
        save_json_file(args.output_file, filled_template)
        print(f"{args.output_file} 文件已生成。")
