import argparse
import os
import pandas as pd
import geopandas as gpd
import shapely.wkt
import shapely.ops
from shapely.ops import linemerge
import shapely.affinity
from shapely.geometry import Point, Polygon, LineString, GeometryCollection
import json
import collections
import pyproj
import numpy as np
import pandas as pd
import math

from sklearn.decomposition import PCA
from scipy.spatial import KDTree

import sys,os
project_root_folder = os.path.abspath(__file__+"/../../../../")
sys.path.append(project_root_folder)

from data_io.script.util import lonlat_to_utm_num, lonlatalt_utm_num_to_utm_xyz, gcj2wgs, wgs2gcj



class ProcessLinkNode:
    def __init__(self, fsd_map_output_dir, workspace_folder, task_id, info_json_path) -> None:
        self.fsd_map_output_dir = fsd_map_output_dir
        self.workspace_folder = workspace_folder
        self.task_id = task_id
        self.info_json_path = info_json_path

        # sd node info
        self.m_node_id_to_node_info_ = collections.OrderedDict() # 所有 node id
        self.m_node_id_to_value_2_ = collections.OrderedDict() # 所有 node id, cross_flag == 2
        self.m_node_id_to_value_3_ = collections.OrderedDict() # 所有 node id, cross_flag == 3

        self.site_center_gcj02 =  None
        self.utm_num_ = 50
        self.t_utm_world_ = [0,0,0]

        self.inner_link_poly_50m = None

        # 加载 task info
        self.load_task_info()
        self.proj_transform_ = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:326{self.utm_num_}", always_xy=True).transform

        # 加载 sd_node.geojson
        self.load_sd_node()

    def get_inner_link_poly_50m_wkt(self):
        if not self.inner_link_poly_50m:
            return ""
        return self.inner_link_poly_50m.wkt

    def load_task_info(self):
        #读取任务信息
        with open(self.info_json_path,'r',encoding='utf8')as fp:
            json_data = json.load(fp)
        fp.close()

        data_scope_wkt = json_data['middle']['task_geom_global']
        self.data_scope_poly_ = shapely.wkt.loads(data_scope_wkt)

        self.utm_num_ = json_data['utm_num']
        self.t_utm_world_ = json_data['t_utm_world']

        if 'site_center_gcj02' in json_data['middle'].keys():
            self.site_center_gcj02 = shapely.wkt.loads(json_data['middle']['site_center_gcj02'])
        else:
            print("!!!! no site_center_gcj02 !!!!")

        # if 'site_center' in json_data['middle'].keys():
        #     self.site_center_local = shapely.wkt.loads(json_data['middle']['site_center'])
        # else:
        #     print("!!!! no site_center_local !!!!")


    def load_sd_node(self):
        geojson_path = os.path.join(self.workspace_folder, "input/sd_node.geojson")
        with open(geojson_path,'r',encoding='utf8')as fp:
            json_data = json.load(fp)
        fp.close()

        node_utm_ls = shapely.ops.transform(self.proj_transform_, self.site_center_gcj02)
        site_center_local_tmp = shapely.affinity.translate(node_utm_ls, -self.t_utm_world_[0], -self.t_utm_world_[1])

        proj_transform_inverse = pyproj.Transformer.from_crs(f"EPSG:326{self.utm_num_}", "EPSG:4326", always_xy=True).transform
        all_link_in_task_world_ls2 = shapely.affinity.translate(site_center_local_tmp.buffer(30, 32), self.t_utm_world_[0], self.t_utm_world_[1])
        scope_poly_30m_gcj = shapely.ops.transform(proj_transform_inverse, all_link_in_task_world_ls2)

        inner_link_node = []
        for node in json_data['features']:
            cross_flag = node["properties"]["cross_flag"]
            coordinates = node["geometry"]["coordinates"]

            point_wkt = "Point ("+str(coordinates[0]) + " " + str(coordinates[1]) + ")"
            point = shapely.wkt.loads(point_wkt)
            if scope_poly_30m_gcj.contains(point):
                if cross_flag == 2 or cross_flag == 3:
                    node_utm_ls = shapely.ops.transform(self.proj_transform_, point)
                    node_world_ls = shapely.affinity.translate(node_utm_ls, -self.t_utm_world_[0], -self.t_utm_world_[1])
                    inner_link_node.append((node_world_ls.xy[0][0], node_world_ls.xy[1][0]))


        inner_link_poly = None
        if len(inner_link_node) >= 3:
            inner_link_poly = Polygon(inner_link_node)
            inner_link_poly_convex = inner_link_poly.convex_hull
            self.inner_link_poly_50m = inner_link_poly_convex.buffer(50)
            print("before area: {}, mid_area: {}, after area: {}".format(inner_link_poly.area, inner_link_poly_convex.area, self.inner_link_poly_50m.area))

