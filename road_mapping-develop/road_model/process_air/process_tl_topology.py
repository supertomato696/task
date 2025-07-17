import argparse
import os
import util
from util import *
from util import log
import pandas as pd
import geopandas as gpd
from sklearn.cluster import DBSCAN
from shapely.geometry import Point, Polygon

road_model_root_folder = os.path.abspath(__file__ + "/../../")
fsd_map_root_folder = os.path.abspath(__file__ + "/../../../")

sys.path.append(fsd_map_root_folder)
from preprocess.script.trail_data import TrailData, FrameData, json_file_to_dict

class TrafficlightTopology:
    def __init__(self) -> None:
        self.task_folder = ""
        self.task_data_folder = ""
        self.trail_dict: Dict[str, TrailData] = {}
        self.proc_gdf: gpd.GeoDataFrame = None
        self.frame_id = None

    def load_data_set(self):
        tl_file = os.path.join(self.bev_mappping_folder, f"tl_res_merging.shp")
        self.proc_gdf = gpd.read_file(tl_file).to_crs(epsg=3857)

        lane_file = os.path.join(self.bev_mappbuild_folder, 'export_to_shp', f"lane.shp")
        self.lane_gdf = gpd.read_file(lane_file).to_crs(epsg=3857)

        aois_file = os.path.join(self.bev_mappbuild_folder, 'export_to_shp', f"junction.shp")
        self.aois_gdf = gpd.read_file(aois_file).to_crs(epsg=3857)

        road_file = os.path.join(self.bev_mappbuild_folder, 'export_to_shp', f"road.shp")
        self.road_gdf = gpd.read_file(road_file).to_crs(epsg=3857)

        lc_file = os.path.join(self.bev_mappbuild_folder, 'export_to_shp', f"lane_center.shp")
        self.lc_gdf = gpd.read_file(lc_file).to_crs(epsg=3857)

        # 读取任务框范围
        task_info_file = os.path.join(os.path.dirname(self.task_folder), 'task_info_demo.json')
        if os.path.exists(task_info_file):
            task_info: dict = json_file_to_dict(task_info_file)
            if 'frame_id' in task_info['middle'] and task_info['middle']['frame_id'] != '':
                self.frame_id = int(task_info['middle']['frame_id'])

        major_version = None
        minor_version = None
        patch_version = None
        self.version_string = None
        os.chdir(fsd_map_root_folder)
        with open('road_model/version/version.h', 'r') as file:
            for line in file:
                if line.startswith('#define ROAD_MAPPING_MAJOR_VERSION'):
                    major_version = line.split()[-1]
                elif line.startswith('#define ROAD_MAPPING_MINOR_VERSION'):
                    minor_version = line.split()[-1]
                elif line.startswith('#define ROAD_MAPPING_PATCH_VERSION'):
                    patch_version = line.split()[-1]
        if major_version is not None and minor_version is not None and patch_version is not None:
            self.version_string = '.'.join([major_version, minor_version, patch_version])


    def make_trafficlight_topolopy(self):

        def is_intersect_or_close(road_polygon: Polygon, intersection_polygon: Polygon, threshold=10.0):
            if road_polygon.intersects(intersection_polygon):
                return True
            min_distance = road_polygon.distance(intersection_polygon)
            if min_distance < threshold:
                return True
            return False

        def is_within_or_close(traffic_light_point: Point, intersection_polygon: Polygon, threshold=20.0):
            if intersection_polygon.contains(traffic_light_point):
                return True
            min_distance = traffic_light_point.distance(intersection_polygon)
            if min_distance < threshold:
                return True
            return False

        def is_line_towards_polygon(line, polygon):
            line_start = line.geometry.coords[0]
            line_end = line.geometry.coords[-1]
            # 计算线的方向向量
            line_vector = (line_end[0] - line_start[0], line_end[1] - line_start[1])
            # 计算从线的起点到多边形质心的向量
            polygon_centroid = np.array(polygon.centroid.xy)
            centroid_vector = (polygon_centroid[0][0] - line_start[0], polygon_centroid[1][0] - line_start[1])
            # 计算两个向量的点积
            dot_product = line_vector[0] * centroid_vector[0] + line_vector[1] * centroid_vector[1]
            return dot_product > 0

        def calculate_line_orientation(line):
            """
            计算线的朝向
            :param line: GeoSeries 中的线对象
            :return: 线的朝向角度（弧度）
            """
            start = np.array(line.geometry.coords[0])
            end = np.array(line.geometry.coords[-1])
            vector = end - start
            return np.arctan2(vector[1], vector[0])

        def calculate_distances(lines, points):
            """
            计算点到线的垂直距离和水平距离
            :param lines: 一组线的 GeoDataFrame
            :param points: 一组点的 GeoDataFrame
            :return: 包含垂直距离和水平距离的数组
            """
            idx = lines.shape[0] // 2
            line_orientation = calculate_line_orientation(lines.iloc[idx])
            vertical_distances = []
            horizontal_distances = []
            angle_distances = []
            l_or_r = []
            for _, point in points.iterrows():
                point_geom = point.geometry
                point_vector = np.array([point_geom.x, point_geom.y])
                v_dist, h_dist, lr = None, None, None
                for j, line in lines.iterrows():
                    line_start = np.array(line.geometry.coords[0])
                    line_end = np.array(line.geometry.coords[-1])
                    vector_to_point = point_vector - line_end
                    # 计算垂直距离
                    line_vec = np.array(line_end) - np.array(line_start)
                    cross_product = np.cross(line_vec, vector_to_point)
                    distance = np.abs(cross_product) / np.linalg.norm(line_vec)
                    if not v_dist or abs(v_dist) > abs(distance):
                        v_dist = distance
                    # 计算水平距离
                    projection = np.dot(vector_to_point, np.array([np.cos(line_orientation), np.sin(line_orientation)]))
                    if not h_dist or h_dist < projection:
                        h_dist = projection
                    # 计算点在车道的左侧还是右侧
                    if not h_dist or cross_product > 0:
                        lr = 1
                    else:
                        lr = 0
                vertical_distances.append(v_dist)
                horizontal_distances.append(h_dist)
                l_or_r.append(lr)
                dang = abs(point['heading'] - line_orientation * 180 / np.pi)
                if dang % 180 <= 90:
                    dang = dang % 180
                else:
                    dang = 180 - dang % 180
                angle_distances.append(dang)
            return vertical_distances, horizontal_distances, angle_distances, l_or_r, line_orientation

        def sort_perpendicular_points(points: list, start, end):
            """根据直线朝向从左到右排序垂直于直线的点"""
            line_direction_vector = [end[0] - start[0], end[1] - start[1]]

            def get_cross_product_value(point):
                """计算点与直线起点构成的向量和直线方向向量的叉积"""
                vector_start_to_point = (point[1] - end[0], point[2] - end[1])
                return line_direction_vector[0] * vector_start_to_point[1] - line_direction_vector[1] * \
                    vector_start_to_point[0]

            return sorted(points, key=get_cross_product_value, reverse=True)

        def project_points(points, r):
            """计算点在直线上的投影点"""
            center = np.mean(points, axis=0, keepdims=True)
            projection_points = []
            for point in points:
                # 直线的方向向量
                direction_vector = np.array([np.cos(r), np.sin(r)])
                # 点到中心点的向量
                vector_to_center = np.array(point) - center
                # 计算投影长度
                projection_length = np.dot(vector_to_center, direction_vector)
                # 计算投影点
                projection = center + projection_length * direction_vector
                projection_points.append(projection.tolist())
            return np.array(projection_points)

        def save_dict(res_dict, l_orient, tl_id, junc_id, lane_ids, xy: Point):
            if tl_id in res_dict['ID']:
                ind = res_dict['ID'].index(tl_id)
                res_dict['LANE_ID'][ind] += (',' + ','.join(map(str, lane_ids)))
                return res_dict

            res_dict['ID'].append(str(self.frame_id) + '_' + str(tl_id))
            l_orient = np.degrees(l_orient)
            l_orient = 180 + l_orient if l_orient < 0 else 180 + l_orient
            res_dict['HEADING'].append(l_orient % 360)
            res_dict['VERSION'].append(self.version_string)
            if isinstance(junc_id, int) or isinstance(junc_id, float):
                res_dict['JUNC_ID'].append(str(junc_id))
            else:
                res_dict['JUNC_ID'].append('')
            res_dict['LANE_ID'].append(','.join(map(str, lane_ids)))
            res_dict['C_WALK_ID'].append('')
            res_dict['TYPE'].append(np.nan)
            res_dict['geometry'].append(xy)

            return res_dict

        # data_dict = {'geometry': [], 'VERSION': [], 'ALG_ID': [], 'HEADING': [], 'JUNC_ID': [], 'C_WALK_ID': [],
        #             'LANE_ID': [], 'TYPE': [], 'ID': []}
        data_dict = {'geometry': [], 'ID': [], 'HEADING': [], 'TYPE': [], 'JUNC_ID': [], 'C_WALK_ID': [],
                    'LANE_ID': [], 'ALG_ID': [], 'VERSION': []}

        tl_rois, tl_lane = defaultdict(list), defaultdict(list)
        person_light, finished_light = [], []

        # 筛选末端的roads
        filtered_roads = self.road_gdf[self.road_gdf['NEXT_ID'].isnull()]

        # 遍历路口面
        for index_intersection, intersection_row in self.aois_gdf.iterrows():
            road_ids, tl_ids = [], []
            intersection_polygon: Polygon = intersection_row['geometry']
            centroid = intersection_polygon.centroid
            centroid_xy = np.array([centroid.x, centroid.y])

            # 筛选在该路口面的交通灯，并保存group_id列表
            for index_traffic_light, traffic_light_row in self.proc_gdf.iterrows():
                traffic_light_point = traffic_light_row['geometry']
                if is_within_or_close(traffic_light_point, intersection_polygon):
                    tl_ids.append(traffic_light_row['group_id'])
            tl_ids = set(tl_ids)
            filtered_tls = self.proc_gdf[(self.proc_gdf['group_id'].isin(tl_ids))]
            if filtered_tls.empty or filtered_tls.shape[0] == 0:
                continue

            # 记录交通灯和路口面的关联关系
            for _, tl_row in filtered_tls.iterrows():
                tl_rois[tl_row['ID']].append(intersection_row['ID'])

            # 遍历每个道路
            for index_road, road_row in filtered_roads.iterrows():
                # 判断道路是否和路口面相交或者距离小于2米
                road_polygon = road_row['geometry']
                if is_intersect_or_close(road_polygon, intersection_polygon):
                    lane_turn, line_turn, get_light = defaultdict(list), defaultdict(list), defaultdict(list)
                    # 根据ROAD_ID筛选在该道路内的车道lane
                    filtered_lanes = self.lane_gdf[self.lane_gdf['ROAD_ID'] == road_row['ID']]
                    
                    # 遍历在其内的车道
                    for _, lane_row in filtered_lanes.iterrows():
                        lc_line = self.lc_gdf[self.lc_gdf['ID'] == lane_row['LEFT_BID']]
                        if lc_line.empty or lc_line.shape[0] == 0:
                            continue

                        # 如果朝向不是向着路口面的，即退出车道则跳过
                        if not is_line_towards_polygon(lc_line.iloc[0], intersection_polygon):
                            continue
                        # 划分left、straight、right三种车道
                        if not lane_row['TURN_TYPE']:
                            lane_turn['straight'].append(lane_row)
                            line_turn['straight'].append(lc_line)
                        else:
                            if '2' in lane_row['TURN_TYPE']:
                                lane_turn['right'].append(lane_row)
                                line_turn['right'].append(lc_line)
                            if '3' in lane_row['TURN_TYPE']:
                                lane_turn['left'].append(lane_row)
                                line_turn['left'].append(lc_line)
                            if '1' in lane_row['TURN_TYPE']:
                                lane_turn['straight'].append(lane_row)
                                line_turn['straight'].append(lc_line)

                    # 根据group_id遍历在路口面内的交通灯
                    pts_groups = filtered_tls.groupby('group_id')
                    # 以直行道路作为参考
                    tmp_lines = line_turn['straight'] if line_turn['straight'] else line_turn['right']
                    if not tmp_lines:
                        continue
                    lines = gpd.GeoDataFrame(pd.concat(tmp_lines, ignore_index=True))

                    # 计算路口面中心点到车道线的投影距离，作为距离判断的基准
                    line_orientation = calculate_line_orientation(tmp_lines[0].iloc[0])
                    line_end = np.array(tmp_lines[0].iloc[0].geometry.coords[-1])
                    vector_to_point = centroid_xy - line_end
                    ref_dist = np.dot(vector_to_point, np.array([np.cos(line_orientation), np.sin(line_orientation)]))

                    # print('00001....')
                    for group_id_pts in pts_groups.groups:
                        points = pts_groups.get_group(group_id_pts)
                        # 计算交通灯到车道lane的垂直/平行距离、角度偏差、左/右侧
                        vertical_distances, horizontal_distances, angle_dist, l_or_r, line_orientation = calculate_distances(lines, points)
                        if min(horizontal_distances) < 15 and horizontal_distances[0] > 0 and min(vertical_distances) < 3*lines.shape[0]:
                            if angle_dist[0] > 60 and points.shape[0] == 1:
                                person_light.append(points)  # 行人灯
                                # print('--------------- 行人灯', road_row['ID'], group_id_pts, angle_dist[0])
                            elif points.shape[0] == 1:
                                # print('*************** 本道机动车灯', road_row['ID'], group_id_pts, angle_dist[0],
                                #     l_or_r[0])
                                finished_light.append(group_id_pts)
                                get_light['here'].append([points, l_or_r, line_orientation])
                            continue

                        if ref_dist * 1.36 < horizontal_distances[0] < ref_dist * 3 and np.min(
                                np.abs(vertical_distances)) < 5:
                            if angle_dist[0] > 60:
                                person_light.append(points)  # 行人灯
                                # print('--------------- 行人灯', road_row['ID'], group_id_pts, angle_dist[0])
                                continue
                            finished_light.append(group_id_pts)
                            get_light['there'].append([points, line_orientation])
                            # print('*************** 对向机动车灯', road_row['ID'], group_id_pts, angle_dist[0])

                    # print('00002....')
                    if not lane_turn['straight']:
                        continue
                    ids = [lane['ID'] for lane in lane_turn['straight']]
                    left_id = [lane['ID'] for lane in lane_turn['left']] if lane_turn['left'] else [np.min(ids)]
                    right_id = [lane['ID'] for lane in lane_turn['right']] if lane_turn['right'] else [np.max(ids)]
                    for pts, l_or_r, line_orientation in get_light['here']:
                        left_id = [lane['ID'] for lane in lane_turn['left']] if lane_turn['left'] else [np.min(ids)]
                        right_id = [lane['ID'] for lane in lane_turn['right']] if lane_turn['right'] else [np.max(ids)]
                        if l_or_r[0] == 1:
                            tl_lane[pts['ID'].iloc[0]].extend(left_id)
                            res_ids = left_id
                        else:
                            tl_lane[pts['ID'].iloc[0]].extend(ids)
                            res_ids = ids
                        data_dict = save_dict(
                            data_dict, line_orientation, pts['ID'].iloc[0], intersection_row['ID'], res_ids, pts['geometry'].iloc[0])
                        person_light.append(pts)

                        # data_dict['ID'].append(len(data_dict['ID']))
                        # orient = np.degrees(line_orientation)
                        # orient = 180+orient if orient < 0 else 180+orient
                        # data_dict['HEADING'].append(np.round(orient, 2))
                        # data_dict['VERSION'].append('v1.00.10')
                        # data_dict['JUNC_ID'].append(str(intersection_row['ID']))
                        # data_dict['LANE_ID'].append(','.join(res_ids))
                        # data_dict['C_WALK_ID'].append('')
                        # data_dict['TYPE'].append('')
                        # data_dict['geometry'].append(pts['geometry'].iloc[0])

                    # print('00003....')
                    # points: list, start, end
                    if not get_light['there']:
                        continue
                    
                    pts, heading, tmp = None, None, None
                    if len(get_light['there']) > 1:
                        tmp = [[tls, line_orient] for tls, line_orient in get_light['there'] if tls.shape[0] > 1]
                    if not tmp:
                        pts = get_light['there'][0][0]
                        heading = get_light['there'][0][1]
                    else:
                        pts = tmp[0][0]
                        heading = tmp[0][1]
                        for ti, (tls, line_orient) in enumerate(get_light['there']):
                            if tls.shape[0] <= 1:
                                get_light['there'].pop(ti)
                                # person_light.append(tls)

                    person_light.append(pts)
                    mid = len(ids) // 2
                    # print(ids, lane_turn, get_light)
                    tlane_center = lane_turn['straight'][mid].geometry.centroid
                    start, end = [tlane_center.x, tlane_center.y], centroid_xy
                    points = []
                    for index, row in pts.iterrows():
                        point_id = row['ID']  # 假设id是GeoDataFrame中的一列
                        point_x = row['geometry'].x
                        point_y = row['geometry'].y
                        points.append([point_id, point_x, point_y])

                    points = sort_perpendicular_points(points, start, end)
                    if len(points) == 1:
                        # tl_lane[points[0][0]].extend(left_id)
                        # tl_lane[points[0][0]].extend(ids)
                        # tl_lane[points[0][0]].extend(right_id)
                        # tl_lane[points[0][0]] = list(set(tl_lane[points[0][0]]))

                        res_ids = list(set(left_id+ids+right_id))
                        data_dict = save_dict(
                            data_dict, heading, points[0][0], intersection_row['ID'], res_ids, pts['geometry'].iloc[0])

                    # pts = np.array(points)
                    res_xy = project_points(np.array(points)[:, 1:3], heading + np.pi*0.5)
                    if len(points) == 2:
                        # tl_lane[points[0][0]].extend(left_id)
                        tids = [tid for tid in ids if (tid not in left_id)]
                        if len(tids) == 0: tids = ids
                        # tl_lane[points[1][0]].extend(tids)
                        # tl_lane[points[1][0]].extend(right_id)

                        data_dict = save_dict(
                            data_dict, heading, points[0][0], intersection_row['ID'], left_id, Point(res_xy[0]))
                        data_dict = save_dict(
                            data_dict, heading, points[1][0], intersection_row['ID'], tids, Point(res_xy[1]))
                    if len(points) == 3:
                        # tl_lane[points[0][0]].extend(left_id)
                        tids = [tid for tid in ids if (tid not in left_id and tid not in right_id)]
                        # tl_lane[points[1][0]].extend(tids)
                        # tl_lane[points[2][0]].extend(right_id)

                        data_dict = save_dict(
                            data_dict, heading, points[0][0], intersection_row['ID'], left_id, Point(res_xy[0]))
                        data_dict = save_dict(
                            data_dict, heading, points[1][0], intersection_row['ID'], tids, Point(res_xy[1]))
                        data_dict = save_dict(
                            data_dict, heading, points[2][0], intersection_row['ID'], right_id, Point(res_xy[2]))
                    if len(points) == 4:
                        # tl_lane[points[0][0]].extend(left_id)
                        # tl_lane[points[1][0]].extend(left_id)
                        tids = [tid for tid in ids if (tid not in left_id)]
                        # tl_lane[points[2][0]].extend(tids)
                        # tl_lane[points[3][0]].extend(tids)

                        data_dict = save_dict(
                            data_dict, heading, points[0][0], intersection_row['ID'], left_id, Point(np.mean(res_xy[:2], axis=0)))
                        data_dict = save_dict(
                            data_dict, heading, points[2][0], intersection_row['ID'], tids, Point(np.mean(res_xy[2:4], axis=0)))
                    if len(points) == 5:
                        # tl_lane[points[0][0]].extend(left_id)
                        # tl_lane[points[1][0]].extend(left_id)
                        tids = [tid for tid in ids if (tid not in left_id)]
                        # tl_lane[points[2][0]].extend(tids)
                        # tl_lane[points[3][0]].extend(tids)
                        # tl_lane[points[4][0]].extend(right_id)

                        data_dict = save_dict(
                            data_dict, heading, points[0][0], intersection_row['ID'], left_id, Point(np.mean(res_xy[:2], axis=0)))
                        data_dict = save_dict(
                            data_dict, heading, points[2][0], intersection_row['ID'], tids, Point(np.mean(res_xy[2:4], axis=0)))
                    if len(points) >= 6:
                        # tl_lane[points[0][0]].extend(left_id)
                        # tl_lane[points[1][0]].extend(left_id)
                        tids = [tid for tid in ids if (tid not in left_id and tid not in right_id)]
                        # tl_lane[points[2][0]].extend(tids)
                        # tl_lane[points[3][0]].extend(tids)
                        # tl_lane[points[4][0]].extend(right_id)
                        # tl_lane[points[5][0]].extend(right_id)

                        data_dict = save_dict(
                            data_dict, heading, points[0][0], intersection_row['ID'], left_id, Point(np.mean(res_xy[:2], axis=0)))
                        data_dict = save_dict(
                            data_dict, heading, points[2][0], intersection_row['ID'], tids, Point(np.mean(res_xy[2:4], axis=0)))
                        data_dict = save_dict(
                            data_dict, heading, points[4][0], intersection_row['ID'], tids, Point(np.mean(res_xy[4:6], axis=0)))

        # print('###########  traffic_light -- roi  ###########')
        # for key in tl_rois:
        #     print(f"{key}: {tl_rois[key]}")
        # print('###########  traffic_light -- lane  ###########')
        # for key in tl_lane:
        #     print(f"{key}: {tl_lane[key]}")
        # print('person_light: ', person_light)
        # print('traffic_light -- roi:', tl_rois)
        # print('traffic_light -- lane:', tl_lane)

        person_light_ids = []
        for t in person_light:
            for _, row in t.iterrows():
                person_light_ids.append(row['ID'])
        for _, tl_row in self.proc_gdf.iterrows():
            if tl_row['ID'] in person_light_ids or tl_row['ID'] in data_dict['ID']:
                continue
            t = tl_row['heading'] - 180
            heading = t % 360 if t % 360 >= 0 else 360 + t % 360
            heading = np.radians(heading)
            roi_id = tl_rois[tl_row['ID']][0] if tl_rois[tl_row['ID']] else []
            data_dict = save_dict(data_dict, heading, tl_row['ID'], roi_id, [], tl_row['geometry'])

        data_dict['ALG_ID'] = [np.nan]*len(data_dict['HEADING']) if self.frame_id is None else [self.frame_id]*len(data_dict['HEADING'])
        res_gdf = gpd.GeoDataFrame(data_dict, crs="EPSG:3857").to_crs("EPSG:4326")
        res_gdf['HEADING'] = res_gdf['HEADING'].round(2)
        res_gdf['ALG_ID'] = res_gdf['ALG_ID'].astype('Int64')
        res_gdf['TYPE'] = res_gdf['TYPE'].astype('Int64')
        res_gdf.to_file(os.path.join(self.bev_mappping_folder, f"traffic_light.shp"))

        if os.path.exists(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp")):
            os.remove(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp"))
        res_gdf.to_file(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp"))

        # log.info("finish trafficlight_topology")

    def run(self, task_folder: str):
        self.task_folder = task_folder
        self.task_data_folder = os.path.join(self.task_folder, "data")
        self.bev_mappping_folder = os.path.join(self.task_folder, "trafficlight_processing")
        self.bev_mappbuild_folder = os.path.join(self.task_folder, "bev_mapbuild_out")
        
        log.info("====== make_trafficlight_topology: 数据载入 ======")
        self.load_data_set()

        if self.proc_gdf is not None:
            log.info("====== make_trafficlight_topology: 开始处理 ======")
            self.make_trafficlight_topolopy()


base_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross/"
default_input_json = os.path.join(base_dir, "11002417915690/PL0169_event_trafficlight_status_event_20240902-101706_0/223126")
parser = argparse.ArgumentParser()
parser.add_argument('--task_folder', type=str, default=default_input_json)
# parser.add_argument('--input', type=str, default='1-2')
# parser.add_argument('--output', type=str, default=default_input_json)
if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.task_folder, f"log/run_bev_trafficlight.log"))
    error_code = 0
    # log.info("============================================")
    # log.info("  Hello TrafficlightTopology")
    # log.info("============================================")
    try:
        bev_tl_merge: TrafficlightTopology = TrafficlightTopology()
        bev_tl_merge.run(args.task_folder)
    except Exception as e:
        log.error("error:", e)
        error_code = 732  # "交通灯建模_异常退出"
    
    # log.info("完成 TrafficlightTopology")
    tl_proc_result = {"error_code": error_code}
    util.dict_to_json_file(os.path.join(args.task_folder, "trafficlight_processing/trafficlight_processing_result.json"), tl_proc_result)  # 保存mapping环境结果文件
