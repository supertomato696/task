import argparse
import os
import util
from util import *
from util import log
import geopandas as gpd
from sklearn.cluster import DBSCAN

road_model_root_folder = os.path.abspath(__file__ + "/../../")
fsd_map_root_folder = os.path.abspath(__file__ + "/../../../")

sys.path.append(fsd_map_root_folder)
from preprocess.script.trail_data import TrailData, FrameData, json_file_to_dict

class TrafficlightTracking:
    def __init__(self) -> None:
        self.task_folder = ""
        self.task_data_folder = ""
        self.trail_dict: Dict[str, TrailData] = {}
        self.proc_gdf: gpd.GeoDataFrame = None
        self.links: list = []

    def load_data_set(self, meta_json):
        data_dict = {'geometry':[], 'timestamp':[], 'is_Ltrail':[], 'heading':[], 'heading_theta_bv':[], 'delta_x':[], 'delta_y':[], 'score':[], 'type':[], 'bulb_type':[], 'pattern':[], }
        trail_heading = []

        trail_json: dict = json_file_to_dict(meta_json)
        if 'trail_id' not in trail_json["header"]:
            log.warning("[TrafficlightProcessing] info.json 不存在，跳过:", frame_data_json_path)
            return
        self.trail_id = trail_json["header"]["trail_id"]

        # if self.trail_id in ['6']:
        #     print('xxx')
        
        # 读取SD link
        link_file = os.path.join(self.task_data_folder, self.trail_id, 'trail_info.json')
        link_json: dict = json_file_to_dict(link_file)['binded_links']
        utm_num, t_utm_world = trail_json["data"][0]["utm_num"], trail_json["data"][0]["t_utm_world"]
        link_dict = {'geometry':[]}
        
        for i in range(len(link_json)):
            line_str = link_json[i]['line_string']
            ori_xys = [tuple(map(float, coord.split())) for coord in line_str.split('(')[1].split(')')[0].split(',')]
            res_xys = []
            for t, xy in enumerate(ori_xys):
                txyz = util.utm_xyz_num_to_lonlatalt(xy[0]+t_utm_world[0], xy[1]+t_utm_world[1], 0, utm_num)
                res_xys.append([txyz[0], txyz[1]])
            link_dict["geometry"].append(LineString(res_xys))
            # line = LineString(xys)
            # self.links.append(LineString(res_xys))
        link_gdf = gpd.GeoDataFrame(link_dict, crs="EPSG:4326").to_crs("EPSG:3857")
        link_gdf.to_file(os.path.join(self.bev_mappping_folder, f"link_data_{self.trail_id}.gpkg"), layer='sd_link', driver='GPKG')
        self.links = link_gdf['geometry'].tolist()

        for frame_meta in trail_json["data"]:
            frame_data_json_path = frame_meta["json_path"].replace('output_refine', 'output')
            # t, q = frame_meta["t"], frame_meta["q"]
            rotation, yaw = util._transform_quaternion(frame_meta["q"])
            t_local, utm_num, t_utm_world = frame_meta["t"], frame_meta["utm_num"], frame_meta["t_utm_world"]
            translation = np.array(t_local) + np.array(t_utm_world)
            trail_heading.append(yaw)
            if np.all(rotation == 0.0):
                log.warning("[TrafficlightProcessing] 四元数异常，跳过:", frame_data_json_path)
                continue
            if not os.path.exists(frame_data_json_path):
                log.warning("[TrafficlightProcessing] frame_data.json 不存在，跳过:", frame_data_json_path)
                continue
            data_json = json_file_to_dict(frame_data_json_path)
            frame_data: list = data_json["detail"]['instance_list']
            dist_threshold = 15 if 'MMT_RC' in data_json["cat_version"] else 50
            for instance_data in frame_data:
                if instance_data["name"] == "traffic_light":
                    cur_local_xyz = instance_data["data"]['points'][0]
                    if np.any(np.abs(cur_local_xyz) > dist_threshold):
                        continue  ## 筛除大于15m的识别结果
                    local_xyz = util._transform_point(cur_local_xyz, rotation, translation)
                    gcj_xy = util.utm_xyz_num_to_lonlatalt(local_xyz[0], local_xyz[1], local_xyz[2], utm_num) # local_to_wgs84_projected(local_xyz[0], local_xyz[1], utm_num)
                    # mercator_xy = util.wgs84_to_mercator(wgs_xyz[0], wgs_xyz[1])
                    # gcj_xy = util.wgs2gcj(wgs_xyz[0], wgs_xyz[1])
                    data_dict["geometry"].append(Point(gcj_xy[0], gcj_xy[1]))
                    data_dict["delta_x"].append(cur_local_xyz[0])
                    data_dict["delta_y"].append(cur_local_xyz[1])
                    data_dict["score"].append(instance_data["attrs"]['score'])

                    if "type" in instance_data["attrs"] and len(instance_data["attrs"]['type']) > 0:
                        data_dict["type"].append(instance_data["attrs"]['type'][0])
                    else:
                        data_dict["type"].append('default_type') 

                    # data_dict["bulb_type"].append(instance_data["attrs"]["bulb"][0]["attrs"]['type'])
                    # data_dict["pattern"].append(instance_data["attrs"]["bulb"][0]["attrs"]['pattern'][0])

                    if "bulb" in instance_data["attrs"] and len(instance_data["attrs"]["bulb"]) > 0:
                        bulb_type = instance_data["attrs"]["bulb"][0]["attrs"].get('type', 'default_type')
                        pattern = instance_data["attrs"]["bulb"][0]["attrs"].get('pattern', ['default_pattern'])[0]  # 取第一个元素
                    else:
                        # 如果没有 bulb 数据，则使用默认值
                        bulb_type = 'default_type'
                        pattern = 'default_pattern'
                    
                    # 添加 bulb_type 和 pattern 到 data_dict
                    data_dict["bulb_type"].append(bulb_type)
                    data_dict["pattern"].append(pattern)
                    heading_theta_bv = instance_data["attrs"]['heading_theta_bv']*180/np.pi
                    data_dict["heading_theta_bv"].append(heading_theta_bv)
                    heading = yaw+heading_theta_bv
                    heading = heading if abs(heading) < 180 else (180-abs(heading)%180)*(-heading/abs(heading))
                    data_dict["heading"].append(heading)
                    data_dict["timestamp"].append(frame_meta["timestamp"])

            # if len(data_dict["geometry"]) > 0:
            #     log.info("[TrafficlightProcessing] 完成载入frame_data", frame_meta["timestamp"])
        
        if max(trail_heading) - min(trail_heading) > 60:
            data_dict["is_Ltrail"] = [1] * len(data_dict["geometry"])
        else:
            data_dict["is_Ltrail"] = [0] * len(data_dict["geometry"])
        
        if len(data_dict["geometry"]) > 0:
            self.proc_gdf = gpd.GeoDataFrame(data_dict, crs="EPSG:4326").to_crs("EPSG:3857")
            self.proc_gdf.to_file(os.path.join(self.bev_mappping_folder, f"tl_tracking_data_{self.trail_id}.gpkg"), layer='tracking_data', driver='GPKG')


    def bev_trafficlight_tracking(self):
        if self.proc_gdf is None:
            return
        xy = self.proc_gdf['geometry'].apply(lambda x: (x.x, x.y)).tolist()
        is_Ltrail = self.proc_gdf['is_Ltrail'].tolist()

        if len(xy) < 5:
            # print('#######################  数据过少  ##########################')
            return

        xy = np.asarray(xy)
        db = DBSCAN(eps=5, min_samples=5).fit(xy)   # origin eps=5

        labels = db.labels_
        self.proc_gdf['cluster'] = labels

        def cal_dist_matrix(angles):
            # 计算n*n的角度差矩阵
            angle_diffs = angles[:, np.newaxis] - angles
            angle_diffs = angle_diffs % 180
            # 计算每个差值与0°和180°的最小差值
            # 使用np.minimum计算较小值，np.abs取绝对值，np.mod确保角度在0°到360°之间
            processed_matrix = np.minimum(np.abs(angle_diffs), 180 - np.abs(angle_diffs))
            return processed_matrix

        newxy = []
        for i in range(max(labels) + 1):
            idx = np.where(labels == i)[0]

            xy_gdf = self.proc_gdf.loc[idx]
            yaws = np.asarray(xy_gdf['heading'])
            dist_matrix = cal_dist_matrix(yaws)
            db_yaw = DBSCAN(eps=20, min_samples=5, metric='precomputed')
            db_yaw.fit(dist_matrix)
            labels_yaw = db_yaw.labels_

            # db_yaw = DBSCAN(eps=20, min_samples=5).fit(yaws)
            # labels_yaw = db_yaw.labels_

            for j in range(max(labels_yaw) + 1):
                idx_yaw = np.where(labels_yaw == j)[0]
                gdf_yaw = xy_gdf.loc[idx[idx_yaw]]

                res_tl = self.subprocess_trafficlight(gdf_yaw)
                res_xy, heading, is_car_light, is_single, angles, num_pts = res_tl

                # if is_car_light and not is_single:
                #     print('#######################  处理机动车信号灯(多灯-不同向)  #######################')
                #     print('坐标：{}， 朝向：{}  .....'.format(res_xy, heading))
                #     print('#######################        处理完毕         #######################')
                # if is_car_light and is_single:
                #     print('#######################   处理行人信号灯(多灯-同向)  #######################')
                #     print('坐标：{}， 朝向：{}  .....'.format(res_xy, heading))
                #     print('#######################        处理完毕         #######################')
                # if not is_car_light:
                #     print('#######################   处理单信号灯(行人/机动车)  #######################')
                #     print('坐标：{}， 朝向：{}  .....'.format(res_xy, heading))
                #     print('#######################        处理完毕         #######################')

                if res_xy is not None:
                    for item in res_xy:
                        car_angle_diff = gdf_yaw['heading_theta_bv'].tolist()
                        car_angle_diff = np.array(car_angle_diff) % 180
                        car_angle_diff = np.minimum(np.abs(car_angle_diff), 180 - np.abs(car_angle_diff))
                        is_L = 1 if np.mean(car_angle_diff) > 60 or is_Ltrail[0] == 1 else 0
                        newxy.append([item[0], item[1], i, j, heading, int(is_car_light), int(is_single), is_L, num_pts, np.mean(car_angle_diff)])

        if not newxy:
            return

        # 将新点的经纬度坐标转换为投影坐标，并创建Point对象
        newxy = np.asarray(newxy)
        new_points = [Point(x, y) for x, y in newxy[:, :2]]

        # 创建一个新的GeoDataFrame来存储新点

        new_gdf = gpd.GeoDataFrame(geometry=new_points, crs='epsg:3857')
        new_gdf['cluster_xy'] = newxy[:, 2]
        new_gdf['cluster_yaw'] = newxy[:, 3]
        new_gdf['heading'] = newxy[:, 4]
        new_gdf['is_car_light'] = newxy[:, 5]
        new_gdf['is_single'] = newxy[:, 6]
        new_gdf['is_L'] = newxy[:, 7]
        new_gdf['num_pts'] = newxy[:, 8]
        new_gdf['angle_diff'] = newxy[:, 9]

        # self.proc_gdf = new_gdf
        new_gdf.to_file(os.path.join(self.bev_mappping_folder, f"tl_tracking_res_{self.trail_id}.gpkg"), layer='tracking_res', driver='GPKG')
    
    def transf(self, ang):
        for i in range(len(ang)):

            if abs(ang[i]) > 180:
                ang[i] = (abs(ang[i]) % 180) * ang[i] / abs(ang[i])

        return ang


    def angle_cluster_and_get_mean(self, data, eps=10, min_samples=2):
        # 将数据转换为二维数组，因为KMeans需要二维输入
        data = np.array(data).reshape(-1, 1)

        # 确定聚类的数量，这里可以根据实际情况调整
        kmeans = DBSCAN(eps=eps, min_samples=min_samples).fit(data)
        labels = kmeans.labels_
        t = [x for x in labels if x != -1]

        # 统计每个聚类中的数据点数量
        cluster_counts = np.bincount(t)
        if cluster_counts.shape[0] > 0:
            most_common_cluster_index = np.argmax(cluster_counts)
            most_common_cluster_data = data[labels == most_common_cluster_index]
            # 计算该聚类的均值
            mean_value = np.mean(most_common_cluster_data)
        else:
            mean_value = np.mean(data)
        return mean_value
    

    def xy_cluster_and_get_mean(self, data, eps=1.0, min_samples=2):
        all_points = np.array(data).reshape([len(data), -1])
        # 确定聚类的数量，这里可以根据实际情况调整
        kmeans = DBSCAN(eps=eps, min_samples=min_samples).fit(all_points)
        labels = kmeans.labels_
        t = [x for x in labels if x != -1]

        # 统计每个聚类中的数据点数量
        all_points = all_points.reshape([len(data), -1, 2])
        cluster_counts = np.bincount(t)
        if cluster_counts.shape[0] > 0:
            most_common_cluster_index = np.argmax(cluster_counts)
            most_common_cluster_data = all_points[labels == most_common_cluster_index]
            # 计算该聚类的均值
            res_xy = np.mean(most_common_cluster_data, axis=0)
        else:
            res_xy = np.mean(all_points, axis=0)
        return res_xy
    

    def is_straight(self, line):
        points = list(line.coords)
        if len(points) < 3:
            return True
        for i in range(len(points) - 2):
            p1, p2, p3 = points[i], points[i + 1], points[i + 2]
            v1 = (p2[0] - p1[0], p2[1] - p1[1])
            v2 = (p3[0] - p2[0], p3[1] - p2[1])
            cross_product = v1[0] * v2[1] - v1[1] * v2[0]
            if cross_product!= 0:
                return False
        return True


    def max_vertical_distance_to_endpoints(self, line):
        start = Point(line.coords[0])
        end = Point(line.coords[-1])
        line_between_endpoints = LineString([start, end])
        max_distance = 0
        for point in line.coords[1:-1]:
            p = Point(point)
            distance = p.distance(line_between_endpoints)
            if distance > max_distance:
                max_distance = distance

        return max_distance
    

    def closest_line_distance(self, lines, point_xy):
        point = Point(point_xy)
        min_distance = math.inf
        closest_line = None

        for line in lines:
            distance = point.distance(line)
            if distance < min_distance:
                min_distance = distance
                closest_line = line

        is_straight = self.is_straight(closest_line)
        max_dist = 0 if is_straight else self.max_vertical_distance_to_endpoints(closest_line)

        start = np.array(closest_line.coords[0])
        end = np.array(closest_line.coords[-1])
        angle = math.atan2(end[1]-start[1], end[0]-start[0])
        angle_degrees = math.degrees(angle)

        line_dist = np.linalg.norm(end-start)

        return min_distance, closest_line, angle_degrees, is_straight, max_dist, line_dist
    

    def sort_perpendicular_points(self, points: list, start, end):
        """根据直线朝向从左到右排序垂直于直线的点"""
        line_direction_vector = [end[0] - start[0], end[1] - start[1]]

        def get_cross_product_value(point):
            """计算点与直线起点构成的向量和直线方向向量的叉积"""
            vector_start_to_point = (point[0] - end[0], point[1] - end[1])
            return line_direction_vector[0] * vector_start_to_point[1] - line_direction_vector[1] * \
                vector_start_to_point[0]

        return sorted(points, key=get_cross_product_value, reverse=True)


    def subprocess_trafficlight(self, gdf):
        num_array, var_array_cz, var_array_px = [], defaultdict(list), defaultdict(list)
        all_points = gdf['geometry'].apply(lambda x: (x.x, x.y)).tolist()
        mean_xy = np.mean(all_points, axis=0)
        ts_set = set(gdf['timestamp'])

        is_car_light = False
        is_single = False

        ang_total = gdf['heading'].tolist()
        heading_total = self.angle_cluster_and_get_mean(ang_total)

        # ang_delta_total = gdf['heading_theta_bv'].tolist()
        # ang_delta = self.angle_cluster_and_get_mean(ang_delta_total)

        min_distance, closest_line, link_angle, is_straight, max_dist, line_dist = self.closest_line_distance(self.links, mean_xy)
        if max_dist / line_dist < 0.02:
            is_straight = True

        #  单帧只识别出单点
        if len(ts_set) == len(all_points):
            # all_points = np.array(all_points)
            # # 确定聚类的数量，这里可以根据实际情况调整
            # kmeans = DBSCAN(eps=1.0, min_samples=2).fit(all_points)
            # labels = kmeans.labels_
            # t = [x for x in labels if x != -1]

            # # 统计每个聚类中的数据点数量
            # cluster_counts = np.bincount(t)
            # if cluster_counts.shape[0] > 0:
            #     most_common_cluster_index = np.argmax(cluster_counts)
            #     most_common_cluster_data = all_points[labels == most_common_cluster_index]
            #     # 计算该聚类的均值
            #     res_xy = np.mean(most_common_cluster_data, axis=0)
            # else:
            #     res_xy = np.mean(all_points, axis=0)

            res_xy = self.xy_cluster_and_get_mean(all_points, eps=1.0, min_samples=2)

            if is_straight:
                angle_diff = (heading_total - link_angle) % 180
                if np.minimum(np.abs(angle_diff), 180 - np.abs(angle_diff)) < 30:
                    is_car_light = True
                    is_single = True

            return res_xy, link_angle+180, is_car_light, is_single, ang_total, len(ts_set)

        # 单帧识别出多点
        n_cz, n_px = 0, 0
        is_ref = False
        angle_diff = (heading_total - link_angle) % 180
        angle_diff = np.minimum(np.abs(angle_diff), 180 - np.abs(angle_diff))
        kheading_ref = None
        if is_straight and angle_diff < 30:
            kheading_ref = link_angle - 90
            is_ref = True
            # 求拟合直线的表达式
            if abs(kheading_ref) % 180 < 5 or abs(kheading_ref) % 180 > 175:
                A_ref, B_ref = 0, -1
            elif 85 < abs(kheading_ref) % 180 < 95:
                A_ref, B_ref = 1, 0
            else:
                k = math.tan(kheading_ref/180*np.pi)
                A_ref, B_ref = k, -1

        for ts in ts_set:
            tmp_df = gdf[gdf['timestamp'] == ts]
            points = tmp_df['geometry'].apply(lambda x: (x.x, x.y)).tolist()
            if len(points) <= 1:
                continue

            # 提取点集中的 x 和 y 坐标
            points = np.array(points)
            x = points[:, 0]
            y = points[:, 1]

            if is_ref:
                kheading = kheading_ref
                A, B = A_ref, B_ref
            else:
                kheading = self.angle_cluster_and_get_mean(tmp_df['heading'].tolist()) - 90
                # 求拟合直线的表达式
                if abs(kheading) % 180 < 5 or abs(kheading) % 180 > 175:
                    A, B = 0, -1
                elif 85 < abs(kheading) % 180 < 95:
                    A, B = 1, 0
                else:
                    k = math.tan(kheading/180*np.pi)
                    A, B = k, -1
            
            CList = A * x + B * y
            C_prime = -self.angle_cluster_and_get_mean(CList, eps=1.0, min_samples=2)
            kb = [A, B, C_prime]

            # 计算到拟合直线的距离均值
            distances_prime = np.abs(A * x + B * y + C_prime) / np.sqrt(A ** 2 + B ** 2)
            variance = np.mean(distances_prime)
            if variance < 3:
                n_cz += 1
                num_pts = tmp_df.shape[0]
                if num_pts in [5, 7]:
                    num_pts -= 1
                num_array.append(num_pts)
                var_array_cz[num_pts].append((variance, kheading, ts, kb))

        if n_cz >= n_px:
            maxn = max(num_array)
            tmp = var_array_cz[maxn]
            if len(tmp) in [1, 2]:
                if maxn-1 >= 0 and len(var_array_cz[maxn-1]) > 2:
                    tmp = var_array_cz[maxn-1]
                    maxn = maxn-1
                elif maxn-2 >= 0 and len(var_array_cz[maxn-2]) > 2:
                    tmp = var_array_cz[maxn-2]
                    maxn = maxn-2
        else:  # 拟合直线 更多是不垂直于heading的，判断其不属于 多灯组信号杆
            is_single = True
            is_car_light = False

            ############################   按照方差大的方向排序，丢到对应点集中取均值   ############################
            res_xy = np.mean(all_points, axis=0)
            ang_total = gdf['heading'].apply(lambda x: x if x > 0 else x + 360).tolist()
            heading_total = np.mean(ang_total)
            heading_total = heading_total if heading_total < 180 else heading_total - 360
            ############################   未修改   ############################

            return res_xy[np.newaxis, :], heading_total, is_car_light, is_single, ang_total, 0
            # tmp = sorted(var_array_px[maxn], key=lambda x: x[1])

        if len(tmp) == 0:
            res_xy = np.mean(all_points, axis=0)
            return res_xy[np.newaxis, :], heading_total, is_car_light, is_single, ang_total, 0
        
        # 新增
        k_headings, ts = [], []
        for i in range(len(tmp)):
            k_headings.append(tmp[i][1])
            ts.append(tmp[i][2])
        if is_ref:
            k_heading = kheading_ref
            A, B = A_ref, B_ref
        else:
            k_heading = self.angle_cluster_and_get_mean(k_headings)
            if abs(k_heading) % 180 < 5 or abs(k_heading) % 180 > 175:
                A, B = 0, -1
            elif 85 < abs(k_heading) % 180 < 95:
                A, B = 1, 0
            else:
                k = math.tan(k_heading/180*np.pi)
                A, B = k, -1

        filtered_gdf = gdf[gdf['timestamp'].isin(ts)]
        points = filtered_gdf['geometry'].apply(lambda x: (x.x, x.y)).tolist()
        points = np.array(points)
        # 提取点集中的 x 和 y 坐标
        x = points[:, 0]
        y = points[:, 1]

        # if maxn in [4, 5, 6, 7]:

        CList = A * x + B * y
        C = -self.angle_cluster_and_get_mean(CList, eps=1.0, min_samples=2)
        rad = np.radians(k_heading+90)
        end = np.array([np.cos(rad), np.sin(rad)])
        start = np.array([0, 0])

        res_xy, res_c = [], []
        for ti in range(len(tmp)):
            tmp_df = gdf[gdf['timestamp'] == tmp[ti][2]]
            tmp_xy = tmp_df['geometry'].apply(lambda x: (x.x, x.y)).tolist()
            tmp_xy = np.array(tmp_xy)
            x_p = (B ** 2 * tmp_xy[:, 0] - A * B * tmp_xy[:, 1] - A * C) / (A ** 2 + B ** 2)
            y_p = (A ** 2 * tmp_xy[:, 1] - A * B * tmp_xy[:, 0] - B * C) / (A ** 2 + B ** 2)

            if np.linalg.norm(np.array([x_p[0], y_p[0]]) - np.array([tmp_xy[0, 0], tmp_xy[0, 1]])) > 3:
                continue

            projection_points = np.column_stack((x_p, y_p))
            projection_points = self.sort_perpendicular_points(projection_points, start, end)
            arr = np.array(projection_points)

            # if len(projection_points) in [4, 6]:
            #     arr1, arr2 = arr[0::2], arr[1::2]
            #     avg_arr = (arr1 + arr2) / 2
            #     projection_points = avg_arr.tolist()
            # elif len(projection_points) in [5, 7]:
            #     if np.linalg.norm(arr[0]-arr[1]) <= np.linalg.norm(arr[-2]-arr[-1]):
            #         arr1, arr2 = arr[0::2], arr[1::2]
            #         avg_arr = (arr1 + arr2) / 2
            #         projection_points = avg_arr.tolist()
            #     else:
            #         arr1, arr2 = arr[1::2], arr[2::2]
            #         avg_arr = (arr1 + arr2) / 2
            #         projection_points = avg_arr.tolist()

            if len(projection_points) > 3:
                proj_origin_pts = self.sort_perpendicular_points(tmp_xy, start, end)
                proj_origin_arr = np.array(proj_origin_pts)

                if len(projection_points) in [4, 6]:
                    key_arr = proj_origin_arr[0::2]
                elif len(projection_points) in [5, 7]:
                    if np.linalg.norm(arr[0]-arr[1]) <= np.linalg.norm(arr[-2]-arr[-1]):
                        key_arr = proj_origin_arr[0::2]
                    else:
                        key_arr = proj_origin_arr[1::2]

                CList = A * key_arr[:, 0] + B * key_arr[:, 1]
                C = -np.mean(CList)
                x_p = (B ** 2 * tmp_xy[:, 0] - A * B * tmp_xy[:, 1] - A * C) / (A ** 2 + B ** 2)
                y_p = (A ** 2 * tmp_xy[:, 1] - A * B * tmp_xy[:, 0] - B * C) / (A ** 2 + B ** 2)
                projection_points = np.column_stack((x_p, y_p))
                projection_points = self.sort_perpendicular_points(projection_points, start, end)
                arr = np.array(projection_points)

            if len(projection_points) in [5, 7]:
                if np.linalg.norm(arr[0]-arr[1]) <= np.linalg.norm(arr[-2]-arr[-1]):
                    projection_points = arr[:-1].tolist()
                else:
                    projection_points = arr[1:].tolist()

            res_xy.append(projection_points)
        
        # 新增
        if len(res_xy) == 0:
            res_xy = np.mean(all_points, axis=0)
            return res_xy[np.newaxis, :], heading_total, is_car_light, is_single, ang_total, 0

        angle = k_heading
        # res_xy = np.mean(np.array(res_xy), axis=0)  #  均值xy
        res_xy = self.xy_cluster_and_get_mean(res_xy, eps=1.0, min_samples=2)  #  众数均值xy
        is_car_light = True
        num_pts = res_xy.shape[0]

        # 新增
        if np.linalg.norm(res_xy[0]-res_xy[-1]) < 2.0:
            res_xy = np.mean(res_xy, axis=0)
            is_single = True
            is_car_light = False
            return res_xy[np.newaxis, :], angle-90, is_car_light, is_single, ang_total, len(tmp)
        else:
            return res_xy, angle-90, is_car_light, is_single, ang_total, len(tmp)


    def run(self, task_folder: str, meta_json: str, out_shp: str):
        self.task_folder = task_folder
        self.task_data_folder = os.path.join(self.task_folder, "data")
        self.bev_mappping_folder = os.path.join(self.task_folder, "trafficlight_processing")
        self.bev_mappbuild_folder = os.path.join(self.task_folder, "bev_mapbuild_out")
        
        # log.info("====== bev_trafficlight_tracking: 数据载入 ======")
        self.load_data_set(meta_json)

        if self.proc_gdf is not None:
            # log.info("====== bev_trafficlight_tracking: 开始处理 ======")
            self.bev_trafficlight_tracking()
        else:
            # log.info("  frame_metainfo:{}".format(meta_json))
            log.info("====== 该数据未找到符合条件的信号灯:{} ======".format(meta_json))


base_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross/"
default_input_json = os.path.join(base_dir, "11002417915690/PL0169_event_trafficlight_status_event_20240902-101706_0/223126")
parser = argparse.ArgumentParser()
parser.add_argument('--task_folder', type=str, default=default_input_json)
parser.add_argument('--input', type=str, default=default_input_json)
parser.add_argument('--output', type=str, default=default_input_json)
if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.task_folder, f"log/run_bev_trafficlight.log"))
    error_code = 0
    # log.info("============================================")
    # log.info("  Hello TrafficlightTracking")
    # log.info("  frame_metainfo:{}".format(args.input))
    # log.info("  output_shapefile:{}".format(args.output))
    # log.info("============================================")
    try:
        bev_tl_track: TrafficlightTracking = TrafficlightTracking()
        bev_tl_track.run(args.task_folder, args.input, args.output)
    except Exception as e:
        log.error("error:", e)
        error_code = 732  # "交通灯建模_异常退出"
    
    # log.info("完成 TrafficlightTracking")
    tl_proc_result = {"error_code": error_code}
    util.dict_to_json_file(os.path.join(args.task_folder, "trafficlight_processing/trafficlight_processing_result.json"), tl_proc_result)  # 保存mapping环境结果文件
