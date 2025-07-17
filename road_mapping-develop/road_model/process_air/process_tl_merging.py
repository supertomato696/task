import argparse
import os
import util
from util import *
from util import log
import pandas as pd
import geopandas as gpd
from sklearn.cluster import DBSCAN
from shapely.wkt import loads
pd.options.mode.chained_assignment = None

road_model_root_folder = os.path.abspath(__file__ + "/../../")
fsd_map_root_folder = os.path.abspath(__file__ + "/../../../")

sys.path.append(fsd_map_root_folder)
from preprocess.script.trail_data import TrailData, FrameData, json_file_to_dict

class TrafficlightMerging:
    def __init__(self) -> None:
        self.task_folder = ""
        self.task_data_folder = ""
        self.trail_dict: Dict[str, TrailData] = {}
        self.proc_gdf: gpd.GeoDataFrame = None

        # 任务框信息
        self.frame_id = None
        self.geofence_gpd = None

    def load_data_set(self, input_files):
        self.proc_gdf = gpd.GeoDataFrame(columns=[
            'id', 'geometry', 'cluster_xy', 'cluster_yaw', 'heading', 'is_car_light', 'is_single', 'is_L', 'num_pts'],
            crs='epsg:3857')

        # trail_json: dict = json_file_to_dict(meta_json)
        trail_ids = input_files.split('-')
        for i, in_file in enumerate(trail_ids):
            filepath = os.path.join(self.bev_mappping_folder, f"tl_tracking_res_{in_file}.gpkg")
            # filename = os.path.basename(filepath)
            if os.path.exists(filepath):
                gdf = gpd.read_file(filepath, layer='tracking_res').to_crs(epsg=3857)
                gdf['id'] = np.asarray(gdf['cluster_xy'].tolist()) * 100 + np.asarray(gdf['cluster_yaw'].tolist()) + int(in_file) * 10000
                self.proc_gdf = gpd.GeoDataFrame(pd.concat([gdf, self.proc_gdf], ignore_index=True))
                
                # log.info("[TrafficlightProcessing] 完成载入trail_data", f"tl_tracking_res_{in_file}.shp")
        
        self.proc_gdf.to_file(os.path.join(self.bev_mappping_folder, f"tl_tracking_res_gather.gpkg"), layer='tracking_res_gather', driver='GPKG')

        # 读取任务框范围和ID
        task_info_file = os.path.join(os.path.dirname(self.task_folder), 'task_info_demo.json')
        if os.path.exists(task_info_file):
            task_info: dict = json_file_to_dict(task_info_file)
            if 'task_geom_global' in task_info['middle']:
                polygon_geom = loads(task_info['middle']['task_geom_global'])
                self.geofence_gpd = gpd.GeoDataFrame(geometry=[polygon_geom], crs = "EPSG:4326").to_crs('EPSG:3857')
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

    
    def angle_cluster_and_get_mean(self, data, eps=10, min_samples=2, is_Ls=[0]):
        # 将数据转换为二维数组，因为KMeans需要二维输入
        data = np.array(data).reshape(-1, 1)
        if is_Ls[0] == 0:
            is_Ls = np.array([1] * data.shape[0])

        # 确定聚类的数量，这里可以根据实际情况调整
        kmeans = DBSCAN(eps=eps, min_samples=min_samples).fit(data)
        labels = kmeans.labels_
        t1 = [x for i, x in enumerate(labels) if x != -1 and is_Ls[i] != 0.1]
        t2 = [x for i, x in enumerate(labels) if x != -1]

        # 统计每个聚类中的数据点数量
        cluster_counts1 = np.bincount(t1)
        cluster_counts2 = np.bincount(t2)
        cluster_counts = cluster_counts1 if cluster_counts1.shape[0] > 0 else cluster_counts2

        if cluster_counts.shape[0] > 0:
            most_common_cluster_index = np.argmax(cluster_counts)
            most_common_cluster_data = data[labels == most_common_cluster_index].reshape([-1])
            # 计算该聚类的均值np.sum(angles * is_Ls) / is_Ls.sum()
            is_Ls = is_Ls[labels == most_common_cluster_index]
            mean_value = np.sum(most_common_cluster_data * is_Ls) / is_Ls.sum() # np.mean(most_common_cluster_data)
        else:
            most_common_cluster_index = np.argmax(is_Ls)
            mean_value = data[most_common_cluster_index][0] # np.sum(data * is_Ls) / is_Ls.sum() # np.mean(data)
        return mean_value
    

    def sort_perpendicular_points(self, points: list, start, end):
        """根据直线朝向从左到右排序垂直于直线的点"""
        line_direction_vector = [end[0] - start[0], end[1] - start[1]]

        def get_cross_product_value(point):
            """计算点与直线起点构成的向量和直线方向向量的叉积"""
            vector_start_to_point = (point[0] - end[0], point[1] - end[1])
            return line_direction_vector[0] * vector_start_to_point[1] - line_direction_vector[1] * \
                vector_start_to_point[0]

        return sorted(points, key=get_cross_product_value, reverse=True)


    def make_trafficlight_merging(self):

        # 根据is_car_light==1筛选数据
        filtered_gdf = self.proc_gdf[self.proc_gdf['is_car_light'] == 1]

        # 根据is_single将数据分为两组
        single_gdf = filtered_gdf[filtered_gdf['is_single'] == 1]
        multi_gdf = filtered_gdf[filtered_gdf['is_single'] == 0]

        # 定义一个函数来进行聚类
        def cluster_data(data):
            if len(data) == 0:
                return []
            # 提取xy坐标
            xy = np.asarray(data['geometry'].apply(lambda x: (x.x, x.y)).tolist())
            db = DBSCAN(eps=5, min_samples=1).fit(xy)
            labels = db.labels_
            data['cluster_label'] = labels
            return data.groupby('cluster_label')

        # 对两组数据分别进行聚类
        single_clusters = cluster_data(single_gdf)
        multi_clusters = cluster_data(multi_gdf)

        total_res_xy = []
        total_res_angle = []
        total_res_id = []
        total_res_Ls = []

        # 遍历所有聚类对象
        all_clusters = list(single_clusters) + list(multi_clusters)
        for i, (label, tmp_gdf) in enumerate(all_clusters):
            if label == -1:
                continue

            # 判断是否在任务框范围内
            if self.geofence_gpd is not None:
                result = gpd.sjoin(tmp_gdf, self.geofence_gpd, predicate='within')
                is_in = True
                # print(result.index, tmp_gdf.index, self.geofence_gpd.index)
                for index, row in tmp_gdf.iterrows():
                    if index not in result.index:
                        is_in = False
                        break
                if not is_in:
                    continue

            num_array, var_array_L, var_array_nL = [], defaultdict(list), defaultdict(list)
            ids = set(tmp_gdf['id'].tolist())

            # def transf(ang):
            #     for j in range(len(ang)):

            #         if abs(ang[j]) > 180:
            #             ang[j] = (abs(ang[j]) % 180) * ang[j] / abs(ang[j])

            #     return ang

            for tid in ids:
                txy = tmp_gdf[tmp_gdf['id'] == tid]['geometry'].apply(lambda x: (x.x, x.y)).tolist()
                is_Ls = np.array(tmp_gdf[tmp_gdf['id'] == tid]['is_L'].tolist())
                num_grps = np.array(tmp_gdf[tmp_gdf['id'] == tid]['num_pts'].tolist())
                num_pts = tmp_gdf[tmp_gdf['id'] == tid].shape[0]
                num_array.append(num_pts)
                if np.all(is_Ls==1):
                    var_array_L[num_pts].append([tmp_gdf[tmp_gdf['id'] == tid]['heading'].tolist(), txy, num_grps, is_Ls])
                else:
                    var_array_nL[num_pts].append([tmp_gdf[tmp_gdf['id'] == tid]['heading'].tolist(), txy, num_grps, is_Ls])

            # counts = np.bincount(num_array)
            # maxt = np.argmax(counts)

            if len(var_array_nL) > 0:
                maxn = max(var_array_nL.keys())
                tmp = var_array_nL[maxn]
            else:
                maxn = max(var_array_L.keys())
                tmp = var_array_L[maxn]

            # if len(tmp) == 0 or (len(var_array_nL)==0 and (maxn==1 or len(tmp)==1)):  // 限制: 仅有一条转弯轨迹识别到的灯会忽略
            if len(tmp) == 0 or (len(var_array_nL)==0 and maxn==1):
                continue

            angles = np.array([angle[0] - 90 for angle, _, _, _ in tmp])
            txy = np.array([ttxy for _, ttxy, _, _ in tmp])
            is_Ls_ori = np.array([l[0] for _, _, _, l in tmp])
            num_grps = np.array([n[0] for _, _, n, _ in tmp])
            # is_Ls[is_Ls == 1] = 0.1
            # is_Ls[is_Ls == 0] = 1
            # print(is_Ls)
            # print(txy.shape, angles.shape, num_grps.shape)
            k_headings = self.angle_cluster_and_get_mean(angles, is_Ls=num_grps) # np.sum(angles * is_Ls) / is_Ls.sum()

            # 提取点集中的 x 和 y 坐标
            points = txy.reshape([-1, 2])
            x = points[:, 0]
            y = points[:, 1]
            c_num_grps = num_grps[np.newaxis, :].repeat(txy.shape[1], axis=0).flatten()

            # print(txy.shape, angles.shape, num_grps.shape, 'mid0.5', points.shape, c_num_grps.shape)

            if abs(k_headings) % 180 < 5 or abs(k_headings) % 180 > 175:
                A, B = 0, -1
            elif 85 < abs(k_headings) % 180 < 95:
                A, B = 1, 0
            else:
                k = math.tan(k_headings/180*np.pi)
                A, B = k, -1
            CList = A * x + B * y
            C = -self.angle_cluster_and_get_mean(CList, eps=1.0, min_samples=2, is_Ls=c_num_grps)
            # C = -np.mean(A * x + B * y)
            rad = np.radians(k_headings+90)
            end = np.array([np.cos(rad), np.sin(rad)])
            start = np.array([0, 0])

            # print(txy.shape, angles.shape, num_grps.shape, 'mid1')

            res_xy, res_n = [], []
            for ti in range(len(angles)):
                tmp_xy = np.array(txy[ti])
                x_p = (B ** 2 * tmp_xy[:, 0] - A * B * tmp_xy[:, 1] - A * C) / (A ** 2 + B ** 2)
                y_p = (A ** 2 * tmp_xy[:, 1] - A * B * tmp_xy[:, 0] - B * C) / (A ** 2 + B ** 2)

                if np.linalg.norm(np.array([x_p[0], y_p[0]]) - np.array([tmp_xy[0, 0], tmp_xy[0, 1]])) > 3:
                    continue

                projection_points = np.column_stack((x_p, y_p))
                projection_points = self.sort_perpendicular_points(projection_points, start, end)
                arr = np.array(projection_points)

                if len(projection_points) in [4, 6]:
                    arr1, arr2 = arr[0::2], arr[1::2]
                    avg_arr = (arr1 + arr2) / 2
                    projection_points = avg_arr.tolist()
                elif len(projection_points) in [5, 7]:
                    if np.linalg.norm(arr[0]-arr[1]) <= np.linalg.norm(arr[-2]-arr[-1]):
                        arr1, arr2 = arr[0::2], arr[1::2]
                        avg_arr = (arr1 + arr2) / 2
                        projection_points = avg_arr.tolist()
                    else:
                        arr1, arr2 = arr[1::2], arr[2::2]
                        avg_arr = (arr1 + arr2) / 2
                        projection_points = avg_arr.tolist()

                res_xy.append(projection_points)
                res_n.append(num_grps[ti])
            
            # print(txy.shape, angles.shape, num_grps.shape, 'mid2')

            tmp_Ls = np.array(res_n).reshape([-1, 1, 1])
            res_xy = np.sum(np.array(res_xy) * tmp_Ls, axis=0) / tmp_Ls.sum()
            # res_angle_radians = math.atan2(-A, B)
            # res_angle = np.degrees(res_angle_radians)

            total_res_xy.append(res_xy)
            l_orient = k_headings+90 + 360 # res_angle + 90
            # l_orient = 180 + l_orient if l_orient < 0 else 180 + l_orient
            # res_dict['HEADING'].append(l_orient % 360)
            total_res_angle += [l_orient % 360] * res_xy.shape[0]
            total_res_id += [6*(10**8)+i+1] * res_xy.shape[0]

            if np.all(is_Ls_ori == 0):
                total_res_Ls += [0] * res_xy.shape[0]
            elif np.all(is_Ls_ori == 1):
                total_res_Ls += [2] * res_xy.shape[0]
            else:
                total_res_Ls += [1] * res_xy.shape[0]

            # print(txy.shape, angles.shape, num_grps.shape, 'finished')
        if len(total_res_xy) == 0:
            empty_gdf = gpd.GeoDataFrame(geometry=[], crs='epsg:3857', columns=['geometry', 'heading', 'group_id', 'is_L', 'ID'])
            empty_gdf.to_file(os.path.join(self.bev_mappping_folder, f"tl_res_merging.shp"))
            empty_gdf = gpd.GeoDataFrame(geometry=[], crs='epsg:3857', columns=['geometry', 'HEADING', 'TYPE', 'LANE_ID', 'C_WALK_ID', 'JUNC_ID', 'ALG_ID', 'VERSION', 'ID'])
            # if os.path.exists(os.path.join(self.bev_mappbuild_folder, f"export_to_origin_shp/traffic_light.shp")):
            #     os.remove(os.path.join(self.bev_mappbuild_folder, f"export_to_origin_shp/traffic_light.shp"))
            # empty_gdf.to_file(os.path.join(self.bev_mappbuild_folder, f"export_to_origin_shp/traffic_light.shp"))

            if os.path.exists(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp")):
                os.remove(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp"))
            empty_gdf.to_file(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp"))
            return

        out_xy = np.vstack(total_res_xy)
        out_angle = np.array(total_res_angle)
        out_ids = np.array(total_res_id)
        out_Ls = np.array(total_res_Ls)
        new_points = [Point(x, y) for x, y in out_xy]

        # 创建一个新的GeoDataFrame来存储新点
        new_gdf = gpd.GeoDataFrame(geometry=new_points, crs='epsg:3857').to_crs('epsg:4326')
        new_gdf['heading'] = out_angle
        new_gdf['group_id'] = out_ids
        new_gdf['is_L'] = out_Ls
        new_gdf['ID'] = np.arange(len(new_points)) + 6*(10**8)
        new_gdf.to_file(os.path.join(self.bev_mappping_folder, f"tl_res_merging.shp"))

        # 按照规格输出结果文件
        new_gdf = gpd.GeoDataFrame(geometry=new_points, crs='epsg:3857').to_crs('epsg:4326')
        new_gdf['HEADING'] = out_angle.round(2)
        new_gdf['TYPE'] = [np.nan]*len(new_points)
        new_gdf['LANE_ID'] = ['']*len(new_points)
        new_gdf['C_WALK_ID'] = ['']*len(new_points)
        new_gdf['JUNC_ID'] = ['']*len(new_points)
        new_gdf['ALG_ID'] = [np.nan]*len(new_points) if self.frame_id is None else [int(self.frame_id)]*len(new_points)
        new_gdf['VERSION'] = [self.version_string]*len(new_points)
        new_gdf['ID'] = np.arange(len(new_points)) + 6*(10**8)
        # if os.path.exists(os.path.join(self.bev_mappbuild_folder, f"export_to_origin_shp/traffic_light.shp")):
        #     os.remove(os.path.join(self.bev_mappbuild_folder, f"export_to_origin_shp/traffic_light.shp"))
        # new_gdf.to_file(os.path.join(self.bev_mappbuild_folder, f"export_to_origin_shp/traffic_light.shp"))

        if os.path.exists(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp")):
            os.remove(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp"))
        new_gdf.to_file(os.path.join(self.bev_mappbuild_folder, f"export_to_shp/traffic_light.shp"))
        
        # log.info("finish trafficlight_merging")

    def run(self, task_folder: str, input_files: str, out_shp: str):
        self.task_folder = task_folder
        self.task_data_folder = os.path.join(self.task_folder, "data")
        self.bev_mappping_folder = os.path.join(self.task_folder, "trafficlight_processing")
        self.bev_mappbuild_folder = os.path.join(self.task_folder, "bev_mapbuild_out")
        
        log.info("====== bev_trafficlight_merging: 数据载入 ======")
        self.load_data_set(input_files)

        if self.proc_gdf is not None:
            log.info("====== bev_trafficlight_merging: 开始处理 ======")
            self.make_trafficlight_merging()


base_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross/"
default_input_json = os.path.join(base_dir, "11002417915690/PL0169_event_trafficlight_status_event_20240902-101706_0/223126")
parser = argparse.ArgumentParser()
parser.add_argument('--task_folder', type=str, default=default_input_json)
parser.add_argument('--input', type=str, default='1-2')
parser.add_argument('--output', type=str, default=default_input_json)
if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.task_folder, f"log/run_bev_trafficlight.log"))
    error_code = 0
    log.info("============================================")
    log.info("  Hello TrafficlightMerging")
    log.info("  task_folder:{}".format(args.task_folder))
    log.info("  IDs of tracking shp:{}".format(args.input))
    log.info("  output_shapefile:{}".format(args.output))
    log.info("============================================")
    try:
        bev_tl_merge: TrafficlightMerging = TrafficlightMerging()
        bev_tl_merge.run(args.task_folder, args.input, args.output)
    except Exception as e:
        log.error("error:", e)
        error_code = 732  # "交通灯建模_异常退出"
    
    # log.info("完成 TrafficlightMerging")
    tl_proc_result = {"error_code": error_code}
    util.dict_to_json_file(os.path.join(args.task_folder, "trafficlight_processing/trafficlight_processing_result.json"), tl_proc_result)  # 保存mapping环境结果文件
