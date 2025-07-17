import open3d as o3d
import os,sys
import heapq

fsd_map_root_folder = os.path.abspath(__file__ + "/../../../")
sys.path.append(fsd_map_root_folder)
from preprocess.script.trail_data import TrailData, FrameData
from data_io.script.bev_label_map import *

# from trail_data import TrailData, FrameData
import util
from util import *
from util import log
import argparse
import matplotlib
import copy
from tqdm import tqdm
import shapely
from shapely.geometry import LineString, Polygon, Point
import scipy
from enum import Enum
from shapely import affinity
from scipy.optimize import linear_sum_assignment
import networkx
from sklearn.cluster import DBSCAN
from itertools import combinations
from scipy.linalg import lstsq
mapping_root_folder = os.path.abspath(__file__ + "/../../")


# TYPE_DICT = {0: 'other_unset_type', 1: "other_solid",  2: "other_dashed", 3: 'other_double_solid', 4: 'other_double_dashed',
#              5: 'other_left_dashed_right_solid', 6: 'other_left_solid_right_dashed', 7: 'white_unset_type', 8: 'white_solid', 9: 'white_dashed',
#              10: 'white_double_solid', 11: 'white_double_dashed', 12: 'white_left_dashed_right_solid', 13: 'white_left_solid_right_dashed', 14: 'yellow_unset_type',
#              15: 'yellow_solid', 16: 'yellow_dashed', 17: 'yellow_double_solid', 18: 'yellow_double_dashed', 19: 'yellow_left_dashed_right_solid',
#              20: 'yellow_left_solid_right_dashed', 21: 'unknow_road_boundary', 22: 'road_boundary_curb', 23: 'road_boundary_fence', 24: 'road_boundary_wall',
#              25: 'road_boundary_surface', 26: 'road_boundary_ditch', 27: 'road_boundary_obstacle', 28: 'stopline', 29: 'crosswalk',
#              30: 'polygon_unknow_kind', 31: 'polygon_flowerbed', 32: 'polygon_sentry_box', 33: 'polygon_physical_safe_island', 34: 'polygon_linear_safe_island',
#              35: 'polygon_6', 36: 'lane_center', 37: 'junction', 38: 'arrow_unknown', 39: 'arrow_left',
#              40: 'arrow_forward', 41: 'arrow_right', 42: 'arrow_left_and_forward', 43: 'arrow_right_and_forward', 44: 'arrow_left_and_right',
#              45: 'arrow_u_turn', 46: 'arrow_u_turn_and_forward', 47: 'arrow_u_turn_and_left', 48: 'arrow_merge_left', 49: 'arrow_merge_right',
#              50: 'crosswalk_notice', 51: 'speed_limit_low', 52: 'speed_limit_high', 53: 'arrow_no_left_turn', 54: 'arrow_no_right_turn',
#              55: 'arrow_no_u_turn', 56: 'arrow_forward_and_left_and_right', 57: 'arrow_forward_and_u_turn_and_left', 58: 'arrow_right_and_u_turn', 59: 'marking_text',
#              60: 'marking_time', 61: 'check_following_distance', 62: 'stopto_give_way', 63: 'slowdown_to_give_way', 64: 'marking_nets', }

# TYPE_DICT = {0: 'other_unset_type',
            
#              1: "lane_unkown_unkown",  2: "lane_unkown_dash", 3: 'lane_unkown_solid', 4: 'lane_white_unkown',
#              5: 'lane_white_dash', 6: 'lane_white_solid', 7: 'lane_yellow_unkown', 8: 'lane_yellow_dash', 9: 'lane_yellow_solid',

#              10: 'road_boundary_unkown_unkown', 11: 'road_boundary_normal', 12: 'road_boundary_ditch', 13: 'road_boundary_curb', 14: 'road_boundary_obstacle',
             
#             #  15: 'other_unset_type',
#              16: 'crosswalk', 
#              17: 'junction', 
             
#              18: 'lane_center_unkown', 19: 'lane_center_normal', 20: 'lane_center_accident', 21: 'lane_center_bus', 22: 'lane_center_no_vechile', 23: 'lane_center_fee', 
#              24: 'lane_center_change_lane', 25: 'lane_center_chaoxi', 26: 'lane_center_left_and_right', 27: 'lane_center_not_full',
             
#              28: 'arrow_unkown', 29: 'arrow_forward',
#              30: 'arrow_left', 31: 'arrow_right', 32: 'arrow_left_and_u_turn', 33: 'arrow_right_and_u_turn', 34: 'arrow_left_heliu',
#              35: 'arrow_right_heliu', 36: 'arrow_daoliu',
             
#              37: 'stopline_unkown', 38: 'stopline_normal', 39: 'stopline_left',
#              40: 'stopline_u_turn', 41: 'stopline_right', 42: 'stopline_forward',
             
#              47: 'traffic_light_horizon', 48: 'traffic_light_vertical', 49: 'traffic_light_single',
#             }

# TYPE_NUM = len(TYPE_DICT)


# class BevLineTrackCategory(Enum):
#     other = 0
#     lane_line = 1
#     stop_line = 2
#     center_line = 3
#     road_boundary = 4
#     crosswalk = 5
#     junction = 6
#     arrow = 7


class BEVLine:
    pass


class BEVLine:
    @classmethod
    def init(cls, line_id: str, frame_line_id: int, track_id: int, type_id: int, bev_score: float, points: list, T_world_veh: np.ndarray, data_type: str):
        instance = cls()
        instance.line_id: str = line_id
        instance.frame_id: str = line_id.split("_")[0]+"_"+line_id.split("_")[1]
        instance.trail_id: str = line_id.split("_")[0]
        instance.frame_line_id: int = frame_line_id
        instance.track_id: int = track_id
        instance.bev_score: float = bev_score
        instance.type_id: int = type_id # raw_bev_id
        instance.data_type = data_type
        instance.type_name: str = get_combine_name(type_id, data_type)
        # instance.category: str = cls.type_id_to_category(type_id)
        instance.category: str = get_bev_track_category(type_id, data_type)
        instance.points: List[np.ndarray] = [np.array(tup) for tup in points]
        instance.points_normal: List[np.ndarray] = curve_points_to_normal_vectors(instance.points)
        if instance.category != BevLineTrackCategory.split_merge:
            instance.linestring: LineString = LineString([(x, y) for x, y in points])
        instance.T_world_veh = T_world_veh
        if instance.category in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line, BevLineTrackCategory.road_boundary):
            instance.cubic_curve_fit()
        instance.tag = []
        return instance

    @classmethod
    def transform(cls, bevline, T_new_old):
        newbevline: cls = copy.deepcopy(bevline)
        R = util.Tab_to_Rab(T_new_old)
        t = util.Tab_to_tab(T_new_old).reshape(1, 3)
        T43 = (np.r_[R, t]).flatten()
        newbevline.linestring = affinity.affine_transform(bevline.linestring, T43)
        newbevline.points = [np.array(tup) for tup in newbevline.linestring.coords]
        newbevline.points_directions = points_tangent_directions(newbevline.points)
        if newbevline.category in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line, BevLineTrackCategory.road_boundary):
            newbevline.cubic_curve_fit()
        return newbevline

    # @staticmethod
    # def type_id_to_category(type_id) -> BevLineTrackCategory:
    #     if type_id in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20):
    #         return BevLineTrackCategory.lane_line  # line
    #     elif type_id in (21, 22, 23, 24, 25, 26, 27):
    #         return BevLineTrackCategory.road_boundary  # road_boundary
    #     elif type_id in (28,):
    #         return BevLineTrackCategory.stop_line  # stop
    #     elif type_id in (29,):
    #         return BevLineTrackCategory.crosswalk  # corsswalk
    #     elif type_id in (38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 53, 54, 55, 56, 57, 58):
    #         return BevLineTrackCategory.arrow  # arrow
    #     elif type_id in (36,):
    #         return BevLineTrackCategory.center_line  # lane_center
    #     elif type_id in (37,):
    #         return BevLineTrackCategory.junction  # junction
    #     else:
    #         return BevLineTrackCategory.other

    # @staticmethod
    # def type_id_to_category_qzc(type_id, data_type) -> BevLineTrackCategory:
    #     if type_id in (1, 2, 3, 4, 5, 6, 7, 8, 9):
    #         return BevLineTrackCategory.lane_line  # line
    #     elif type_id in (10, 11, 12, 13, 14):
    #         return BevLineTrackCategory.road_boundary  # road_boundary
    #     elif type_id in (37, 38, 39, 40, 41, 42):
    #         return BevLineTrackCategory.stop_line  # stop
    #     elif type_id in (16,):
    #         return BevLineTrackCategory.crosswalk  # corsswalk
    #     elif type_id in (28, 29, 30, 31, 32, 33, 34, 35, 36):
    #         return BevLineTrackCategory.arrow  # arrow
    #     elif type_id in (18, 19, 20, 21, 22, 23, 24, 25, 26, 27):
    #         return BevLineTrackCategory.center_line  # lane_center
    #     elif type_id in (17,):
    #         return BevLineTrackCategory.junction  # junction
    #     else:
    #         return BevLineTrackCategory.other
        
    def __init__(self) -> None:
        self.data_type: str = "BYD_BEV"
        self.track_id: int = 0
        self.frame_line_id: int = 0
        self.line_id: str = ""
        self.frame_id: str = ""
        self.trail_id: str = ""
        self.bev_score: float = 0
        self.type_id: int = 0
        self.type_name: str = "empty"
        self.category: BevLineTrackCategory = BevLineTrackCategory.other

        self.points: List[np.ndarray] = []
        self.points_normal: List[np.ndarray] = []
        self.linestring: LineString = LineString()

        self.T_world_veh: np.ndarray = np.array([])
        self.cubic_fit_curvature_mean: float = 0
        self.cubic_fit_curvature_max: float = 0
        self.cubic_fit_error_mean: float = 0
        self.cubic_fit_error_max: float = 0
        self.cubic_fit_switch_xy: bool = False
        self.cubic_fit_coeff: list = []
        self.tag = []

    def to_image_points(self, max_x, min_x, max_y, min_y, pixel_size) -> list:
        width = int(round((max_y-min_y)/pixel_size))
        height = int(round((max_x-min_x)/pixel_size))
        img_points = []
        for point in self.linestring.coords:
            x = int(round((max_y-point[1])/pixel_size))
            y = int(round((max_x-point[0])/pixel_size))
            img_points.append([x, y])
            if (x < 0 or y < 0 or x >= width or y >= height):
                continue
        return img_points

    def cal_curvature(self, x):
        # 使用拟合系数生成拟合曲线
        poly = np.poly1d(self.cubic_fit_coeff)

        # 计算一阶导数和二阶导数
        poly_first_derivative = np.polyder(poly, 1)  # 一阶导数
        poly_second_derivative = np.polyder(poly, 2)  # 二阶导数
        x_eval = np.linspace(x.min(), x.max(), 15) # 目前去了前后15米，按照2米一个点就采样计算就行
        # 计算一阶导数和二阶导数的值
        first_derivative_values = poly_first_derivative(x_eval)
        second_derivative_values = poly_second_derivative(x_eval)

        # 计算曲率
        curvature = np.abs(second_derivative_values) / (1 + first_derivative_values**2)**1.5
        self.cubic_fit_curvature_mean = np.mean(curvature)  # 平均曲率
        self.cubic_fit_curvature_max = np.max(curvature)  # 最大曲率

    def cubic_curve_fit(self):
        # if self.track_id == "41":
        #     print(self.line_id)
        x = np.array(self.points)[:, 0]
        y = np.array(self.points)[:, 1]
        cubic_fit_coeff_1 = np.polyfit(x, y, 3)
        y_pred_1 = np.polyval(cubic_fit_coeff_1, x)
        error_list = np.abs(y_pred_1 - y)
        cubic_fit_error_mean_1 = np.sum(error_list)/len(y)
        cubic_fit_error_max_1 = error_list.max()

        x = np.array(self.points)[:, 1]
        y = np.array(self.points)[:, 0]
        cubic_fit_coeff_2 = np.polyfit(x, y, 3)
        y_pred_2 = np.polyval(cubic_fit_coeff_2, x)
        error_list = np.abs(y_pred_2 - y)
        cubic_fit_error_mean_2 = np.sum(error_list)/len(y)
        cubic_fit_error_max_2 = error_list.max()

        if cubic_fit_error_mean_1 <= cubic_fit_error_mean_2:
            self.cubic_fit_coeff = cubic_fit_coeff_1
            self.cubic_fit_error_mean = cubic_fit_error_mean_1
            self.cubic_fit_error_max = cubic_fit_error_max_1
            self.cubic_fit_switch_xy = False

            x = np.array(self.points)[:, 0]
            # y = np.array(self.points)[:, 1]
            self.cal_curvature(x)

            # self.points: List[np.ndarray] = []
            self.points = list(np.column_stack((x, y_pred_1)))

        else:
            self.cubic_fit_coeff = cubic_fit_coeff_2
            self.cubic_fit_error_mean = cubic_fit_error_mean_2
            self.cubic_fit_error_max = cubic_fit_error_max_2
            self.cubic_fit_switch_xy = True

            x = np.array(self.points)[:, 1]
            # y = np.array(self.points)[:, 0]
            self.cal_curvature(x)

            # self.points: List[np.ndarray] = []
            self.points = list(np.column_stack((y_pred_2, x)))


    def quadratic_fit(self):
        def func(x, a, b, c):
            return a * x ** 2 + b * x + c
        x = self.points[:, 0]
        y = self.points[:, 1]
        popt, pcov = scipy.optimize.curve_fit(func, x, y)
        score = np.sum((y - func(x, *popt)) ** 2)
        self.quad_fit_coeff = popt
        self.quad_fit_scroe = score

    def info_str(self) -> str:
        tag = ""
        for one in self.tag:
            tag += f"{one}|"
        s = f"{self.frame_line_id}|{self.track_id}|{self.type_id}-{self.type_name}|s{round(self.bev_score,1)}|e{round(self.cubic_fit_error_mean,2)},{round(self.cubic_fit_error_max,2)},{round(self.cubic_fit_curvature_max,2)}|{round(self.linestring.length,1)}m|{tag}"
        s.strip("|")
        return s

    def distance_to_other_bevline(self, other_bevline: BEVLine) -> float:
        total_distance = 0
        num_points = 0
        [xid, yid] = [1, 0] if self.cubic_fit_switch_xy else [0, 1]
        for point in other_bevline.linestring.coords:
            total_distance += abs(np.polyval(self.cubic_fit_coeff, point[xid])-point[yid])
            num_points += 1
        [xid, yid] = [1, 0] if other_bevline.cubic_fit_switch_xy else [0, 1]
        for point in self.linestring.coords:
            total_distance += abs(np.polyval(other_bevline.cubic_fit_coeff, point[xid])-point[yid])
            num_points += 1
        average_distance = total_distance / num_points
        return average_distance
    
    def min_distance_to_other_bevline(self, other_bevline: BEVLine) -> float:
        min_distance = 1001
        d = self.linestring.distance(other_bevline.linestring)
        if d < min_distance:
            min_distance = d

        return min_distance

    def iou_to_other_bevline(self, other_bevline: BEVLine) -> float:
        [xid, yid] = [1, 0] if self.cubic_fit_switch_xy else [0, 1]
        l1, r1 = sorted([self.points[0][xid], self.points[-1][xid]])
        l2, r2 = sorted([other_bevline.points[0][xid], other_bevline.points[-1][xid]])
        intersection = max(0, min(r1, r2) - max(l1, l2))
        union = max(r1, r2) - min(l1, l2)
        iou = intersection / union
        return iou


class BEVFrame:
    pass


class BEVFrame:
    def __init__(self, frame: FrameData, trail_id: str, data_type: str) -> None:
        self.data_type = data_type
        self.frame = frame
        self.stamp_ms = frame.stamp_ms
        self.trail_id = trail_id
        self.frame_id = f"{frame.trail_id}_{frame.stamp_ms}"
        self.t_world_veh = frame.opt_t_world_veh  # frame.ins_t_world_veh
        self.q_world_veh = frame.opt_q_world_veh  # frame.ins_q_world_veh
        self.T_world_veh = frame.opt_T_world_veh  # frame.ins_T_world_veh
        self.bev_label_path = frame.bev_label_path
        self.un_image_path = frame.un_image_path
        self.good_category_dict: Dict[BevLineTrackCategory, Dict[str, BEVLine]] = {}
        self.bad_category_dict: Dict[BevLineTrackCategory, Dict[str, BEVLine]] = {}
        for cate_name in BevLineTrackCategory:
            self.good_category_dict[cate_name] = {}
            self.bad_category_dict[cate_name] = {}
        self.load_frame_bev_label()

    def load_frame_bev_label(self):
        bev_label_info = util.json_file_to_dict(self.bev_label_path)
        instance_list = bev_label_info["detail"]["instance_list"]
        instance_n = len(instance_list)
        for frame_line_id in range(0, instance_n):
            one_instance = instance_list[frame_line_id]
            type_id = one_instance["attrs"]["type"][0]
            element_type = get_bev_track_category(type_id, self.data_type)
            near_points = []
            if element_type in (BevLineTrackCategory.lane_line, BevLineTrackCategory.road_boundary, BevLineTrackCategory.center_line):
                for one_point in one_instance["data"]["points"]:
                    if ((one_point[0] >= -15) and (one_point[0] <= 15)):
                        near_points.append((one_point[0], one_point[1]))
                
                if len(near_points) < 5:
                    continue
            else:
                for one_point in one_instance["data"]["points"]:
                    near_points.append((one_point[0], one_point[1]))

                if element_type == BevLineTrackCategory.split_merge:
                    pass
                elif len(near_points) < 2:
                    continue

            track_id = -1
            if "track_id" in one_instance:
                track_id = one_instance["track_id"]
            bev_line: BEVLine = BEVLine.init(f"{self.frame_id}_{frame_line_id}", frame_line_id, track_id,
                                             type_id, one_instance["attrs"]["score"],
                                             near_points, self.T_world_veh, self.data_type)
            self.good_category_dict[bev_line.category][bev_line.line_id] = bev_line

    def filter_bad_line_by_score(self, good_line_dict: dict, bad_line_dict: dict):
        element_to_del = []
        for line_id, one_line in good_line_dict.items():
            if one_line.bev_score <= 0.3:
                one_line.tag.append("low_scr")
                element_to_del.append(line_id)
        for line_id in set(element_to_del):
            bad_line_dict[line_id] = good_line_dict[line_id]
            del good_line_dict[line_id]

    def filter_bad_line_by_fit_error(self, good_line_dict: dict, bad_line_dict: dict):
        cubic_fit_error_mean_th = 0.2
        cubic_fit_error_max_th = 0.4
        cubic_fit_curvature_max_th = 1
        if self.data_type == DataType.MMT_RC.name:
            cubic_fit_error_mean_th = 0.2
            cubic_fit_error_max_th = 0.4
        elif self.data_type == DataType.BYD_BEV.name or self.data_type == DataType.BYD_LIDAR_B.name or self.data_type == DataType.BYD_LIDAR_BEV_B.name:
            cubic_fit_error_mean_th = 0.17
            cubic_fit_error_max_th = 0.35

        to_del = []
        for line_id, one_line in good_line_dict.items():
            if one_line.cubic_fit_error_mean > cubic_fit_error_mean_th or one_line.cubic_fit_error_max > cubic_fit_error_max_th:
                one_line.tag.append("fit_err")
                to_del.append(line_id)
            elif one_line.cubic_fit_curvature_max > cubic_fit_curvature_max_th:
                one_line.tag.append("curvature_err")
                to_del.append(line_id)
        for line_id in set(to_del):
            bad_line_dict[line_id] = good_line_dict[line_id]
            del good_line_dict[line_id]

    def filter_bad_line_too_short(self,  good_line_dict: dict, bad_line_dict: dict):
        to_del = []
        for line_id, one_line in good_line_dict.items():
            if one_line.linestring.length < 5:
                one_line.tag.append("short")
                to_del.append(line_id)
        for line_id in set(to_del):
            bad_line_dict[line_id] = good_line_dict[line_id]
            del good_line_dict[line_id]

    def filter_bad_line_duplicate(self, cate: BevLineTrackCategory, good_line_dict: dict, bad_line_dict: dict):
        thres = 0.2
        if cate == BevLineTrackCategory.center_line:
            thres = 1.2
        to_del = []
        for line1_id, line1 in good_line_dict.items():
            for line2_id, line2 in good_line_dict.items():
                if line1_id >= line2_id:
                    continue
                if line1.category != line2.category:
                    continue
                dis = line1.distance_to_other_bevline(line2)
                iou = line1.iou_to_other_bevline(line2)
                if dis < thres and iou > 0.6:
                    line2.tag.append(f"dup.w.{line1.frame_line_id}-{round(dis,2)}-iou{round(iou,2)}")
                    to_del.append(line2_id)
        for line_id in set(to_del):
            bad_line_dict[line_id] = good_line_dict[line_id]
            del good_line_dict[line_id]

    def filter_bad_line_cross(self, cate: BevLineTrackCategory, good_line_dict: dict, bad_line_dict: dict):
        thres = 0.2
        cubic_fit_curvature_max_th = 0.025
        if cate == BevLineTrackCategory.center_line:
            thres = 0.2
        to_del = []
        for line1_id, line1 in good_line_dict.items():
            cross_num = 0
            last_frame_id = ""
            for line2_id, line2 in good_line_dict.items():
                if line1_id == line2_id:
                    continue
                if line1.category != line2.category:
                    continue
                dis = line1.min_distance_to_other_bevline(line2)
                if dis < thres and line1.cubic_fit_curvature_max > cubic_fit_curvature_max_th:
                    cross_num += 1
                    
                    if cross_num == 1:
                        last_frame_id = str(line2.frame_line_id)
                    else:
                        last_frame_id = last_frame_id + "_" + str(line2.frame_line_id)
            
            # if cross_num >= 2:
            if cross_num >= 1:
                line1.tag.append(f"cross.w.{last_frame_id}-num:{cross_num}")
                to_del.append(line1_id)
        for line_id in set(to_del):
            bad_line_dict[line_id] = good_line_dict[line_id]
            del good_line_dict[line_id]

    def filter_bad_element(self):
        for cate, good_element_dict in self.good_category_dict.items():
            # if cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
            if cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
                self.filter_bad_line_by_score(good_element_dict, self.bad_category_dict[cate])
                self.filter_bad_line_by_fit_error(good_element_dict, self.bad_category_dict[cate])
                self.filter_bad_line_too_short(good_element_dict, self.bad_category_dict[cate])
                self.filter_bad_line_duplicate(cate, good_element_dict, self.bad_category_dict[cate])
                self.filter_bad_line_cross(cate, good_element_dict, self.bad_category_dict[cate])


class BEVMatch:
    def __init__(self) -> None:
        pass

    def xyl_match(self, line_dict_1: dict, line_dict_2: dict, T_1_2, thres):
        line_list_1 = list(line_dict_1.values())
        line_list_2 = list(line_dict_2.values())
        xyl_match_report = []
        frame1_line_n = len(line_list_1)
        frame2_line_n = len(line_list_2)
        cost_mat = np.zeros((frame1_line_n, frame2_line_n))
        for i1 in range(0, frame1_line_n):
            for i2 in range(0, frame2_line_n):
                line1: BEVLine = line_list_1[i1]
                line2: BEVLine = line_list_2[i2]
                trans_line2 = BEVLine.transform(line2, T_1_2)
                dis = line1.distance_to_other_bevline(trans_line2)
                iou = line1.iou_to_other_bevline(trans_line2)
                cost = dis
                if cost > thres:
                    cost = 99999999
                if iou < 0.6:
                    cost = 99999999
                cost_mat[i1, i2] = cost
        row_inds, col_inds = linear_sum_assignment(cost_mat)
        match_n = len(row_inds)
        for i in range(match_n):
            row_id = row_inds[i]
            col_id = col_inds[i]
            cost = cost_mat[row_id, col_id]
            if cost > thres:
                continue
            xyl_match_report.append({"line1_id": line_list_1[row_id].line_id,
                                     "line2_id": line_list_2[col_id].line_id,
                                     "frame_line1_id": line_list_1[row_id].frame_line_id,
                                     "frame_line2_id": line_list_2[col_id].frame_line_id,
                                     "cost": cost_mat[row_id, col_id]})
        return xyl_match_report

    def line_match_between_frame(self, frame1: BEVFrame, frame2: BEVFrame) -> dict:
        T_1_2 = Tab_to_Tba(frame1.T_world_veh).dot(frame2.T_world_veh)
        match_report = {"frame1_id": frame1.frame_id,
                        "frame2_id": frame2.frame_id,
                        "lane_line": self.xyl_match(frame1.good_category_dict[BevLineTrackCategory.lane_line], frame2.good_category_dict[BevLineTrackCategory.lane_line], T_1_2, 0.5),
                        # "stop_line": self.xyl_match(frame1.good_category_dict[BevLineTrackCategory.stop_line], frame2.good_category_dict[BevLineTrackCategory.stop_line], T_1_2, 1.0),
                        "center_line": self.xyl_match(frame1.good_category_dict[BevLineTrackCategory.center_line], frame2.good_category_dict[BevLineTrackCategory.center_line], T_1_2, 1.0)}
        return match_report


class LineTrack:
    def __init__(self) -> None:
        self.track_id = ""
        self.track_num = 0
        self.line_ids = []
        self.bevline_list = []
        self.category = BevLineTrackCategory.other
        self.pcd_path = ""

    def to_dict(self) -> dict:
        return {"track_id": self.track_id,
                "track_num": self.track_num,
                "line_ids": self.line_ids}

    def to_pcd(self, pcd_path: str):
        point3d = []
        bevline: BEVLine
        for bevline in self.bevline_list:
            world_bevline: BEVLine = BEVLine.transform(bevline, bevline.T_world_veh)
            for p in world_bevline.points:
                point3d.append([p[0], p[1], 0])
        self.pcd_path = pcd_path
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(point3d)
        o3d.io.write_point_cloud(self.pcd_path, cloud)


class BEVVis:
    max_x, min_x = 32, -32
    max_y, min_y = 70, -20
    pixel_size = 0.1
    width = int(round((max_y-min_y)/pixel_size))
    height = int(round((max_x-min_x)/pixel_size))
    # data_taye = "BYD_BEV"

    def __init__(self, data_type) -> None:
        self.data_type = data_type

    def add_veh_mark_to_vis(self, img):
        veh_center_x = int(round(self.max_y/self.pixel_size))
        veh_center_y = int(round(self.max_x/self.pixel_size))
        cv2.rectangle(img, pt1=(veh_center_x+10, veh_center_y+20),  pt2=(veh_center_x-10, veh_center_y-20), color=(100, 100, 100),  thickness=1)

    def add_front_image_to_vis(self, img, image_path):
        fc120 = cv2.imread(image_path)
        neww = 640
        scale = neww/fc120.shape[1]
        newh = int(fc120.shape[0]*scale)
        img[self.height-newh:, :neww] = cv2.resize(fc120, (neww, newh))

    def add_bev_line_to_vis(self, img, bevline: BEVLine, rgb):
        points = np.array(bevline.to_image_points(self.max_x, self.min_x, self.max_y, self.min_y, self.pixel_size))
        cv2.putText(img, f"{bevline.frame_line_id}", (points[0][0], points[0][1]+(bevline.frame_line_id % 3)*10), cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)
        cv2.putText(img, f"{bevline.frame_line_id}", (points[-1][0], points[-1][1]-(bevline.frame_line_id % 2)*10), cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)
        cv2.polylines(img, [points], False, rgb, thickness=1)

    def add_string_to_vis(self, img, string, line_num, rgb):
        cv2.putText(img, string, [0, 25+line_num*15], cv2.FONT_HERSHEY_PLAIN, 1, rgb, 1)

    def vis_frame_bev_png(self, frame: BEVFrame, png_path: str):
        TYPE_NUM = get_type_len(self.data_type)

        img = np.zeros((self.height, self.width, 3), np.uint8)
        cv2.putText(img, f"{frame.frame_id}", (0, 10), cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 255], 1)
        cv2.putText(img, f"{round(frame.t_world_veh[0],1)},{round(frame.t_world_veh[1],1)}", (self.width-150, self.height-5), cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 255], 1)
        self.add_veh_mark_to_vis(img)
        # self.add_front_image_to_vis(img, frame.un_image_path)
        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            for bad_line in frame.bad_category_dict[cate].values():
                rgb = [100, 100, 100]
                self.add_bev_line_to_vis(img, bad_line, rgb)
                self.add_string_to_vis(img, bad_line.info_str(), bad_line.frame_line_id, rgb)
            for good_line in frame.good_category_dict[cate].values():
                rgb = [int(255 * x) for x in matplotlib.cm.jet(good_line.type_id / TYPE_NUM)[:3]]
                self.add_bev_line_to_vis(img, good_line, rgb)
                self.add_string_to_vis(img, good_line.info_str(), good_line.frame_line_id, rgb)
        write_image(png_path, img)

    def vis_line_match(self, frame1: BEVFrame, frame2: BEVFrame, match_report, png_path: str):
        img = np.zeros((self.height, self.width, 3), np.uint8)
        cv2.putText(img, f"{frame1.frame_id}_{frame2.frame_id}", (0, 10), cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 255], 1)
        self.add_veh_mark_to_vis(img)
        # self.add_front_image_to_vis(img, frame1.un_image_path)
        line_num = 0
        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            for good_line in frame1.good_category_dict[cate].values():
                bgr = [0, 0, 255]
                self.add_bev_line_to_vis(img, good_line, bgr)
                self.add_string_to_vis(img, good_line.info_str(), line_num, bgr)
                line_num += 1
            T_new_old = Tab_to_Tba(frame1.T_world_veh).dot(frame2.T_world_veh)
            other_line: BEVLine
            for other_line in frame2.good_category_dict[cate].values():
                new_other_line = BEVLine.transform(other_line, T_new_old)
                bgr = [0, 255, 0]
                self.add_bev_line_to_vis(img, new_other_line, bgr)
                self.add_string_to_vis(img, new_other_line.info_str(), line_num, bgr)
                line_num += 1

        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            for line_match in match_report[cate.name]:
                match_str = f'''{cate.name}_{line_match["frame_line1_id"]}_{line_match["frame_line2_id"]}_{round(line_match["cost"],2)}'''
                bgr = [255, 255, 255]
                self.add_string_to_vis(img, match_str, line_num, bgr)
                line_num += 1
        write_image(png_path, img)


class BEVTrail:
    def __init__(self, data_trail_folder: str, bev_trail_folder: str, debug_vis: int, info_json_path: str) -> None:
        self.data_trail_folder = data_trail_folder
        self.bev_trail_folder = bev_trail_folder

        data_set_json_path = os.path.join(self.data_trail_folder, "data_set.json")
        self.trail = TrailData.init_from_json(data_set_json_path)
        self.trail_id = self.trail.trail_id
        self.data_type = self.trail.data_type

        self.bevframe_dict: Dict[str, BEVFrame] = {}
        self.load_trail_bev_label()

        self.bewteen_frame_match_report_list = []
        self.line_track_dict = {}

        # self.all_obj_info = {}
        self.all_obj_info: Dict[BevLineTrackCategory, Dict[int, list]] = {}

        self.debug_vis = debug_vis

        # 获取路口中心点   
        try:
            with open(info_json_path,'r',encoding='utf8')as fp:
                json_data = json.load(fp)
            fp.close()
            self.site_center = json_data['middle']['site_center']
            # log.debug("site_center:", self.site_center)
        except:
            self.site_center = None
            log.error("site_center is None")

    def load_trail_bev_label(self):
        for frame in self.trail.frame_dict.values():
            if(frame.bev_label_path == ""):
                log.info("frame 无 bev_label_path, ", frame.stamp_ms)
                continue
            if(len(frame.opt_t_world_veh) == 0):
                log.info("frame 无 opt_t_world_veh, ", frame.stamp_ms)
                continue
            bevframe = BEVFrame(frame, self.trail_id, self.data_type)
            self.bevframe_dict[bevframe.frame_id] = bevframe
        log.info(f"{self.trail_id} 载入Frame 数 {len(self.bevframe_dict)}")

    def vis_trail_bev_png(self, vis_name: str):
        log.info("输出", vis_name, "可视化")
        bevvis: BEVVis = BEVVis(self.data_type)
        bevframe_list = list(self.bevframe_dict.values())
        for i in tqdm(range(len(bevframe_list))):
            bevframe: BEVFrame = bevframe_list[i]
            bevvis.vis_frame_bev_png(bevframe, os.path.join(self.bev_trail_folder, f"{vis_name}/{bevframe.stamp_ms}_{vis_name}.png"))
        log.info("完成", vis_name, "可视化")

    def vis_line_match(self):
        log.info("输出", "line_match", "可视化")
        bevvis: BEVVis = BEVVis(self.data_type)
        for i in tqdm(range(len(self.bewteen_frame_match_report_list))):
            match_report = self.bewteen_frame_match_report_list[i]
            frame1 = self.bevframe_dict[match_report["frame1_id"]]
            frame2 = self.bevframe_dict[match_report["frame2_id"]]
            bevvis.vis_line_match(frame1, frame2, match_report, os.path.join(self.bev_trail_folder, f"line_match/{frame1.stamp_ms}_{frame2.stamp_ms}.png"))
        log.info("完成", "line_match", "可视化")

    def vis_line_track_in_one(self):
        txt_lines = ["//X Y Z R G B track_id type_id"]
        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            one_line_track: LineTrack
            for one_line_track in self.line_track_dict[cate]:
                if one_line_track.track_num < 3:  # filter short track
                    continue
                rgb = tuple(np.random.randint(50, 255, 3).tolist())
                line_track_id = one_line_track.track_id
                track_id_num = int(line_track_id.split("_")[-1])
                for line_id in one_line_track.line_ids:
                    sp = line_id.split("_")
                    trail_id = sp[0]
                    frame_stamp = sp[1]
                    frame_id = f"{trail_id}_{frame_stamp}"
                    frame: BEVFrame = self.bevframe_dict[frame_id]
                    bevline: BEVLine = frame.good_category_dict[cate][line_id]
                    trans_line: BEVLine = BEVLine.transform(bevline, frame.T_world_veh)
                    for p in trans_line.points:
                        txt_lines.append(f"{p[0]} {p[1]} 0 {rgb[0]} {rgb[1]} {rgb[2]} {track_id_num} {cate.value}")
        util.write_lines_to_file_override(os.path.join(self.bev_trail_folder, f"vis_line_track-{self.trail_id}.txt"), txt_lines)

    def update_data_set_json(self):
        for frame in self.trail.frame_dict.values():
            frame.good_bev_line_id = []

        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):        
            one_line_track: LineTrack
            for one_line_track in self.line_track_dict[cate]:
                if one_line_track.track_num < 3:  # filter short track
                    continue
                for line_id in one_line_track.line_ids:
                    sp = line_id.split("_")
                    frame_stamp = sp[1]
                    frame_line_id = sp[2]
                    frame: FrameData = self.trail.frame_dict[int(frame_stamp)]
                    frame.good_bev_line_id.append(int(frame_line_id))
        self.trail.to_json(os.path.join(self.data_trail_folder, "data_set.json"))  

    def update_bev_label_info(self):
        for bevframe in self.bevframe_dict.values():
            bev_label_info = util.json_file_to_dict(bevframe.bev_label_path)
            instance_list = bev_label_info["detail"]["instance_list"]
            # debug
            # for instance in instance_list:
            #     instance["attrs"]["score"] = 0
            for cate, good_element_dict in bevframe.good_category_dict.items():
                if cate not in (BevLineTrackCategory.stop_line, BevLineTrackCategory.crosswalk, BevLineTrackCategory.arrow):
                    continue
                for line_id, bevline in good_element_dict.items():
                    frame_line_id = int(line_id.split("_")[-1])
                    one_instance = instance_list[frame_line_id]
                    type_id = one_instance["attrs"]["type"][0]
                    element_type = get_bev_track_category(type_id, self.data_type)
                    if element_type != cate:
                        continue
                    points = bevframe.good_category_dict[cate][line_id].points
                    if isinstance(points, list):
                        one_instance["data"]["points"] = [arr.tolist() if isinstance(arr, np.ndarray) else arr for arr in points]
                    else:
                        one_instance["data"]["points"] = points.tolist() if isinstance(points, np.ndarray) else points
                    # debug
                    # trail_id = int(line_id.split("_")[0])
                    # one_instance["attrs"]["score"] = trail_id
                    
            with open(bevframe.bev_label_path, 'w') as f:
                json.dump(bev_label_info, f, indent=4)


    def filter_trail_bad_element(self):
        log.info("过滤错误元素 in trail", self.trail_id)
        bevframe_list = list(self.bevframe_dict.values())
        for i in tqdm(range(len(bevframe_list))):
            bevframe_list[i].filter_bad_element()
        log.info("完成过滤错误元素")

    def calculate_line_direction(self, points):
        if not isinstance(points, np.ndarray):
            points = np.array(points)
        centered_points = points - np.mean(points, axis=0)
        cov_matrix = np.cov(centered_points.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        main_direction = eigenvectors[:, np.argmax(eigenvalues)]
        return main_direction / np.linalg.norm(main_direction)
    
    def calculate_angle_between_vectors(self, vec1, vec2):
        norm_vec1 = np.linalg.norm(vec1)
        norm_vec2 = np.linalg.norm(vec2)  
        if norm_vec1 == 0 or norm_vec2 == 0:
            return np.pi/2

        cos_theta = np.dot(vec1, vec2) / (norm_vec1 * norm_vec2)
        return np.arccos(np.clip(cos_theta, -1.0, 1.0))

    def filter_lane_group_for_object(self, object_center, distance_threshold):
        lane_group = []
        if self.site_center is None:
            return lane_group

        site_center = np.array(self.site_center)
        object_center = np.array(object_center)
        if (site_center.shape[0] >= 2):
            site_center = site_center[0:2]
        else:
            log.error("site_center 维度不正确")
            return lane_group

        scope = np.linalg.norm(object_center - site_center)
        scope = scope - 15 if scope > 60 else scope

        tmp_lane_list = []
        for bevframe in self.bevframe_dict.values():
            for cate, good_element_dict in bevframe.good_category_dict.items():
                if cate != BevLineTrackCategory.lane_line:
                    continue               

                for line_id, bevline in good_element_dict.items():   
                    trans_line = BEVLine.transform(bevline, bevframe.T_world_veh)
                    line_points = np.array(trans_line.points)
                    line_center = np.mean(line_points, axis=0)

                    dist_line_to_object = np.linalg.norm(line_center - object_center)
                    if dist_line_to_object > 50:
                        continue
                    # log.info(f"[0]site_center: {site_center}, object_center: {object_center}, line_center: {line_center}, scope: {scope}, dist_line_to_object: {dist_line_to_object}")

                    dist_line_to_site = np.linalg.norm(line_center - site_center) 
                    if dist_line_to_site < scope:
                        continue
                    # log.info(f"[1]site_center: {site_center}, object_center: {object_center}, line_center: {line_center}, scope: {scope}, dist_line_to_object: {dist_line_to_object}, dist_line_to_site: {dist_line_to_site}")

                    site_line_vec = (line_center - site_center) / np.linalg.norm(line_center - site_center)
                    site_obj_vec = (object_center - site_center) / np.linalg.norm(object_center - site_center)
                    cos_site_angle = np.dot(site_line_vec, site_obj_vec)
                    site_angle = np.arccos(np.clip(cos_site_angle, -1.0, 1.0))
                    if np.pi/4 < site_angle < 3*np.pi/4:
                        continue
                    # log.info(f"[2]site_center: {site_center}, object_center: {object_center}, line_center: {line_center}, dist_line_to_object: {dist_line_to_object}, dist_line_to_site: {dist_line_to_site}, site_angle: {site_angle}")
                    
                    line_dir = self.calculate_line_direction(line_points)
                    line_dir /= np.linalg.norm(line_dir)
                    obj_line_vec = (object_center - line_center) / np.linalg.norm(object_center - line_center)
                    cos_obj_angle = np.dot(line_dir, obj_line_vec)
                    object_angle = np.arccos(np.clip(cos_obj_angle, -1.0, 1.0))
                    if np.pi/4 < object_angle < 3*np.pi/4:
                        continue
                    # log.info(f"[3]site_center: {site_center}, object_center: {object_center}, line_center: {line_center}, dist_line_to_object: {dist_line_to_object}, dist_line_to_site: {dist_line_to_site}, site_angle: {site_angle}, object_angle: {object_angle}")
                    
                    dists = np.linalg.norm(line_points - object_center, axis=1)
                    mask = dists <= distance_threshold
                    points_in_threshold = line_points[mask]
                    if len(points_in_threshold) >= 2:
                        trans_line.points = points_in_threshold.tolist()
                        min_dist = np.min(dists[mask])
                        tmp_lane_list.append((line_center, min_dist, trans_line, site_angle, object_angle))
        
        sorted_lanes = heapq.nsmallest(10, tmp_lane_list, key=lambda x: x[1])
        lane_group.extend([x[2] for x in sorted_lanes])

        # 日志打印
        # for i in range(min(len(tmp_lane_list), 20)):
        #     lane_info = tmp_lane_list[i]
        #     log.info(f"site_center: {site_center}, object_center: {object_center}, line_center: {lane_info[0]}, min_dist_to_object: {lane_info[1]}, site_angle: {lane_info[3]}, object_angle: {lane_info[4]}")

        return lane_group

    def filter_and_average_directions(self, directions, threshold):
        avg_direction = np.mean(directions, axis=0)
        avg_direction = avg_direction / np.linalg.norm(avg_direction)

        filtered_directions = []
        for direction in directions:
            if np.dot(direction, avg_direction) > threshold:
                filtered_directions.append(direction)

        if len(filtered_directions) < 2:
            return avg_direction

        final_avg_direction = np.mean(filtered_directions, axis=0)
        final_avg_direction = final_avg_direction / np.linalg.norm(final_avg_direction)
        return final_avg_direction

    def get_lane_group_main_direction(self, lane_lines, threshold=0.866):
        """
        计算车道组的主方向。
        1. 先单独计算每条车道线的方向。
        2. 对这些方向进行筛选和平均化处理。
        (cos(80°)= 0.1736, cos(70°)=0.292, cos(60°)=0.5, cos(45°)=0.707, cos(30°)=0.866, cos20°=0.939, cos(10°)=0.984)
        """
        lane_directions = [self.calculate_line_direction(lane.points) for lane in lane_lines]
        return self.filter_and_average_directions(lane_directions, threshold)

    def is_orthogonal(self, angle, threshold):
        return abs(angle - 90) <= threshold

    def is_parallel(self, angle, threshold):
        return abs(angle) <= threshold or abs(angle - 180) <= threshold

    def correct_obj_direction(self, obj_points, rotation_angle):
        """
        根据旋转的角度对object的坐标进行矫正。
        如果 rotation_angle 为正数，那么 object 会按照逆时针方向旋转。
        如果 rotation_angle 为负数，那么 object 会按照顺时针方向旋转。
        """
        theta = np.radians(rotation_angle)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])

        # # 确定旋转中心，这里选择第一个点作为旋转中心
        # center = np.array(obj[0])
        # 确定旋转中心，这里选择第一个点作为旋转中心
        center = np.mean(obj_points, axis=0)

        processed_obj = []
        for point in obj_points:
            new_point = np.dot(rotation_matrix, np.array(point) - center) + center
            processed_obj.append(new_point.tolist())
        return processed_obj
    
    def calculate_max_edge_length(self, points):
        if not isinstance(points, np.ndarray):
            points = np.array(points)

        edge_lengths = [np.linalg.norm(points[i] - points[j]) for i, j in combinations(range(points.shape[0]), 2)]

        return max(edge_lengths) if edge_lengths else 0 

    def extract_and_cluster_objects(self, element_type, eps, min_samples):
        all_objects = []
        for bevframe in self.bevframe_dict.values():
            for cate, good_element_dict in bevframe.good_category_dict.items():
                if cate == element_type:
                    for line_id, bevline in good_element_dict.items():
                        if not bevline.points:
                            continue
                        trans_bevline: BEVLine = BEVLine.transform(bevline, bevframe.T_world_veh)
                        center = np.mean(np.array(trans_bevline.points), axis=0)
                        if len(trans_bevline.points) > 2:
                            length = self.calculate_max_edge_length(trans_bevline.points)
                        else:
                            length = np.linalg.norm(np.array(trans_bevline.points[-1]) - np.array(trans_bevline.points[0]))
                        T_veh_world = np.linalg.inv(bevframe.T_world_veh)                
                        all_objects.append((line_id, center, length, trans_bevline, T_veh_world))
                        # log.info(f"before line_id: {line_id}, raw stopline: {stopline.points}, T_world_veh: {bevframe.T_world_veh}, T_veh_world: {T_veh_world}, center: {center}")

        if not all_objects:
            return {}
        centerpoints = np.array([center for _, center, _, _, _ in all_objects])
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(centerpoints)

        clustered_objects = {}
        for label, (line_id, center, length, trans_bevline, T_veh_world) in zip(labels, all_objects):
            if label not in clustered_objects:
                clustered_objects[label] = []
            clustered_objects[label].append((line_id, center, length, trans_bevline, T_veh_world))

        # 日志打印
        # for label, cluster in clustered_objects.items():    
        #     if label == -1:
        #         continue
        #     for line_id, center, length, trans_bevline, T_veh_world in cluster:
        #         log.info(f"after cluster label: {label}, line_id: {line_id}, center: {center}, length: {length}, trans_bevline: {trans_bevline.points}")
  

        return clustered_objects
    
    def point_transform(self, points, T_new_old):
        if not isinstance(points, np.ndarray):
            points = np.array(points)
        
        homogeneous_points = np.hstack((points, np.zeros((points.shape[0], 1)), np.ones((points.shape[0], 1))))

        transformed_homogeneous_points = (T_new_old @ homogeneous_points.T).T
        w = transformed_homogeneous_points[:, 3]
        if np.any(np.isclose(w, 0)):
            raise ValueError("Transformation resulted in zero w component. Cannot normalize.")

        transformed_points = transformed_homogeneous_points[:, :3] / w[:, np.newaxis]
        
        return transformed_points[:, :2].tolist()

    def clear_object_bev_info(self, element_type, line_id):
        for bevframe in self.bevframe_dict.values():  
            if line_id in bevframe.good_category_dict[element_type]:
                bevframe.good_category_dict[element_type][line_id].points = []
                break     

    def process_stopline_by_lane_group(self, stopline, lane_group_main_direction):
        processed_stopline = []

        orthogonal_threshold = 30
        parallel_threshold = 30

        direction = np.array(stopline.points[-1]) - np.array(stopline.points[0])
        direction = direction / np.linalg.norm(direction)

        angle = np.degrees(self.calculate_angle_between_vectors(direction, lane_group_main_direction))
        cross_product = np.cross(direction, lane_group_main_direction)
        
        if self.is_orthogonal(angle, orthogonal_threshold):
            rotation_angle = 90 - angle if cross_product < 0 else angle - 90
        elif self.is_parallel(angle, parallel_threshold):
            if abs(angle) < parallel_threshold:
                rotation_angle = -angle if cross_product < 0 else angle
            else:
                rotation_angle = 180 - angle if cross_product < 0 else angle - 180
        else:
            return processed_stopline

        processed_stopline = self.correct_obj_direction(stopline.points, rotation_angle)
        # log.info(f"angle: {angle}°, rotate: {rotation_angle}°, stopline before: {stopline.points}, after: {processed_stopline}")

        return processed_stopline

    def cal_line_direction(self, line_points):
        direction = np.array(line_points[-1]) - np.array(line_points[0])
        direction = direction / np.linalg.norm(direction)
        return direction
    
    def preprocess_trail_stopline(self):
        log.info("预处理stopline")
        eps = 3
        min_samples = 5
        clustered_stoplines = self.extract_and_cluster_objects(BevLineTrackCategory.stop_line, eps, min_samples)

        for label, cluster in clustered_stoplines.items():
            if label == -1:
                for line_id, _, _, _, _ in cluster:
                    self.clear_object_bev_info(BevLineTrackCategory.stop_line, line_id)
                continue

            # debug
            trail_id = int(cluster[0][0].split("_")[0])
            # if not (trail_id == 6 and label == 3):
            #     for line_id, _, _, _, _ in cluster:   
            #         self.clear_object_bev_info(BevLineTrackCategory.stop_line, line_id)
            #     continue

            # 计算聚类的中心和平均长度
            cluster_center = np.mean(np.array([center for _, center, _, _, _ in cluster]), axis=0)
            cluster_avg_length = np.mean(np.array([length for _, _, length, _, _ in cluster]), axis=0)
            # log.info(f"trail_id: {trail_id}, label: {label}, cluster size: {len(cluster)}, cluster_center: {cluster_center}, cluster_avg_length: {cluster_avg_length}")

             # 找附近最相关车道组
            lane_group = self.filter_lane_group_for_object(cluster_center, cluster_avg_length/2)
            if not lane_group:
                lane_group = self.filter_lane_group_for_object(cluster_center, cluster_avg_length)
            if not lane_group:
                log.info(f"trail_id: {trail_id}, label: {label}, cluster size: {len(cluster)}, cluster_center: {cluster_center}, cluster_avg_length: {cluster_avg_length}, stopline cluster 找不到车道组")
                for line_id, _, _, trans_bevline, T_veh_world in cluster:   
                    self.clear_object_bev_info(BevLineTrackCategory.stop_line, line_id)                
                continue

            # 计算车道组主方向
            lane_group_main_direction = self.get_lane_group_main_direction(lane_group)
            # log.info(f"trail_id: {trail_id}, label: {label}, lane_group: {len(lane_group)}, lane_group_main_direction: {lane_group_main_direction}")

            # 计算聚类中所有stopline的平均倾斜角
            sum_angle = 0
            for line_id, _, _, trans_bevline, T_veh_world in cluster:
                stopline_direction = self.cal_line_direction(trans_bevline.points)
                angle = np.degrees(self.calculate_angle_between_vectors(stopline_direction, lane_group_main_direction))
                if self.is_orthogonal(angle, 45):
                    angle = abs(angle - 90)
                elif self.is_parallel(angle, 45):
                    if angle < 45:
                        angle = abs(angle)
                    else:
                        angle = 180 - angle
                sum_angle += angle
            avg_angle = sum_angle / len(cluster)
            if avg_angle > 25 and avg_angle < 35:
                log.info(f"trail_id: {trail_id}, label: {label}, cluster size: {len(cluster)}, cluster_center: {cluster_center}, avg_angle: {avg_angle}, stopline平均倾斜角大于25°，不做校正处理")
                continue

            for line_id, _, _, trans_bevline, T_veh_world in cluster:
                processed_stopline = self.process_stopline_by_lane_group(trans_bevline, lane_group_main_direction)                
                if not processed_stopline :
                    for line_id, _, _, _, _ in cluster:
                        self.clear_object_bev_info(BevLineTrackCategory.stop_line, line_id)
                    continue

                final_stopline = self.point_transform(processed_stopline, T_veh_world)

                for bevframe in self.bevframe_dict.values():
                    if line_id in bevframe.good_category_dict[BevLineTrackCategory.stop_line]:
                        bevframe.good_category_dict[BevLineTrackCategory.stop_line][line_id].points = final_stopline
                        break
        log.info("完成预处理stopline")    

    def cal_polygon_long_side_direction(self, polygon_points):
        """
        计算四边形的长边方向
        """
        p1, p2, p3, p4 = polygon_points
        length1 = np.linalg.norm(np.array(p2) - np.array(p1))
        length2 = np.linalg.norm(np.array(p4) - np.array(p1))
        if length1 >= length2:
            long_direction = np.array(p2) - np.array(p1)
        else:
            long_direction = np.array(p4) - np.array(p1)
        long_direction = long_direction / np.linalg.norm(long_direction)

        return long_direction

    def process_polygon_by_lane_group(self, polygon, lane_group_main_direction):
        """
        根据车道组的主方向对人行横道进行过滤。
        如果人行横道的某一方向与车道组的主方向近似垂直或平行，则保留该人行横道。
        并使用人行横道与车道组的夹角修正人行横道
        """
        processed_polygon = []

        orthogonal_threshold = 30
        parallel_threshold = 30

        # 仅处理四边形
        if len(polygon.points) != 4:
            return processed_polygon

        long_direction = self.cal_polygon_long_side_direction(polygon.points)
        angle = np.degrees(self.calculate_angle_between_vectors(long_direction, lane_group_main_direction))  
        cross_product = np.cross(long_direction, lane_group_main_direction)
        # log.info(f"long_direction: {long_direction}, lane_group_main_direction: {lane_group_main_direction}, angle: {angle}, cross_product: {cross_product}")
     
        # 确定旋转方向
        if self.is_orthogonal(angle, orthogonal_threshold):
            rotation_angle = 90 - angle if cross_product < 0 else angle - 90
        elif self.is_parallel(angle, parallel_threshold):
            if abs(angle) < parallel_threshold:
                rotation_angle = -angle if cross_product < 0 else angle
            else:
                rotation_angle = 180 - angle if cross_product < 0 else angle - 180
        else:
            return processed_polygon

        processed_polygon = self.correct_obj_direction(polygon.points, rotation_angle)
        # log.info(f"angle: {angle}°, rotate: {rotation_angle}°, polygon before: {polygon.points}, after: {processed_polygon}")

        return processed_polygon

    def preprocess_trail_crosswalk(self):
        log.info("预处理crosswalk")

        eps = 3
        min_samples = 5
        clustered_crosswalkes = self.extract_and_cluster_objects(BevLineTrackCategory.crosswalk, eps, min_samples)

        for label, cluster in clustered_crosswalkes.items():
            if label == -1:
                for line_id, _, _, trans_bevline, T_veh_world in cluster:   
                    self.clear_object_bev_info(BevLineTrackCategory.crosswalk, line_id)
                continue

            # debug
            trail_id = int(cluster[0][0].split("_")[0])
            # if not (trail_id == 6 and label == 1):
            #     for line_id, _, _, _, _ in cluster:  
            #         self.clear_object_bev_info(BevLineTrackCategory.crosswalk, line_id)
            #     continue

            cluster_center = np.mean(np.array([center for _, center, _, _, _ in cluster]), axis=0)
            cluster_avg_length = np.mean(np.array([length for _, _, length, _, _ in cluster]), axis=0)

            # 找最相关的车道组并计算车道组主方向
            lane_group = self.filter_lane_group_for_object(cluster_center, cluster_avg_length * 0.75)
            if not lane_group:
                lane_group = self.filter_lane_group_for_object(cluster_center, cluster_avg_length)
            if not lane_group:
                log.info(f"trail_id: {trail_id}, label: {label}, cluster size: {len(cluster)}, cluster_center: {cluster_center}, crosswalk cluster 找不到车道组")
                for line_id, _, _, trans_bevline, T_veh_world in cluster:   
                    self.clear_object_bev_info(BevLineTrackCategory.crosswalk, line_id)
                continue

            lane_group_main_direction = self.get_lane_group_main_direction(lane_group)
            # log.info(f"trail_id: {trail_id}, label: {label}, lane_group: {len(lane_group)}, lane_group_main_direction: {lane_group_main_direction}")
            
            # 计算聚类中所有crosswalk的平均倾斜角
            sum_angle = 0
            for line_id, _, _, trans_bevline, T_veh_world in cluster:
                long_direction = self.cal_polygon_long_side_direction(trans_bevline.points)         
                angle = np.degrees(self.calculate_angle_between_vectors(long_direction, lane_group_main_direction))               
                if self.is_orthogonal(angle, 45):
                    angle = abs(angle - 90)
                elif self.is_parallel(angle, 45):
                    if angle < 45:
                        angle = abs(angle)
                    else:
                        angle = 180 - angle
                sum_angle += angle
            avg_angle = sum_angle / len(cluster)
            if avg_angle > 25 and avg_angle < 35:
                log.info(f"trail_id: {trail_id}, label: {label}, cluster size: {len(cluster)}, cluster_center: {cluster_center}, avg_angle: {avg_angle}, crosswalk平均倾斜角大于25°，不做校正处理")
                continue

            for line_id, _, _, trans_bevline, T_veh_world in cluster:
                processed_crosswalk = self.process_polygon_by_lane_group(trans_bevline, lane_group_main_direction)         
                if not processed_crosswalk:
                    self.clear_object_bev_info(BevLineTrackCategory.crosswalk, line_id)
                    continue

                final_crosswalk = self.point_transform(processed_crosswalk, T_veh_world)
                for bevframe in self.bevframe_dict.values():
                    if line_id in bevframe.good_category_dict[BevLineTrackCategory.crosswalk]:
                        bevframe.good_category_dict[BevLineTrackCategory.crosswalk][line_id].points = final_crosswalk
                        break

        log.info("完成预处理crosswalk")

    def preprocess_trail_arrow(self):
        log.info("预处理arrow")

        eps = 0.5
        min_samples = 5
        clustered_arrows = self.extract_and_cluster_objects(BevLineTrackCategory.arrow, eps, min_samples)

        for label, cluster in clustered_arrows.items():
            if label == -1:
                for line_id, _, _, trans_bevline, T_veh_world in cluster:   
                    self.clear_object_bev_info(BevLineTrackCategory.arrow, line_id)
                continue

            # debug
            trail_id = int(cluster[0][0].split("_")[0])
            # if not (trail_id == 6):
            #     for line_id, _, _, trans_bevline, T_veh_world in cluster:   
            #         self.clear_object_bev_info(BevLineTrackCategory.arrow, line_id)
            #     continue

            cluster_center = np.mean(np.array([center for _, center, _, _, _ in cluster]), axis=0)
            # log.info(f"trail_id: {trail_id}, label: {label}, cluster size: {len(cluster)}, cluster_center: {cluster_center}")

            # 找最相关的车道组并计算车道组主方向
            lane_group = self.filter_lane_group_for_object(cluster_center, 5)
            if not lane_group:
                log.info(f"trail_id: {trail_id}, label: {label}, arrow cluster 找不到车道组")
                for line_id, _, _, trans_bevline, T_veh_world in cluster: 
                    self.clear_object_bev_info(BevLineTrackCategory.arrow, line_id)
                continue

            lane_group_main_direction = self.get_lane_group_main_direction(lane_group)
            # log.info(f"trail_id: {trail_id}, label: {label}, lane_group: {len(lane_group)}, lane_group_main_direction: {lane_group_main_direction}")

            for line_id, _, _, trans_bevline, T_veh_world in cluster:
                processed_arrow = self.process_polygon_by_lane_group(trans_bevline, lane_group_main_direction)         
                if not processed_arrow:
                    self.clear_object_bev_info(BevLineTrackCategory.arrow, line_id)
                    continue

                final_arrow = self.point_transform(processed_arrow, T_veh_world)
                for bevframe in self.bevframe_dict.values():
                    if line_id in bevframe.good_category_dict[BevLineTrackCategory.arrow]:
                        bevframe.good_category_dict[BevLineTrackCategory.arrow][line_id].points = final_arrow
                        break

        log.info("完成预处理arrow")

    def match_trail_frames(self):
        log.info("进行帧间元素match")
        bevmatch: BEVMatch = BEVMatch()
        frame_list = list(self.bevframe_dict.values())
        frame_n = len(self.bevframe_dict)
        if frame_n <= 5:
            log.warning("帧数不足不执行帧间track: ", self.trail_id, frame_n)
            return
        for jump in (1, 2, 3):
            for i in tqdm(range(jump, frame_n)):
                frame1: BEVFrame = frame_list[i]
                frame2: BEVFrame = frame_list[i-jump]
                line_match_report = bevmatch.line_match_between_frame(frame1, frame2)
                self.bewteen_frame_match_report_list.append(line_match_report)
        log.info("完成帧间元素match")

    def track_trail_lines(self):
        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            self.line_track_dict[cate] = []

        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            graph: networkx.Graph = networkx.Graph()
            for match_report in self.bewteen_frame_match_report_list:
                for one_match in match_report[cate.name]:
                    graph.add_node(one_match["line1_id"])
                    graph.add_node(one_match["line2_id"])
                    graph.add_edge(one_match["line1_id"], one_match["line2_id"])
            connect_graphs = [graph.subgraph(c).copy() for c in networkx.connected_components(graph)]
            for connect_graph in connect_graphs:
                line_track: LineTrack = LineTrack()
                line_track.track_id = f"{self.trail_id}_{len(self.line_track_dict[cate])}"
                line_track.category = cate
                for line_id in connect_graph.nodes():
                    line_track.line_ids.append(line_id)
                    sp = line_id.split("_")
                    frame: BEVFrame = self.bevframe_dict[f"{sp[0]}_{sp[1]}"]
                    line_track.bevline_list.append(frame.good_category_dict[cate][line_id])
                line_track.track_num = len(line_track.line_ids)
                self.line_track_dict[cate].append(line_track)
                line_track.to_pcd(os.path.join(self.bev_trail_folder, f"line_track-{line_track.track_num}-{line_track.category.name}-{line_track.track_id}.pcd"))

        line_track_reprot = {"trail_id": self.trail_id, "lane_line": [], "stop_line": [], "center_line": []}
        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            for line_track in self.line_track_dict[cate]:
                line_track_reprot[f"{cate.name}"].append(line_track.to_dict())
        util.dict_to_json_file(os.path.join(self.bev_trail_folder, f"line_track_report-{self.trail_id}.json"), line_track_reprot)

    def run(self):
        if os.getenv("DEBUG_LIDAR_MAPPING") == "YES" or self.debug_vis:
            self.vis_trail_bev_png("raw_line")

        self.filter_trail_bad_element()

        if os.getenv("DEBUG_LIDAR_MAPPING") == "YES" or self.debug_vis:
            self.vis_trail_bev_png("filter_line")

        self.match_trail_frames()

        if os.getenv("DEBUG_LIDAR_MAPPING") == "YES" or self.debug_vis:
            self.vis_line_match()

        self.track_trail_lines()
        
        self.vis_line_track_in_one()

        self.update_data_set_json()

        log.info(f"{self.trail_id} 完成处理")


    def track_trail_lines_qzc(self):
        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            self.line_track_dict[cate] = []
            self.all_obj_info[cate] = {}
            # self.all_lines_map: Dict[BevLineTrackCategory, Dict[str, BEVLine]] = {}
        
        # self.frame_id = f"{self.trail_id}_{frame_stamp}"
        # frame_line_id = instance_id
        # line_id = f"{self.frame_id}_{frame_line_id}"
        # instance.trail_id: str = line_id.split("_")[0]

        log.info("add track info from trail", self.trail_id)
        for frame_id, frame in self.bevframe_dict.items(): # 当前轨迹所有的帧
            # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
            for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
                for line_id, bev_line in frame.good_category_dict[cate].items(): # 当前帧所有的线类型
                    # self.good_category_dict[bev_line.category][bev_line.line_id] = bev_line
                    type_id=bev_line.type_id
                    # category=BEVLine.type_id_to_category_qzc(type_id)
                    category=get_bev_track_category(type_id, self.data_type)

                    track_id = bev_line.track_id
                    if track_id in self.all_obj_info[category]:
                        self.all_obj_info[category][track_id].append(line_id)
                    else:
                        self.all_obj_info[category][track_id] = [line_id]
                    # print()

        # for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.stop_line, BevLineTrackCategory.center_line):
        for cate in (BevLineTrackCategory.lane_line, BevLineTrackCategory.center_line):
            for track_id, line_ids in self.all_obj_info[cate].items(): # 当前帧所有的线类型
                line_track: LineTrack = LineTrack()
                line_track.track_id = f"{self.trail_id}_{len(self.line_track_dict[cate])}"
                line_track.category = cate
                line_track.line_ids = line_ids
                line_track.track_num = len(line_track.line_ids)
                self.line_track_dict[cate].append(line_track)
    
        log.info("add track info done.")


    def run_qzc(self):
        if os.getenv("DEBUG_LIDAR_MAPPING") == "YES" or self.debug_vis:
            self.vis_trail_bev_png("raw_line")

        self.filter_trail_bad_element()

        if os.getenv("DEBUG_LIDAR_MAPPING") == "YES" or self.debug_vis:
            self.vis_trail_bev_png("filter_line")

        # self.match_trail_frames()

        # if os.getenv("DEBUG_LIDAR_MAPPING") == "YES" or self.debug_vis:
        #     self.vis_line_match()

        self.track_trail_lines_qzc()
        self.vis_line_track_in_one()

        self.update_data_set_json()

        self.preprocess_trail_stopline()

        self.preprocess_trail_crosswalk()

        self.preprocess_trail_arrow()

        self.update_bev_label_info()

        log.info(f"{self.trail_id} 完成处理")


dafault_data_trail_folder="/mnt/d/04_dataset/1_dilabel/crowd_source/new_6lukou/batch-195/model/auto_label/1000/data/10"
default_bev_trail_folder ="/mnt/d/04_dataset/1_dilabel/crowd_source/new_6lukou/batch-195/model/auto_label/1000/bev_mapping/bev_line_track/10"
default_debug_vis=0
# default_debug_vis=1
parser = argparse.ArgumentParser()
parser.add_argument('--data_trail_folder', type=str, default=dafault_data_trail_folder)
parser.add_argument('--bev_trail_folder', type=str, default=default_bev_trail_folder)
parser.add_argument('--debug_vis', type=int, default=default_debug_vis)
parser.add_argument('--info_json_path', type=str, default=" ")

if __name__ == "__main__":
    args = parser.parse_args()
    log.set_log_file_path(os.path.join(args.bev_trail_folder, f"log/bev_line_track.log"))
    log.debug("==============================================================")
    log.debug("=========            Hello bev_line_track             ========")
    log.debug("==============================================================")
    bevtrail: BEVTrail = BEVTrail(args.data_trail_folder, args.bev_trail_folder, args.debug_vis, args.info_json_path)
    # bevtrail.run()
    bevtrail.run_qzc()
