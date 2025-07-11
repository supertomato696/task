# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 18:57
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : crosswalk.py
import copy
import os
from dataclasses import dataclass
from typing import Tuple, List, Dict

import pandas as pd
from geopandas import GeoDataFrame, GeoSeries
import matplotlib

matplotlib.use('agg')
import matplotlib.pyplot as plt
from shapely import Polygon, LineString
from shapely.ops import unary_union

from src.errors import DataEmpty
from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluatorParams

import matplotlib.patches as mpatches


class PolygonPairEvaluationResult:
    """评测结果类，封装IOU、Hausdorff距离和重心距离。"""

    def __init__(self, iou: float = 0, hausdorff_distance: float = 0,
                 centroid_distance: float = 0,
                 std_area: float=0, intersect_area: float=0,
                 relation_geom: LineString = None,
                 arrow_pointy_distance=float('inf')):
        self.iou = iou
        self.hausdorff_distance = hausdorff_distance
        self.centroid_distance = centroid_distance
        self.arrow_tip_distance = arrow_pointy_distance

        self.relation_geom = relation_geom
        # FIXME 这里应该搞个setter，赶时间先这么顶住先吧....
        self.std_area = std_area
        self.intersect_area = intersect_area

    def __repr__(self):
        return (f"EvaluationResult(IOU={self.iou:.4f}, "
                f"Hausdorff Distance={self.hausdorff_distance:.4f} 米, "
                f"Centroid Distance={self.centroid_distance:.4f} 米)")


@dataclass
class PolygonEvaluateSummary:
    """
    Polygon校验的结果汇总
    Pydantic用累了换个口味...
    """
    title: str
    total_true_polygon_count: int
    total_input_polygon_count: int
    total_input_area: float  # 召回率
    matched_pairs: list[Tuple[Polygon, Polygon]]
    unmatched_polygons: list[Polygon]
    evaluation_results: list[PolygonPairEvaluationResult]
    total_std_area: float  # 真值总面积
    total_intersection_area: float  # 总相交面积
    total_filtered_area: float  # 输入总面积(filter过的)
    intersect_percentage: float
    truth_polygon_count: int
    input_polygon_count: int

    def get_dict(self) -> Dict:
        """
        获取评测结果的文本描述信息。

        Returns:
            Dict: 包含评测指标及相关信息的字典
        """
        #FIXME 临时解决方案，应输出为数字或null

        # --- 初始化统计值和占位符 ---
        avg_hausdorff_str = "N/A"
        avg_centroid_offset_str = "N/A"
        iou_str = "N/A"
        accuracy_str = "N/A"
        recall_str = f"{self.intersect_percentage:.2f}%" # 召回率通常基于真值面积，这里直接用传入的

        # --- 计算平均距离 (如果结果列表不为空) ---
        num_results = len(self.evaluation_results)
        if num_results > 0:
            avg_hausdorff = sum(e.hausdorff_distance for e in self.evaluation_results) / num_results
            avg_centroid_offset = sum(e.centroid_distance for e in self.evaluation_results) / num_results
            avg_hausdorff_str = f"{avg_hausdorff:.4f}"
            avg_centroid_offset_str = f"{avg_centroid_offset:.4f}"
            # avg_iou (单项平均) 如果需要可以类似计算

        # --- 计算整体 IOU (Intersection over Union) ---
        # Union = Area(A) + Area(B) - Intersection(A, B)
        total_union_area = self.total_std_area + self.total_filtered_area - self.total_intersection_area
        tolerance = 1e-9 # 定义浮点数比较的容差

        # 检查 Union 面积是否有效 (大于容差)
        if abs(total_union_area) > tolerance:
            iou_val = self.total_intersection_area / total_union_area
            # IOU 理论上应该在 0 到 1 之间，可以加个钳制以防意外
            iou_val = max(0.0, min(1.0, iou_val))
            iou_str = f"{iou_val:.4f}"
        # 如果 Union 面积接近零
        elif abs(self.total_intersection_area) <= tolerance:
            # 如果相交面积也接近零 (0/0 的情况)，可以视为 IOU=0 或 IOU=1 (如果真值和输入面积都为0)
            # 这里我们保守地设为 0，或者根据具体业务定义
             if abs(self.total_std_area) <= tolerance and abs(self.total_filtered_area) <= tolerance:
                 iou_str = "1.0000 (Both Areas 0)" # 如果两者都是0，可以认为是完美匹配
             else:
                 iou_str = "0.0000 (Union Area 0)"
        else:
            # Union 面积为0，但相交面积不为0 (理论上不可能，除非面积计算有误或为负)
            # 这意味着 A 和 B 完全重合且面积大于 0
            # total_union_area = A + B - I = 0 => A + B = I. Since I <= A and I <= B, this implies A=B=I.
            # 所以这种情况 IOU 应该是 1
            iou_str = "1.0000 (Perfect Overlap)" # 或者标记为 '未定义（异常）'

        # --- 计算输入面积准确率 (Intersection / Filtered Input Area) ---
        # 检查 Filtered Input Area 是否有效 (大于容差)
        if abs(self.total_filtered_area) > tolerance:
            # 准确率 = 相交部分占输入部分的比例
            accuracy_val = (self.total_intersection_area / self.total_filtered_area) * 100
            # 理论上准确率 <= 100%，可以加钳制
            accuracy_val = max(0.0, min(100.0, accuracy_val))
            accuracy_str = f"{accuracy_val:.2f}%"
        # 如果 Filtered Input Area 接近零
        elif abs(self.total_intersection_area) <= tolerance:
            # 相交面积和输入面积都接近零 (0/0)
            # 这种情况可以定义为 100% (没有“错误”的面积) 或 0% (没有“正确”的面积) 或 N/A
            accuracy_str = "N/A (0/0)" # 或者 "100.00%", "0.00%"
        else:
            # 输入面积为0，但相交面积不为0 (理论上不可能)
            accuracy_str = "未定义 (输入面积为零)"

        # --- 构建最终的字典 ---
        stats_dict = {
            "总真值面积(平米)": f"{self.total_std_area:.2f}",
            "总输入面积(平米)": f"{self.total_input_area:.2f}",     # 这个是过滤前的总面积
            "参评的输入面积(平米)": f"{self.total_filtered_area:.2f}", # 这个是过滤后的，用于计算准确率
            "相交面积(平米)": f"{self.total_intersection_area:.2f}",
            "输入面积准确率": accuracy_str, # Precision = TP / (TP + FP) ~ Intersection / FilteredInput
            "输入面积召回率": recall_str,   # Recall = TP / (TP + FN) ~ Intersection / TruthArea
            "被匹配真值个数/输入个数/真值个数": f"{len(self.matched_pairs)} / {self.input_polygon_count} / {self.truth_polygon_count}",
            "整体IOU": iou_str,
            "整体平均H距离(米)": avg_hausdorff_str,
            "整体平均重心距离(米)": avg_centroid_offset_str
        }
        return stats_dict


def create_evaluation_summary(title: str,
                              total_true_polygon_count: int,
                              total_input_polygon_count: int,
                              total_input_area: float,
                              matched_pairs: List[Tuple[Polygon, Polygon]],
                              unmatched_polygons: List[Polygon],
                              evaluation_results: List[PolygonPairEvaluationResult],
                              total_std_area: float,
                              total_intersection_area: float,
                              total_filtered_area: float,
                              truth_polygon_count: int,
                              input_polygon_count: int
                              ) -> PolygonEvaluateSummary:
    """
    创建并返回一个 PolygonEvaluateSummary 实例。
    """
    summary = PolygonEvaluateSummary(
        title=title,
        total_true_polygon_count=total_true_polygon_count,
        total_input_polygon_count=total_input_polygon_count,
        total_input_area=total_input_area,
        matched_pairs=matched_pairs,
        unmatched_polygons=unmatched_polygons,
        evaluation_results=evaluation_results,
        total_std_area=total_std_area,
        total_intersection_area=total_intersection_area,
        total_filtered_area=total_filtered_area,
        intersect_percentage=(total_intersection_area / total_std_area * 100) if total_std_area else 0,
        truth_polygon_count=truth_polygon_count,
        input_polygon_count=input_polygon_count
    )
    return summary


class CrosswalkEvaluator(object):
    """
        仅数量化车道线评测类，用于仅数量化的评测道路线的真值数据与输入数据。

        :param truth_intersection: 真值路口对象。
        :param input_intersection: 带评测路口对象
        :param evaluator_setting: 评测工具配置。
    """

    def __init__(self, truth_intersection: Intersection, input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams):
        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.tool_config = evaluator_setting
        self.truth_crosswalk_gdf = self.truth_intersection.crosswalks
        self.input_crosswalk_gdf = self.input_intersection.crosswalks

        if self.truth_crosswalk_gdf is None or self.input_crosswalk_gdf is None:
            raise DataEmpty(
                f"数据缺失: 真值非空{self.truth_crosswalk_gdf is not None}, 待评测值值非空{self.input_crosswalk_gdf is not None}")

        # 默认的crs同步自输入的数据
        self.standard_utm_crs = self.input_intersection.crosswalks.crs  # 可以改成crosswalk更合适

        self.total_filtered_area = self.filtered_input_crosswalks["geometry"].area.sum()
        self.total_intersection_area = 0
        self.total_std_area = self.truth_crosswalk_gdf["geometry"].area.sum()

        self.max_centroid_distance = self.tool_config.polygon_match_distance_meter

        self.title = input_intersection.name

    def filter_input_crosswalks(self, truth_crosswalks_gdf: GeoDataFrame = None,
                                input_crosswalks_gdf: GeoDataFrame = None) -> GeoDataFrame:
        if not truth_crosswalks_gdf:
            truth_crosswalks_gdf = self.truth_crosswalk_gdf
        if not input_crosswalks_gdf:
            input_crosswalks_gdf = self.input_crosswalk_gdf

        gdf_truth = truth_crosswalks_gdf.copy().reset_index(drop=True)
        gdf_truth["buffer"] = gdf_truth.geometry.buffer(20)

        # 构建空间索引，提升相交查询效率
        sindex = gdf_truth.sindex

        # 2. 利用并查集（Union-Find）对真值多边形进行聚类
        # 初始化每个多边形为各自的根节点
        parent = {idx: idx for idx in gdf_truth.index}

        def find(i):
            """找到节点 i 的根，同时路径压缩"""
            i = int(i)
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            """合并两个节点所在的集合"""
            root_i = find(i)
            root_j = find(j)
            if root_i != root_j:
                parent[root_j] = root_i

        # 遍历每个真值多边形，根据缓冲区相交情况合并到同一聚类中
        for idx, row in gdf_truth.iterrows():
            buffered_geom = row["buffer"]
            # 利用空间索引查询与当前多边形可能相交的候选项（候选的边界框可能相交）
            possible_matches_index = list(sindex.intersection(buffered_geom.bounds))
            for other_idx in possible_matches_index:
                # 避免重复比较，可以要求只比较索引大于当前的
                if other_idx <= idx:
                    continue
                other_buffer = gdf_truth.iloc[other_idx]["buffer"]
                if buffered_geom.intersects(other_buffer):
                    union(idx, other_idx)

        # 根据并查集结果，将各真值多边形按聚类根节点归类到一起
        clusters = {}
        for idx in gdf_truth.index:
            root = find(idx)
            clusters.setdefault(root, []).append(idx)

        # 3. 对每个聚类区域，利用原始几何（不含缓冲区）做联合，形成评测区域
        cluster_geoms = {}
        for root, indices in clusters.items():
            # 注意：这里选用原始真值多边形，若希望使用扩展后的区域进行评测，可用缓冲区
            union_geom = unary_union(gdf_truth.loc[indices, "geometry"].values)
            cluster_geoms[root] = union_geom

        # 4. 利用聚类评测区域对输入识别结果进行筛选
        selected_detection_ids = []

        for det_idx, det_row in input_crosswalks_gdf.iterrows():
            det_geom = det_row.geometry
            # 检查输入多边形是否与任一聚类区域相交
            for cluster_geom in cluster_geoms.values():
                if det_geom.intersects(cluster_geom):
                    selected_detection_ids.append(det_idx)
                    break

        # 获取参与评测的输入多边形结果子集
        gdf_eval_input = input_crosswalks_gdf.loc[selected_detection_ids]

        return gdf_eval_input

    def calculate_iou(self, std_polygon: Polygon, test_polygon: Polygon) -> float:
        """
        计算交并比（IOU）
        交并比（IoU，Intersection over Union）是目标检测和分割任务中一个重要的衡量标准。
        ‌它表示两个边界框（或分割掩模）的交集区域面积与它们的并集区域面积之比。交并比的取值范围在0到1之间，值越大表示两个边界框之间的重叠程度越高。
        当IoU为1时，表示两个边界框完全重叠；当IoU为0时，表示两个边界框没有任何重叠。‌

        IOU = 交集面积 / 并集面积

        returns:
        - IOU 值（0 到 1 之间）
        """
        intersection_area = std_polygon.intersection(test_polygon).area
        # FIXME 在里面汇总不太好，下次提到外面去，赶时间暂时先这样.
        # self.total_intersection_area += intersection_area
        # self.total_std_area += std_polygon.area
        union_area = std_polygon.union(test_polygon).area
        iou = intersection_area / union_area if union_area != 0 else 0
        return iou

    @staticmethod
    def calculate_hausdorff_distance(std_polygon: Polygon, test_polygon: Polygon) -> float:
        """
        用Shapely内置方法计算Hausdorff距离

        returns:
        - Hausdorff 距离值（浮点数，以米为单位）
        """
        hausdorff = std_polygon.hausdorff_distance(test_polygon)
        return hausdorff

    @staticmethod
    def calculate_centroid_distance(std_polygon: Polygon, test_polygon: Polygon) -> float:
        """
        计算两个多边形重心距离

        returns:
        - 重心距离值（浮点数，以米为单位）
        """
        centroid_std = std_polygon.centroid
        centroid_test = test_polygon.centroid
        centroid_dist = centroid_std.distance(centroid_test)
        return centroid_dist

    def evaluate_metrics(self, std_polygon: Polygon, test_polygon: Polygon) -> PolygonPairEvaluationResult:
        """
        评测各项指标，计算IOU、Hausdorff距离、重心距离
        只对匹配上的真值和输入多边形计算上述指标
        returns:
        - EvaluationResult 实例
        """
        # 计算各指标
        iou = self.calculate_iou(std_polygon, test_polygon)
        hausdorff = self.calculate_hausdorff_distance(std_polygon, test_polygon)
        centroid_dist = self.calculate_centroid_distance(std_polygon, test_polygon)
        # 构造关系线
        centroid_std = std_polygon.centroid
        centroid_test = test_polygon.centroid
        relation_line_geom = LineString([centroid_std, centroid_test])
        # self.total_std_area += std_polygon.area
        intersection_area = std_polygon.intersection(test_polygon).area
        self.total_intersection_area += intersection_area
        # 返回评测结果实例
        return PolygonPairEvaluationResult(iou=iou,
                                           hausdorff_distance=hausdorff, centroid_distance=centroid_dist,
                                           #    std_area=std_polygon.area,
                                           relation_geom=relation_line_geom,
                                           intersect_area=intersection_area
                                           )

    @staticmethod
    def match_polygons(true_polygons: GeoSeries, filtered_input_polygons: GeoSeries,
                       max_distance: float = 5) -> tuple[list[tuple[Polygon, Polygon]], list[Polygon]]:
        """
        二维数组，以真值为基础找匹配
        1. 距离真值多边形质心距离超过5米且与真值多边形没有任何交集的输入多边形视为无法匹配；
        2. 如果有多个多边形满足质心距离在5米内，那么覆盖度高的视为匹配。
        3. 如果覆盖度相同，那么质心距离最近的视为匹配。
        4. 如果覆盖度和质心距离最接近，那么任选一个视为匹配。
        5. 一个真值多边形只会被一个输入多边形匹配。但一个输入多边形可以和多个真值匹配。
        :param true_polygons: 真值多边形
        :param input_polygons: 输入多边形
        :param max_distance: 计算匹配的质心最大距离默认5米
        :return:
        """
        matched: dict[Polygon: Polygon] = {}

        unmatched_input_polygons = copy.deepcopy(filtered_input_polygons.tolist())

        for true_poly in true_polygons:
            # 候选列表
            candidates = []
            for input_poly in unmatched_input_polygons:
                true_centroid = true_poly.centroid
                input_centroid = input_poly.centroid
                distance = true_centroid.distance(input_centroid)
                if distance <= max_distance or true_poly.intersects(input_poly):
                    coverage = true_poly.intersection(input_poly).area / true_poly.area
                    candidates.append((input_poly, coverage, distance, input_centroid))
            if len(candidates) > 1:
                # 首先按照 coverage（第二个元素）降序排序：coverage 越大，优先级越高。
                # 如果 coverage 相同，则按照 distance（第三个元素）升序排序：distance 越小，优先级越高。
                candidates.sort(key=lambda x: (-x[1], x[2]))
                # sorted_candidates = sorted(candidates,key=lambda x: x[1])
                matched_poly = candidates[0][0]
                matched[true_poly] = matched_poly
                # unmatched_input_polygons.remove(matched_poly) # 不可以二次匹配
            elif len(candidates) == 0:
                continue
            else:
                matched[true_poly] = candidates[0][0]
        # 筛选出未匹配的输入多边形
        # matched将字典中的值收集到一个集合中，用于快速查找
        dict_values_set = set(matched.values())
        # 过滤出不在字典值集合中的polygon
        unmatched_input_polygons = [value for value in filtered_input_polygons if value not in dict_values_set]

        return list(matched.items()), unmatched_input_polygons

    def evaluate(self) -> PolygonEvaluateSummary:
        """
        执行多边形匹配与评测，返回所有评测结果和平均分

        returns:
        - matched_pairs: List of tuples (真值多边形, 测试多边形)
        - evaluations: List of EvaluationResult 实例
        """
        matched_pairs, unmatched_list = self.match_polygons(
            true_polygons=self.truth_intersection.crosswalks["geometry"],
            filtered_input_polygons=self.filtered_input_crosswalks["geometry"],
            max_distance=self.max_centroid_distance)

        evaluation_results = []
        logger.debug("\n开始进行综合评测...")
        for idx, (std_poly, test_poly) in enumerate(matched_pairs, start=1):
            eval_result = self.evaluate_metrics(std_poly, test_poly)
            logger.debug(f"评测多边形对 {idx}:{eval_result}")
            evaluation_results.append(eval_result)

        return create_evaluation_summary(
            title=self.title,
            total_input_area=self.input_crosswalk_gdf["geometry"].area.sum(),
            # recall=len(matched_pairs) / len(self.input_crosswalk_gdf["geometry"]), # 无意义，可删去
            total_true_polygon_count=len(self.input_crosswalk_gdf["geometry"]),
            total_input_polygon_count=len(self.filtered_input_crosswalks["geometry"]),
            matched_pairs=matched_pairs,
            unmatched_polygons=unmatched_list,
            evaluation_results=evaluation_results,
            total_std_area=self.total_std_area,
            total_filtered_area=self.total_filtered_area,
            total_intersection_area=self.total_intersection_area,
            truth_polygon_count=self.truth_crosswalk_gdf.shape[0],
            input_polygon_count=self.filtered_input_crosswalks.shape[0],
        )

    def save_as_gkpg(self, results: PolygonEvaluateSummary, gpkg_path):

        # 确保目标目录存在
        output_dir = os.path.dirname(gpkg_path)
        os.makedirs(output_dir, exist_ok=True)

        # 将结果组织为gdf，匹配pair的匹配情况附着在relation_geom线上。
        # 假设 results 是包含 PolygonPairEvaluationResult 对象的列表
        data = []
        res_list: list[PolygonPairEvaluationResult] = results.evaluation_results
        for res in res_list:
            # 如果某些结果的relation_geom有可能为 None，此处可做检查
            if res.relation_geom is None:
                continue
            data.append({
                "geometry": res.relation_geom,
                "hausdorff_distance": res.hausdorff_distance,
                "centroid_distance": res.centroid_distance,
                "iou": res.iou
            })

        if not data:
            logger.warning("没有有效的结果数据，将创建空的 '配对关系与独立评测结果' 图层。")
            result_columns = ["geometry", "hausdorff_distance", "centroid_distance", "iou"]
            # 创建一个空的 DataFrame 指定列
            # 将其转换为空的 GeoDataFrame，明确指定 geometry 列和 CRS
            # 需要确保 geometry 列是 GeoSeries 类型，即使是空的
            gdf = GeoDataFrame(pd.DataFrame(columns=result_columns), geometry='geometry', crs=self.standard_utm_crs)
        else:
            # 生成GeoDataFrame其中"geometry"列作为几何列,同时设置crs
            gdf = GeoDataFrame(data, geometry="geometry", crs=self.standard_utm_crs)
            # self.standard_utm_crs现在应该是32650，这里统一做个转换
            gdf = gdf.to_crs(epsg=4326)

        gdf.to_file(gpkg_path, layer="配对关系与独立评测结果", driver="GPKG")

        self.truth_intersection.crosswalks.to_crs(epsg=4326).to_file(gpkg_path, layer="真值人行横道", driver="GPKG")
        self.input_intersection.crosswalks.to_crs(epsg=4326).to_file(gpkg_path, layer="输入人行横道", driver="GPKG")
        self.truth_intersection.lane_lines.to_crs(epsg=4326).to_file(gpkg_path, layer="真值车道线参考图层",
                                                                     driver="GPKG")
        self.truth_intersection.stop_lines.to_crs(epsg=4326).to_file(gpkg_path, layer="真值停止线参考图层",
                                                                     driver="GPKG")

        logger.debug("人行横道写gpkg搞定")

    def save_svg(self, title: str, results: PolygonEvaluateSummary,
                 is_to_4326=True,
                 is_visualize=False,  # 调试的时候可以开一下
                 vector_file_path=None):

        # 创建绘图窗口
        fig, ax = plt.subplots(figsize=(24, 18))

        # -------------------------------
        # 1. 坐标系转换
        # -------------------------------
        if is_to_4326:
            stop_lines = self.truth_intersection.stop_lines.to_crs(epsg=4326)
            arrows = self.truth_intersection.arrows.to_crs(epsg=4326)
            lane_lines = self.truth_intersection.lane_lines.to_crs(epsg=4326)
            road_boundaries = self.truth_intersection.road_boundaries.to_crs(epsg=4326)
            truth_crosswalks = self.truth_intersection.crosswalks.to_crs(epsg=4326)
            input_crosswalks = self.input_crosswalk_gdf.to_crs(epsg=4326)
            filtered_crosswalks = self.filtered_input_crosswalks.to_crs(epsg=4326)
        else:
            stop_lines = self.truth_intersection.stop_lines
            arrows = self.truth_intersection.arrows
            lane_lines = self.truth_intersection.lane_lines
            road_boundaries = self.truth_intersection.road_boundaries
            truth_crosswalks = self.truth_intersection.crosswalks
            input_crosswalks = self.input_crosswalk_gdf
            filtered_crosswalks = self.filtered_input_crosswalks

        # -------------------------------
        # 2. 绘制参考要素：停止线、车道线、路界和箭头
        # -------------------------------
        for feature in [stop_lines, lane_lines, road_boundaries, arrows]:
            feature.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)

        # -------------------------------
        # 3. 绘制真值人行横道
        # -------------------------------
        truth_crosswalks.plot(ax=ax, alpha=0.8, color='#1b315e')

        # -------------------------------
        # 4. 绘制输入的人行横道（分匹配和未匹配）
        # -------------------------------
        matched_inputs = []
        unmatched_inputs = []
        orphan_inputs = []

        # 遍历 input_crosswalks.geometry，假设 results.unmatched_polygons 中存储的均为 Shapely 几何对象
        for input_poly in filtered_crosswalks.geometry:
            if input_poly in results.unmatched_polygons:
                unmatched_inputs.append(input_poly)
            else:
                matched_inputs.append(input_poly)

        for input_cs in input_crosswalks.geometry:
            if (input_cs not in matched_inputs) or (input_cs not in unmatched_inputs):
                orphan_inputs.append(input_cs)

        # 绘制已匹配的输入人行横道
        if matched_inputs:
            GeoSeries(matched_inputs, crs=input_crosswalks.crs) \
                .plot(ax=ax, facecolor='green', edgecolor='none', alpha=0.5)
        # 绘制未匹配的输入人行横道
        if unmatched_inputs:
            GeoSeries(unmatched_inputs, crs=input_crosswalks.crs) \
                .plot(ax=ax, facecolor='#c63c26', edgecolor='none', alpha=0.9)

        # 绘制未参与批配输入人行横道
        if orphan_inputs:
            GeoSeries(orphan_inputs, crs=input_crosswalks.crs) \
                .plot(ax=ax, facecolor='orange', edgecolor='none', alpha=0.5)
        # -------------------------------
        # 5. 添加图例与标题
        # -------------------------------
        # 这里手动创建代理对象作为图例内容
        patch_truth = mpatches.Patch(facecolor='#1b315e', edgecolor='black', alpha=0.8, label='真值多边形')
        legend_handles = [patch_truth]

        if matched_inputs:
            patch_matched = mpatches.Patch(facecolor='green', edgecolor='none', alpha=0.5, label='已匹配测试多边形')
            legend_handles.append(patch_matched)
        if unmatched_inputs:
            patch_unmatched = mpatches.Patch(facecolor='#c63c26', edgecolor='none', alpha=0.9, label='未匹配测试多边形')
            legend_handles.append(patch_unmatched)
        if orphan_inputs:
            patch_unmatched = mpatches.Patch(facecolor='orange', edgecolor='none', alpha=0.5, label='未参与匹配')
            legend_handles.append(patch_unmatched)

        ax.legend(handles=legend_handles, loc='upper right')
        ax.set_title(title)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("Longitude")
        ax.set_ylabel("Latitude")
        plt.grid(True)

        # -------------------------------
        # 6. 添加统计信息
        # -------------------------------
        stats_text = results.get_dict()
        stats_text_formatted = "\n".join([f"{key}: {value}" for key, value in stats_text.items()])

        logger.info(f"\n评测任务:{title} 结论:\n{stats_text_formatted}")
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.2)
        ax.text(1.01, 0.2, stats_text_formatted, transform=ax.transAxes, fontsize=9,
                verticalalignment='center', bbox=props)
        # 调整图形布局，确保文本框不会被裁剪
        plt.subplots_adjust(right=0.8)

        # -------------------------------
        # 7. 保存和/或显示图像
        # -------------------------------
        if vector_file_path:
            plt.savefig(vector_file_path, format='svg', bbox_inches='tight')
            logger.debug(f"已保存 svg 到：{vector_file_path}")
            plt.close(fig)
        if is_visualize:
            plt.show()
            plt.close(fig)

    @property
    def filtered_input_crosswalks(self) -> GeoDataFrame:
        return self.filter_input_crosswalks()
