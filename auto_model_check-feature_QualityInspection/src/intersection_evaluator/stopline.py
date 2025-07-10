# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 20:32
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : stopline.py
import math
import os
from typing import Optional, List, Dict

import numpy as np
from geopandas import GeoDataFrame
import matplotlib

from src.plotter import LineConfig, GeoEvaPlotter

matplotlib.use('agg')
import matplotlib.pyplot as plt
from pydantic import BaseModel, Field
from shapely import LineString, Point
from shapely.ops import unary_union, linemerge

from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluatorParams


def get_line_angle(line: LineString) -> float:
    """
    根据 LineString 的起点和终点计算角度（弧度值）
    如果line只有一个点或者无效返回None。
    """
    if not isinstance(line, LineString) or len(line.coords) < 2:
        return None
    x0, y0 = line.coords[0]
    x1, y1 = line.coords[-1]
    # 当起点和终点重合时返回 None
    if math.isclose(x0, x1) and math.isclose(y0, y1):
        return None
    angle = math.atan2(y1 - y0, x1 - x0)
    return angle


def angle_difference(a1: float, a2: float) -> float:
    """
    计算两个角度之间的绝对差值（弧度值），自动归一化到 [0, pi]
    """
    if a1 is None or a2 is None:
        return np.inf
    diff = abs(a1 - a2) % (2 * math.pi)
    return diff if diff <= math.pi else 2 * math.pi - diff


class MatchResult(BaseModel):
    """配对的结果"""
    combined_metric: Optional[float] = None
    angle_diff_deg: Optional[float] = None
    centroid_distance: Optional[float] = None
    length_diff: Optional[float] = None
    hausdorff_distance: Optional[float] = None

    # 保存原始几何信息，用于计算几何（匹配成功的才有两个几何）
    input_index: Optional[int] = None
    truth_index: Optional[int] = None
    input_line: Optional[LineString] = None
    truth_line: Optional[LineString] = None

    class Config:
        arbitrary_types_allowed = True

    @property
    def geometry(self) -> Optional[LineString]:
        """
        返回一条连接输入线和真值线中点的LineString几何。
        如果任意一侧几何不存在，则返回None。
        """
        if self.input_line is None or self.truth_line is None:
            return None

        def midpoint(line: LineString) -> Point:
            coords = list(line.coords)
            # 计算第一点与最后一点中点
            x_mid = (coords[0][0] + coords[-1][0]) / 2.0
            y_mid = (coords[0][1] + coords[-1][1]) / 2.0
            return Point(x_mid, y_mid)

        pt_input = midpoint(self.input_line)
        pt_truth = midpoint(self.truth_line)
        return LineString([pt_input, pt_truth])

    def to_dict(self) -> dict:
        """
        转换成字典格式，排除原始input_line与truth_line字段，
        添加计算后的 geometry 字段，方便生成 GeoDataFrame。
        """
        data = self.dict(exclude={"input_line", "truth_line"})
        data["geometry"] = self.geometry  # 这里存放的是 shapely 几何对象
        return data


class StopLineEvaluateResult(BaseModel):
    title: str = None

    accuracy: float = Field(title="准确率", description="准确率Precision = TP/(TP+FP)")
    precision: float = Field(title="精准率")
    recall: float = Field(title="召回率")
    max_hausdorff: float = Field(title="Hausdorff最大距离(米)", description="Hausdorff最大距离(米)")
    min_hausdorff: float = Field(title="Hausdorff最小距离(米)", description="Hausdorff最小距离(米)")
    max_rotate_angle: float
    min_rotate_angle: float
    max_len_diff: float = Field(title="最大长度差(米)", description="最大长度差(米)")
    min_len_diff: float = Field(title="最小长度差(米)", description="最小长度差(米)")

    match_result: List[MatchResult]

    @property
    def f1_score(self) -> float:
        """
        计算F1分数，综合考虑精准率和召回率。

        :return: float F1分数
        """
        return 2 * (self.precision * self.recall) / (self.precision + self.recall) if (
                                                                                              self.precision + self.recall) > 0 else 0

    def get_dict(self) -> Dict:
        stats_dict = {
            "准确率": f"{self.accuracy * 100:.2f}%",
            "召回率": f"{self.recall * 100:.2f}%",
            "F1分数": f"{self.f1_score:.4f}",
            "Hausdorff最大距离(米)": f" {self.max_hausdorff:.2f}",
            "Hausdorff最小距离(米)": f" {self.min_hausdorff:.2f}",
            "最大偏转角": f"{self.max_rotate_angle:.2f}°",
            "最小偏转角": f"{self.min_rotate_angle:.2f}°",
            "最大长度差(米)": f"{self.max_len_diff:.2f}",
            "最小长度差(米)": f"{self.min_len_diff:.2f}"
        }
        return stats_dict

    class Config:
        arbitrary_types_allowed = True  # 允许任意类型的字段


class StopLineEvaluator(object):
    def __init__(self, truth_intersection: Intersection,
                 input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams):

        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.tool_config = evaluator_setting

        self.standard_utm_crs = self.input_intersection.stop_lines.crs

        # 将真值停止线数据帧赋值给实例变量
        self.truth_stopline_gdf = self.truth_intersection.stop_lines
        self.input_stopline_gdf = self.input_intersection.stop_lines
        # 由于真值停止线是按照车道来画的，这里先合并真值停止线
        self.truth_stopline_gdf = self.__merge_lines(self.truth_stopline_gdf).copy()
        self.input_stopline_gdf = self.__filter_input_stoplines().copy()

        self._precompute_geometric_properties(self.truth_stopline_gdf)
        self._precompute_geometric_properties(self.input_stopline_gdf)

        # 重置索引
        self.input_stopline_gdf: GeoDataFrame = self.input_stopline_gdf.reset_index(drop=True)
        self.truth_stopline_gdf: GeoDataFrame = self.truth_stopline_gdf.reset_index(drop=True)

    @staticmethod
    def __merge_lines(gdf):
        """
        合并GeoDataFrame中的线段为一个几何对象。

        :param gdf: 包含线几何的GeoDataFrame。
        :return: 合并后的GeoDataFrame，包含合并后的几何和长度信息。
        """
        merged_geometry = unary_union(gdf['geometry'])

        if merged_geometry.geom_type == 'MultiLineString':
            # 如果是MultiLineString，使用linemerge合并连续的LineString
            merged_lines = list(linemerge(merged_geometry).geoms)
        else:
            # 本身就是LineString，直接转换为列表
            merged_lines = [merged_geometry]

        # 创建一个新的GeoDataFrame来存储合并后的几何
        merged_gdf = GeoDataFrame(geometry=merged_lines, crs=gdf.crs)
        merged_gdf['length'] = merged_gdf['geometry'].length  # 计算每条线的长度
        return merged_gdf

    @staticmethod
    def _precompute_geometric_properties(gdf: GeoDataFrame):
        """
        预计算 GeoDataFrame 中每一条停止线的质心、角度与长度，并增加对应列。
        """
        centroids = []
        angles = []
        lengths = []
        for geom in gdf.geometry:
            if geom is None or geom.is_empty:
                centroids.append(None)
                angles.append(None)
                lengths.append(0)
                continue
            try:
                centroids.append(geom.centroid)
            except Exception:
                centroids.append(None)
            angle = get_line_angle(geom)
            angles.append(angle)
            lengths.append(geom.length)
        gdf.loc[:, "centroid"] = centroids
        gdf.loc[:, "angle"] = angles
        gdf.loc[:, "length"] = lengths
        logger.debug("precompute stopline geometric properties done~")

    @staticmethod
    def calculate_direction(line):
        """
        计算线段的方向向量。

        :param line: 输入的LineString对象。
        :return: 单位方向向量。
        """
        start_point = np.array(line.coords[0])  # 起点
        end_point = np.array(line.coords[-1])  # 终点
        direction_vector = end_point - start_point  # 方向向量
        direction_vector = direction_vector / np.linalg.norm(direction_vector)  # 单位化
        return direction_vector

    def __filter_input_stoplines(self, buffer_radius: float = 30.0) -> GeoDataFrame:
        """
        针对真值停止线的中心生成buffer，利用并查集聚类，将在任一聚类内的输入停止线留下，其余过滤掉。

        :param buffer_radius: float, 真值线中心的缓冲半径，默认30米
        :return: GeoDataFrame，包含所有需要评测的输入停止线
        """
        # 1. 为每个真值停止线生成缓冲区
        truth_buffers = []  # 存放每个真值缓冲区的几何
        for idx, row in self.truth_stopline_gdf.iterrows():
            # 使用真值停止线的中心（centroid）生成缓冲区
            centroid = row.geometry.centroid
            buf = centroid.buffer(buffer_radius)
            truth_buffers.append(buf)

        # 2. 使用并查集（Union-Find）对缓冲区聚类
        n = len(truth_buffers)
        parent = list(range(n))

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]  # 路径压缩
                x = parent[x]
            return x

        def union(x: int, y: int) -> None:
            rx, ry = find(x), find(y)
            if rx != ry:
                parent[ry] = rx

        # 遍历所有缓冲区对，若相交则 union
        for i in range(n):
            for j in range(i + 1, n):
                if truth_buffers[i].intersects(truth_buffers[j]):
                    union(i, j)

        # 根据并查集的父节点信息整理聚类
        clusters = {}
        for i in range(n):
            root = find(i)
            if root not in clusters:
                clusters[root] = []
            clusters[root].append(truth_buffers[i])

        # 对每个聚类内的缓冲区进行合并，形成联合区域
        cluster_unions = [unary_union(clusters[k]) for k in clusters]

        # 3. 筛选输入停止线
        valid_rows = []
        for idx, row in self.input_stopline_gdf.iterrows():
            # 以输入停止线的中心作为代表
            centroid: Point = row.geometry.centroid
            # 判断该输入停止线中心是否落入任一聚类合并区域内
            in_cluster = any(cluster.contains(centroid) for cluster in cluster_unions)
            if in_cluster:
                valid_rows.append(row)

        # 4. 返回符合条件的输入停止线（评测候选）
        # 注意：valid_rows 是一个 list，需要转换为 GeoDataFrame
        if valid_rows:
            filtered_gdf = GeoDataFrame(valid_rows, columns=self.input_stopline_gdf.columns, crs=self.standard_utm_crs)
        else:
            # 如果没有符合条件的输入停止线，返回空的 GeoDataFrame
            filtered_gdf = GeoDataFrame(columns=self.input_stopline_gdf.columns, crs=self.standard_utm_crs)
            logger.error()

        return filtered_gdf

    # @property
    # def filtered_input_stoplines(self) -> GeoDataFrame:
    #     """获取判定为应该被评测的输入停止线"""
    #     return self.__filter_input_stoplines()

    def match_stoplines(self,
                        angle_weight: float = 0.1,
                        centroid_weight: float = 0.4,
                        length_weight: float = 0.2,
                        hausdorff_weight: float = 0.3,
                        match_threshold: float = 30) -> List['MatchResult']:
        """
        对真实停止线与输入停止线进行匹配，基于真值先在其周围生成buffer，
        然后从候选输入中选择最匹配的输入。返回MatchResult列表。

        :param angle_weight: 角度差度量权重
        :param centroid_weight: 中点距离权重
        :param length_weight: 长度差权重
        :param hausdorff_weight: Hausdorff 距离权重
        :param match_threshold: 匹配度量阈值，只有综合度量小于该阈值才认为匹配成功
        :return: List[MatchResult]
        """
        results = []
        # 构建输入停止线的空间索引，用于高效查找候选
        input_sindex = self.input_stopline_gdf.sindex
        used_input_indices = set()  # 记录已经匹配的输入停止线

        # 遍历每个真值停止线（以真值为主）
        for truth_idx, truth_row in self.truth_stopline_gdf.iterrows():
            truth_geom = truth_row.geometry
            truth_centroid = truth_geom.centroid
            truth_angle = truth_row.angle
            truth_length = truth_row.length

            if truth_centroid is None or truth_geom is None or truth_geom.is_empty:
                # 如果该真值停止线几何无效，则忽略
                continue

            # 以真值停止线的中心构建一个buffer，这里使用 buffer(1) 如你所示，
            # 如果需要根据真实长度构建 buffer，可以调整参数
            buffer_bounds = truth_geom.buffer(1).bounds

            # sindex.intersection 返回的是行的“位置索引”
            candidate_input_idxs = list(input_sindex.intersection(buffer_bounds))

            best_candidate = None
            best_metrics = None
            best_combined_metric = np.inf

            # 在候选输入中选择最佳匹配
            for cand_idx in candidate_input_idxs:
                if cand_idx in used_input_indices:
                    continue  # 已经匹配则跳过

                # 使用 iloc 进行位置索引查找
                input_row = self.input_stopline_gdf.iloc[int(cand_idx)]
                input_geom = input_row.geometry
                input_centroid = input_geom.centroid
                input_angle = input_row.angle
                input_length = input_row.length

                if input_centroid is None or input_geom is None or input_geom.is_empty:
                    continue

                # 计算各个指标
                a_diff = math.degrees(angle_difference(input_angle, truth_angle))
                d_centroid = input_centroid.distance(truth_centroid)
                d_length = abs(input_length - truth_length)
                try:
                    d_hausdorff = input_geom.hausdorff_distance(truth_geom)
                except Exception:
                    d_hausdorff = np.inf

                combined_metric = (angle_weight * a_diff +
                                   centroid_weight * d_centroid +
                                   length_weight * d_length +
                                   hausdorff_weight * d_hausdorff)

                if combined_metric < best_combined_metric:
                    best_combined_metric = combined_metric
                    best_candidate = cand_idx
                    best_metrics = {
                        "angle_diff_deg": a_diff,
                        "centroid_distance": d_centroid,
                        "length_diff": d_length,
                        "hausdorff_distance": d_hausdorff
                    }

            if best_candidate is not None and best_combined_metric < match_threshold:
                used_input_indices.add(best_candidate)
                # 注意这里仍然采用 iloc
                input_geom = self.input_stopline_gdf.iloc[best_candidate].geometry
                results.append(MatchResult(
                    input_index=best_candidate,
                    truth_index=truth_idx,
                    combined_metric=best_combined_metric,
                    angle_diff_deg=best_metrics["angle_diff_deg"],
                    centroid_distance=best_metrics["centroid_distance"],
                    length_diff=best_metrics["length_diff"],
                    hausdorff_distance=best_metrics["hausdorff_distance"],
                    input_line=input_geom,
                    truth_line=truth_geom
                ))
            else:
                # 如果不匹配，可以根据需要添加未匹配的记录
                pass

        return results

    def match_stoplines_v2(self) -> List['MatchResult']:
        """
        对真实停止线与输入停止线进行匹配，基于真值先在其周围生成buffer，
        然后从候选输入中选择最匹配的输入。返回MatchResult列表。

        :param angle_weight: 角度差度量权重
        :param centroid_weight: 中点距离权重
        :param length_weight: 长度差权重
        :param hausdorff_weight: Hausdorff 距离权重
        :param match_threshold: 匹配度量阈值，只有综合度量小于该阈值才认为匹配成功
        :return: List[MatchResult]
        """
        results = []
        # 构建输入停止线的空间索引，用于高效查找候选
        input_sindex = self.input_stopline_gdf.sindex
        # used_input_indices = set()  # 记录已经匹配的输入停止线
        used_input_dict = {}
        results_dict = {}

        # 遍历每个真值停止线（以真值为主）
        for truth_idx, truth_row in self.truth_stopline_gdf.iterrows():
            truth_geom = truth_row.geometry
            truth_centroid = truth_geom.centroid
            truth_angle = truth_row.angle
            truth_length = truth_row.length

            if truth_centroid is None or truth_geom is None or truth_geom.is_empty:
                # 如果该真值停止线几何无效，则忽略
                continue

            # 以真值停止线的中心构建一个buffer，这里使用 buffer(1) 如你所示，
            # 如果需要根据真实长度构建 buffer，可以调整参数
            buffer_bounds = truth_geom.buffer(1).bounds

            # sindex.intersection 返回的是行的“位置索引”
            candidate_input_idxs = list(input_sindex.intersection(buffer_bounds))

            best_candidate = None
            best_metrics = None
            best_combined_metric = np.inf

            # 在候选输入中选择最佳匹配
            for cand_idx in candidate_input_idxs:
                # if cand_idx in used_input_indices:
                #     continue  # 已经匹配则跳过

                # 使用 iloc 进行位置索引查找
                input_row = self.input_stopline_gdf.iloc[int(cand_idx)]
                input_geom = input_row.geometry
                input_centroid = input_geom.centroid
                input_angle = input_row.angle
                input_length = input_row.length

                if input_centroid is None or input_geom is None or input_geom.is_empty:
                    continue

                # 计算各个指标
                a_diff = math.degrees(angle_difference(input_angle, truth_angle))
                d_centroid = input_centroid.distance(truth_centroid)
                d_length = abs(input_length - truth_length)
                try:
                    d_hausdorff = input_geom.hausdorff_distance(truth_geom)
                except Exception:
                    d_hausdorff = np.inf

                # combined_metric = (angle_weight * a_diff +
                #                    centroid_weight * d_centroid +
                #                    length_weight * d_length +
                #                    hausdorff_weight * d_hausdorff)
                # 最完美状态，所有diff都是0，就是输入和真值重合
                combined_metric = a_diff + d_centroid + d_length

                if combined_metric < best_combined_metric:
                    best_combined_metric = combined_metric
                    best_candidate = cand_idx
                    best_metrics = {
                        "angle_diff_deg": a_diff,
                        "centroid_distance": d_centroid,
                        "length_diff": d_length,
                        "hausdorff_distance": d_hausdorff
                    }

            if best_candidate is not None:
                # 如果这个最佳匹配已经有其他真值匹配了，则根据这个最佳匹配与两个真值之间的得分，取得分更优的那个
                if best_candidate in used_input_dict and used_input_dict[best_candidate] < best_combined_metric:
                    continue

                used_input_dict[best_candidate] = best_combined_metric

                # used_input_indices.add(best_candidate)
                # 注意这里仍然采用 iloc
                input_geom = self.input_stopline_gdf.iloc[best_candidate].geometry
                results_dict[best_candidate] = MatchResult(
                    input_index=best_candidate,
                    truth_index=truth_idx,
                    combined_metric=best_combined_metric,
                    angle_diff_deg=best_metrics["angle_diff_deg"],
                    centroid_distance=best_metrics["centroid_distance"],
                    length_diff=best_metrics["length_diff"],
                    hausdorff_distance=best_metrics["hausdorff_distance"],
                    input_line=input_geom,
                    truth_line=truth_geom
                )
                # results.append(MatchResult(
                #     input_index=best_candidate,
                #     truth_index=truth_idx,
                #     combined_metric=best_combined_metric,
                #     angle_diff_deg=best_metrics["angle_diff_deg"],
                #     centroid_distance=best_metrics["centroid_distance"],
                #     length_diff=best_metrics["length_diff"],
                #     hausdorff_distance=best_metrics["hausdorff_distance"],
                #     input_line=input_geom,
                #     truth_line=truth_geom
                # ))
            else:
                # 如果不匹配，可以根据需要添加未匹配的记录
                pass
        results = list(results_dict.values())
        return results

    def calculate_metrics(self, results: List[MatchResult]):
        """
        计算评测指标：准确率、精确率和召回率。

        :param results: List[MatchResult]
        :return: 准确率、精确率和召回率。

        Args:
            results:
        """
        # TP: 每个输入停止线都有对应的真值线
        true_positives = len(results)
        # FN: 每个真值停止线没有匹配到输入线
        false_negatives = self.truth_stopline_gdf.shape[0] - len(results)
        # FP: 每条输入线应该匹配真值线，但实际上没有匹配上
        false_positives = self.input_stopline_gdf.shape[0] - len(results)

        recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
        precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0
        accuracy = true_positives / len(self.input_stopline_gdf) if len(self.input_stopline_gdf) > 0 else 0

        return accuracy, precision, recall

    def evaluate(self) -> StopLineEvaluateResult:
        """
        评测方法，用于执行停止线匹配。
        """
        title = self.input_intersection.result_save_title_prefix
        match_results = self.match_stoplines_v2()  # 执行匹配
        res = self.generate_evaluate_result(title=title, results=match_results)
        return res

    def generate_evaluate_result(self, title, results: List[MatchResult]) -> StopLineEvaluateResult:
        """
        Args:
            title:
            results:

        Returns:

        """
        accuracy, precision, recall = self.calculate_metrics(results)  # 计算指标

        angles = [min(item.angle_diff_deg, 180 - item.angle_diff_deg) for item in results]
        hausdorffs = [item.hausdorff_distance for item in results]
        center_distances = [item.centroid_distance for item in results]
        length_diffs = [item.length_diff for item in results]

        res = StopLineEvaluateResult(
            title=title,
            accuracy=accuracy,
            precision=precision,
            recall=recall,

            max_hausdorff=np.max(hausdorffs) if hausdorffs else np.inf,
            min_hausdorff=np.min(hausdorffs) if hausdorffs else np.inf,

            max_rotate_angle=np.max(angles) if angles else np.inf,
            min_rotate_angle=np.min(angles) if angles else np.inf,

            max_len_diff=np.max(length_diffs) if length_diffs else np.inf,
            min_len_diff=np.min(length_diffs) if length_diffs else np.inf,

            match_result=results
        )
        stats_text = res.get_dict()
        stats_text_formatted = "\n".join([f"{key}: {value}" for key, value in stats_text.items()])
        logger.info(f"\n评测任务:{title} 结论:\n{stats_text_formatted}")  # 记录评测结论
        return res

    def save_svg(self, evaluate_result: StopLineEvaluateResult, vector_file_path: str, title: str = None,
                 is_visualize=False):

        # 获取匹配与未匹配的线要素
        matched_input_gdf = self.input_stopline_gdf.iloc[[item.input_index for item in evaluate_result.match_result]]
        unmatched_input_gdf = self.input_stopline_gdf.drop(
            [item.input_index for item in evaluate_result.match_result])  # drop方法默认返回一个新的gdf，而不会修改原始数据。
        matched_truth_gdf = self.truth_stopline_gdf.iloc[[item.truth_index for item in evaluate_result.match_result]]
        unmatched_truth_gdf = self.truth_stopline_gdf.drop([item.truth_index for item in evaluate_result.match_result])

        # 设置样式
        layer_unmatched_input = LineConfig(
            **{"name": "unmatched_input", "color": 'red', "zorder": 2, "gdf": unmatched_input_gdf}
        )
        layer_matched_input = LineConfig(
            **{"name": "matched_input", "color": 'green', "zorder": 2, "gdf": matched_input_gdf}
        )
        layer_unmatched_truth = LineConfig(
            **{"name": "unmatched_truth", "color": 'purple', "zorder": 2, "gdf": unmatched_truth_gdf}
        )
        layer_matched_truth = LineConfig(
            **{"name": "matched_truth", "color": 'purple', "zorder": 2, "gdf": matched_truth_gdf}
        )

        GeoEvaPlotter(interactive_show=is_visualize).plot(
            title=f"{evaluate_result.title}_停止线（搜索范围{self.tool_config.stopline_match_distance_meter}米）",
            main_layers=[layer_unmatched_input, layer_matched_input, layer_unmatched_truth, layer_matched_truth],
            ref_layers=[self.truth_intersection.lane_lines, self.truth_intersection.crosswalks,
                        self.truth_intersection],
            text="\n".join([f"{key}: {value}" for key, value in evaluate_result.get_dict().items()]),
            output_svg_path=vector_file_path,
        )

    def save_as_gkpg(self, results: StopLineEvaluateResult, gpkg_path):
        """
        将一StopLineEvaluateResult保存为gpkg文件。

        :param results: MatchResult实例列表
        :param gpkg_path: 保存gpkg文件的路径
        """
        # 过滤掉几何为空的结果，方便构造 GeoDataFrame

        data_dicts = [result.to_dict() for result in results.match_result]
        if not data_dicts:
            raise ValueError("没有有效的几何数据可以保存。")

        GeoEvaPlotter().save_gpkg(
            layers_raw=[
                LineConfig(name="真值停止线", gdf=self.truth_intersection.stop_lines),
                LineConfig(name="输入停止线", gdf=self.input_intersection.stop_lines),
                LineConfig(name="真值停止线", gdf=self.truth_intersection.stop_lines),
                LineConfig(name="输入停止线", gdf=self.input_intersection.stop_lines),
                LineConfig(name="匹配关系连接线",
                           gdf=GeoDataFrame(data_dicts, geometry='geometry', crs=self.standard_utm_crs)),

            ], output_gpkg_path=gpkg_path
        )
