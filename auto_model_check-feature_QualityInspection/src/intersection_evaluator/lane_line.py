# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 15:10
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : lane_line
import math
import os
import time
from typing import Optional, Dict, Set

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from geopandas import GeoDataFrame
from pydantic import BaseModel, Field
from shapely import LineString, MultiLineString
from shapely.geometry import CAP_STYLE

from src.errors import DataEmpty
from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluatorParams
from src.scripts.obj2pickles import load_this

# TODO 收到配置文件里面
# 0506定义的公共变量
min_overlap_ratio = 0.3
max_angle_diff_degree = 5.0


class LineMatchResult(BaseModel):
    """
    评测结果类，用于存储线匹配评测的各项指标。

    Attributes:
        title (str): 标题，默认为空。
        algo_id (str): 算法标识。
        data_version (str): 数据版本信息。
        nickname (Optional[str]): 昵称，可选项。
        resample_segment_length (float): 重采样切分的线段长度（单位：米）。
        buffer_distance (float): 缓冲区距离（单位：米）。
        precision (float): 精确率，计算公式为 TP/(TP+FP)。
        recall (float): 召回率。
        hit_length (float): 累计命中长度（单位：米）。
        extra_length (float): 累计冗余长度（单位：米）。
        mis_match_length (float): 累计漏识别长度（单位：米）。
        truth_segments_gdf (GeoDataFrame): 真值线段的 GeoDataFrame。
        input_lines_gdf (GeoDataFrame): 输入线段的 GeoDataFrame。
        total_truth_length (float): 真值线段总长度（单位：米）。
        total_input_length (float): 输入线段总长度（单位：米）。
    """

    title: str = None
    algo_id: str
    data_version: str
    nickname: Optional[str] = None

    resample_segment_length: float
    buffer_distance: float

    precision: float = Field(title="准确率", description="准确率 Precision = TP/(TP+FP)")
    recall: float = Field(title="召回率")
    hit_truth_length: float = Field(title="累计真值命中长度", description="累计真值命中长度")
    hit_input_length: float = Field(title="累计输入命中长度", description="累计输入命中长度")
    extra_length: float = Field(title="累计输入冗余长度", description="累计冗余累计输入冗余长度长度")
    mis_match_length: float = Field(title="累计真值漏识别长度", description="累计真值漏识别长度")
    truth_segments_gdf: GeoDataFrame  # 存储真值线段
    input_lines_gdf: GeoDataFrame  # 存储输入线段列表

    total_truth_length: float  # 真值线总长度
    total_input_length: float  # 输入线总长度

    @property
    def f1_score(self) -> float:
        """
        计算 F1 分数，综合考虑精确率和召回率。

        Returns:
            float: 计算得到的 F1 分数。如果精确率和召回率之和为 0，则返回 0。
        """
        return 2 * (self.precision * self.recall) / (self.precision + self.recall) if (
                                                                                              self.precision + self.recall) > 0 else 0

    class Config:
        # 允许任意类型的字段
        arbitrary_types_allowed = True

    def get_dict(self) -> Dict:
        """
        获取评测结果的文本描述信息。

        Returns:
            Dict: 包含评测指标及相关信息的字典，主要字段包括算法信息、真值及输入线总长度、准确率、召回率、F1 分数、命中长度、冗余误报长度与漏识别长度。
        """
        stats_dict = {
            "真值线两侧缓冲区(米)": f"{self.buffer_distance:.2f}",
            "真值累计长度(米)": f"{self.total_truth_length:.2f}",
            "输入累计长度(米)": f"{self.total_input_length:.2f}",
            "准确率": f"{self.precision * 100:.2f}%",
            "召回率": f"{self.recall * 100:.2f}%",
            "F1分数": f"{self.f1_score:.2f}",
            "累计真值命中长度(米)": f"{self.hit_truth_length:.2f}",
            "累计输入命中长度(米)": f"{self.hit_input_length:.2f}",
            "累计输入冗余长度(米)": f"{self.extra_length:.2f}",
            "累计真值漏识别长度(米)": f"{self.mis_match_length:.2f}"
        }
        return stats_dict


class LaneLineEvaluator:
    """
    车道线评测类，用于评测道路线的真值数据与输入数据。

    Attributes:
        truth_intersection (Intersection): 真值路口对象，其包含真值线段等数据。
        input_intersection (Intersection): 待评测的输入路口对象。
        evaluator_setting (EvaluatorParams): 评测工具的配置信息，包括阈值、分段长度、缓冲距离等。
        truth_lines_gdf (GeoDataFrame): 真值线段数据，来源于 truth_intersection。
        input_lines_gdf (GeoDataFrame): 输入线段数据，来源于 input_intersection。
        segment_length (float): 分段长度，取自 evaluator_setting。
    """

    def __init__(self,
                 truth_intersection: Intersection,
                 input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams  # 阈值等设置，直接扔进来
                 ):
        """
        构造 LaneLineEvaluator 实例。

        Args:
            truth_intersection (Intersection): 真值路口对象。
            input_intersection (Intersection): 待评测的输入路口对象。
            evaluator_setting (EvaluatorParams): 评测工具配置参数。
        """
        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.evaluator_setting = evaluator_setting

        self.truth_lines_gdf = self.truth_intersection.lane_lines
        self.input_lines_gdf = self.input_intersection.lane_lines

        self.segment_length = evaluator_setting.resample_segment_length_meter
        self.truth_lines_buffer_dist = self.evaluator_setting.line_buffer_length_meter

        if self.truth_lines_gdf is None:
            raise DataEmpty("无真值数据")
        if self.input_lines_gdf is None:
            raise DataEmpty("无待评测数据")

    def set_truth_lines_buffer_dist(self, new_buffer_dist: float):
        logger.debug(f"线要素评测器正在修改自身buffer：{self.truth_lines_buffer_dist}->{new_buffer_dist}")
        self.truth_lines_buffer_dist = new_buffer_dist

    @staticmethod
    def split_line(line: LineString, segment_length: float):
        """
        按指定长度将 LineString 切分为多个子段，并保留原始形状。

        Args:
            line (LineString): 待切分的 LineString 对象。
            segment_length (float): 切分子段的目标长度（单位：米）。

        Returns:
            List[LineString]: 切分得到的多个 LineString 子段列表，保留原始曲线形状。

        Raises:
            ValueError: 当输入几何无效或 segment_length 非正数时抛出异常。
            ImportError: 如果 shapely 版本低于 1.8 或无法导入 substring。
        """
        # 检查输入
        if not isinstance(line, LineString) or line.is_empty:
            raise ValueError(f"Invalid input geometry: {line}. Expected a non-empty LineString.")
        if segment_length <= 0:
            raise ValueError("Segment length must be a positive number.")

        try:
            # 尝试导入 substring，如果失败说明版本太低
            from shapely.ops import substring
        except ImportError:
            raise ImportError("shapely.ops.substring is required. Please upgrade shapely to version 1.8 or higher.")

        total_length = line.length
        if segment_length >= total_length:
            return [line]  # 如果目标长度大于总长，直接返回原线

        segmented_lines = []
        start_dist = 0.0
        while start_dist < total_length:
            end_dist = min(start_dist + segment_length, total_length)
            # 使用 substring 提取子串
            # normalized=False 表示距离是绝对距离，不是比例
            # 增加一个微小的容差处理浮点精度可能导致无法提取最后一点点的问题
            if abs(start_dist - end_dist) > 1e-9:  # 避免创建零长度段
                segment = substring(line, start_dist, end_dist, normalized=False)
                # 有时浮点问题可能导致返回空几何，需要检查
                if not segment.is_empty:
                    segmented_lines.append(segment)
            start_dist = end_dist  # 更新下一段的起点

            # 防止无限循环 (虽然理论上不应发生)
            if start_dist >= total_length:
                break

        return segmented_lines

    def prepare_truth_segments(self, segment_length: Optional[float] = None) -> GeoDataFrame:
        """
        切分真值线并生成包含额外匹配统计字段的 GeoDataFrame。

        每个切分后的子段增加字段：
          - is_matched: 是否匹配。
          - matched_input_line_id: 匹配的输入线标识。
          - offset: 真值段上匹配中点到原线段的距离偏移。

        Args:
            segment_length (Optional[float]): 分段长度；若为 None 则使用 self.segment_length。

        Returns:
            GeoDataFrame: 包含切分后的真值子段的 GeoDataFrame，geometry 字段为切分后的子段。
        """
        if segment_length is None:
            segment_length = self.segment_length

        segments = []  # 存储切分后的真值子段
        for _, row in self.truth_lines_gdf.iterrows():
            original_geometry = row['geometry']  # 获取真值线的几何
            try:
                for segment in self.split_line(original_geometry, segment_length):
                    new_row = row.copy()  # 继承原始属性
                    new_row['geometry'] = segment  # 替换几何为当前子段
                    # 增加匹配统计的初始字段
                    new_row['is_matched'] = False
                    new_row['matched_input_line_id'] = None
                    new_row['offset'] = np.nan
                    new_row['input_length'] = np.nan
                    new_row['angle_diff'] = np.nan
                    new_row['overlap_ratio'] = np.nan
                    segments.append(new_row)
            except Exception as e:
                logger.warning(f"切分线段遇到问题：{e}. 已经跳过处理")
        # 返回新的 GeoDataFrame，包含切分后的子段
        return GeoDataFrame(segments, geometry='geometry', crs=self.truth_lines_gdf.crs).reset_index(drop=True)

    def compute_coverage(self, segment_length: Optional[float] = None, buffer_distance: Optional[float] = None,
                         strong_match=True):
        """
        计算输入线与真值段之间的匹配覆盖情况。

        核心流程说明：
            1. 调用 prepare_truth_segments() 获取切分后的真值段 GeoDataFrame。
            2. 构建真值段的空间索引，并遍历每条输入线：
               - 利用空间索引查询候选真值段；
               - 对候选每段生成缓冲区（允许一定偏差），若输入线与缓冲区相交，则计算交集长度并累加；
                    a. 计算交集。
                    b. （如果strong_match值为True）进行 strong_match 检查。
                    c. 计算偏移 offset。
                    d. 竞争更新: 如果当前匹配的offset优于该真值段已记录的 offset，
                       则更新truth_segments_gdf中该行的'is_matched', 'matched_input_line_id', 'offset',
                       以及 'input_length' (记录导致此最小 offset 的交集段长度)。
            3. 通过候选匹配统计计算出精确率和召回率等指标。

        Args:
            segment_length: 切分真值线的分段长度，默认为 None 则使用 self.segment_length。
            buffer_distance: 缓冲距离，默认为 None 时使用 evaluator_setting 中配置的值。
            strong_match: 是否使用增强校验。
        Returns:
            LineMatchResult: 包含以下字段的评测结果对象：
                - total_input_length: 输入线总长度。
                - hit_length: 累计匹配（交集）长度。
                - false_positive_length: 未匹配部分的累计长度。
                - matched_truth_indices: 匹配到的真值段索引集合。
                以及精确率、召回率、真值、输入等其他统计信息。
        """
        start_time = time.time()
        epsilon = 1e-9
        # min_overlap_ratio = 0.3
        # max_angle_diff_deg = 5.0

        # 处理默认输入
        if buffer_distance is None:
            buffer_distance = self.evaluator_setting.line_buffer_length_meter
        if segment_length is None:
            segment_length = self.segment_length

        # 准备真值段 =================================================================
        truth_segments_gdf = self.prepare_truth_segments(segment_length)
        if truth_segments_gdf.empty:
            raise ValueError("真值Segment GDF为空.")

        # 准备真值段Buffer ===========================================================
        # 由真值线段生成一个新的，由他们的buffer组成的gdf，保持index与原gdf一致。直接用空间换时间。
        # 注意这里buffer的样式要是flat
        logger.debug("计算buffer中....")
        buffered_truth_gdf = truth_segments_gdf.copy()
        buffered_truth_gdf['geometry'] = buffered_truth_gdf['geometry'].buffer(buffer_distance,
                                                                               cap_style=CAP_STYLE.flat)  # buffer操作

        # 初始化统计值 ===========================================================
        hit_length = 0.0  # # 全局累加：输入线中与真值段成功匹配部分的总长度
        false_positive_length = 0.0  # 全局累加：输入线中未能匹配任何真值段部分的总长度
        matched_truth_indices: Set[int] = set()  # 存储所有被至少一个输入线段成功匹配的真值段的索引

        # 主循环：遍历每一条输入线 ================================================
        for input_idx, input_row in self.input_lines_gdf.iterrows():
            input_line = input_row['geometry']
            if input_line is None or input_line.is_empty:  # 忽略空或无效的几何
                continue

            # 用真值Buffer去与输入线做相交 ===========================================================
            matching_buffers = buffered_truth_gdf[buffered_truth_gdf["geometry"].intersects(input_line)]
            if matching_buffers.empty:  # 尽早退出
                # 如果不与任何buffer相交，整条线计入FP(没有命中)
                false_positive_length += input_line.length
                continue  # 然后今早退出直接处理下一条输入线

            # 下面开始处理相交部分 ===========================================================
            matching_indices = matching_buffers.index.tolist()
            candidates_gdf = truth_segments_gdf.loc[matching_indices]  # .loc 确保按索引精确查找，因为索引大概率不连续

            # # 初始化当前输入线的强匹配命中长度累加器
            # strong_matched_length_for_this_input = 0.0

            # 内循环：遍历每个候选真值段 ==========================================================
            for candidate_idx, candidate in candidates_gdf.iterrows():
                # 获取几何对象
                truth_segment_geom = candidate['geometry']
                if truth_segment_geom is None or truth_segment_geom.is_empty: continue

                buffer_geom = matching_buffers.loc[candidate_idx, 'geometry']  # 从matching_buffers中获取对应的缓冲区几何
                # 计算输入线与缓冲区的交集
                intersection = input_line.intersection(buffer_geom)
                if intersection.is_empty: continue  # 跳过空交集

                calculated_offset = np.nan
                best_intersection_segment_length = 0.0  # 用于记录导致最佳匹配的交集长度
                segment_length_for_update = 0.0  # 存储该交集段的长度

                # 处理落在真值buffer段中的输入部分，交集是LineString ===============================
                if intersection.geom_type == 'LineString':
                    if intersection.length >= epsilon:

                        is_considered_match = True
                        if strong_match:
                            # 调用强匹配检查函数，比较交集线段和原始真值段
                            # 加强判断:
                            # 1. 如果投影长度大于真值线段长度的30%才计算命中，
                            # 2. 矢量同向，或者大致同向（5度）才算match
                            # 3. Multilinestring也会如此判断
                            # is_considered_match = self.strong_match(intersection, truth_segment_geom)

                            overlap_ratio = intersection.length / truth_segment_geom.length
                            angle_diff = self.angle_difference_between_lines(intersection, truth_segment_geom)

                            if np.isnan(truth_segments_gdf.at[candidate_idx, 'angle_diff']) or (
                                    angle_diff < truth_segments_gdf.at[candidate_idx, 'angle_diff']):
                                truth_segments_gdf.at[candidate_idx, 'angle_diff'] = angle_diff
                                truth_segments_gdf.at[candidate_idx, 'overlap_ratio'] = overlap_ratio

                            if angle_diff <= max_angle_diff_degree and overlap_ratio >= min_overlap_ratio:
                                is_considered_match = True
                            else:
                                is_considered_match = False

                        if is_considered_match:
                            matched_truth_indices.add(candidate_idx)
                        else:
                            continue

                        mid_point = intersection.interpolate(0.5, normalized=True)
                        projected_distance = truth_segment_geom.project(mid_point)
                        projected_point = truth_segment_geom.interpolate(projected_distance)
                        calculated_offset = mid_point.distance(projected_point)

                        if calculated_offset < epsilon: calculated_offset = 0.0  # 应用阈值
                    else:
                        # 如果出现了很短的intersection线，那么直接继续下一次循环
                        continue
                    segment_length_for_update = intersection.length

                elif intersection.geom_type == 'MultiLineString':
                    min_offset_for_this_multi = np.inf
                    best_single_segment = None  # 存储产生最小 offset 的那个 segment
                    processed_valid_match = False  # 标记是否至少有一个 segment 匹配成功

                    for single_segment in intersection.geoms:
                        if single_segment.geom_type == 'LineString' and not single_segment.is_empty and single_segment.length >= epsilon:

                            is_considered_match = True
                            if strong_match:
                                # is_considered_match = self.strong_match(single_segment, truth_segment_geom)
                                overlap_ratio = single_segment.length / truth_segment_geom.length
                                angle_diff = self.angle_difference_between_lines(single_segment, truth_segment_geom)

                                if (truth_segments_gdf.at[candidate_idx, 'angle_diff'] is np.nan or angle_diff >
                                        truth_segments_gdf.at[candidate_idx, 'angle_diff']):
                                    truth_segments_gdf.at[candidate_idx, 'angle_diff'] = angle_diff
                                    truth_segments_gdf.at[candidate_idx, 'overlap_ratio'] = overlap_ratio

                                if angle_diff <= max_angle_diff_degree and overlap_ratio >= min_overlap_ratio:
                                    is_considered_match = True
                                else:
                                    is_considered_match = False

                            if is_considered_match:
                                processed_valid_match = True  # 标记找到至少一个匹配

                                mid_point = single_segment.interpolate(0.5, normalized=True)
                                projected_distance = truth_segment_geom.project(mid_point)
                                projected_point = truth_segment_geom.interpolate(projected_distance)
                                offset_single = mid_point.distance(projected_point)
                                # 如果当前偏移更小，更新最小偏移和对应的 segment
                                if offset_single < min_offset_for_this_multi:
                                    min_offset_for_this_multi = offset_single
                                    best_single_segment = single_segment

                    # 只有MultiLineString中至少一个匹配成功，
                    if processed_valid_match:
                        # matched_truth_indices.add(candidate_idx)
                        if calculated_offset < epsilon: calculated_offset = 0.0

                        # 下面用两个方式计算命中长度：
                        # # 1. 这个分支是累加所有强匹配的 segment 长度 (更符合 TP 定义)
                        # current_multi_hit_length = 0.0
                        # for seg in intersection.geoms:
                        #     if seg.geom_type == 'LineString' and self.strong_match(seg, truth_segment_geom,
                        #                                                            min_overlap_ratio,
                        #                                                            max_angle_diff_deg, epsilon):
                        #         current_multi_hit_length += seg.length
                        # strong_matched_length_for_this_input += current_multi_hit_length

                        # 2. 这个分支是只累加最佳 segment 的长度 (不太标准，但是对于作业准入来说更有参考价值)
                        if best_single_segment:
                            segment_length_for_update = best_single_segment.length

                elif not intersection.is_empty:
                    logger.warning(f"交集非Linestring，跳过 {candidate_idx}: {intersection.geom_type}")
                    pass

                # TODO 这里抽出一个函数来
                # --- 更新真值段 GDF 的匹配信息 (基于 offset 竞争) ---
                if not pd.isna(calculated_offset):
                    current_offset = truth_segments_gdf.at[candidate_idx, 'offset']
                    # 只有当新 offset 更好时，才更新该真值段的匹配信息
                    if pd.isna(current_offset) or calculated_offset < current_offset:
                        logger.trace(
                            f"更新 Truth:{candidate_idx} 的匹配信息: Offset {calculated_offset:.4f}, Input:{input_idx}, InputLength:{segment_length_for_update:.2f}")
                        truth_segments_gdf.at[candidate_idx, 'is_matched'] = True
                        truth_segments_gdf.at[candidate_idx, 'matched_input_line_id'] = input_idx
                        truth_segments_gdf.at[candidate_idx, 'offset'] = calculated_offset
                        # **核心修改**: 'input_length' 记录的是导致这个最佳匹配的交集段的长度
                        truth_segments_gdf.at[candidate_idx, 'input_length'] = segment_length_for_update
                        # 记录下这个真值段被匹配了 (用于 FN 计算)
                        matched_truth_indices.add(candidate_idx)
                    else:  # 如果 offset 不是更好，则不更新，之前的匹配保持不变
                        logger.trace(
                            f"真值段:{candidate_idx} 已有更好匹配 (Offset {current_offset:.4f})，忽略 Input:{input_idx} (Offset {calculated_offset:.4f}) 的更新")

        logger.info("主匹配函数完成.")

        # TODO 以下部分也抽出一个函数来
        # 计算最终指标 ==============================================
        logger.debug("正在计算统计指标...")
        total_truth_length = self.truth_lines_gdf["geometry"].length.sum()
        total_input_length = self.input_lines_gdf["geometry"].length.sum()

        # 计算TP，也就是命中长度 =====================================
        # TP = 成功匹配的真值段记录的具体'input_length'之和
        matched_segments = truth_segments_gdf[truth_segments_gdf['is_matched'] == True]
        hit_length = matched_segments['input_length'].sum()

        # 计算 FP (Extra Length) ,代表了输入线中没有最终以最佳匹配规则匹配上任何真值段匹配的部分长度。
        # FP = 输入总长度 - TP (基于 TP 的新定义)
        false_positive_length = max(total_input_length - hit_length, 0.0)

        # 计算FN，未被匹配的真值段的总长度 =====================================
        unmatched_segments = truth_segments_gdf[truth_segments_gdf['is_matched'] == False]
        false_negative_length = unmatched_segments['geometry'].length.sum()  # 更直接

        # 准确率计算和召回率计算
        denominator_precision = hit_length + false_positive_length
        precision = hit_length / denominator_precision if denominator_precision > epsilon else 0.0

        # Recall = TP / (TP + FN)
        denominator_recall = hit_length + false_negative_length
        recall = hit_length / denominator_recall if denominator_recall > epsilon else 0.0

        logger.debug(
            f"最终指标: Hit={hit_length:.2f}, FP={false_positive_length:.2f}, FN={false_negative_length:.2f}, Precision={precision:.4f}, Recall={recall:.4f}")

        # --- 打包结果 ---
        logger.debug("Packaging results...")
        evaluation_results = LineMatchResult(**{
            "algo_id": self.input_intersection.name,
            "data_version": self.input_intersection.version,
            "nickname": self.input_intersection.nickname,
            "resample_segment_length": segment_length,  # 使用函数内确定的值
            "buffer_distance": buffer_distance,  # 使用函数内确定的值
            "hit_truth_length": hit_length,
            "hit_input_length": hit_length,   # 0506更新，目前这个值应该是等于hit_truth_length，看看这个指标要不要改一下。
            "extra_length": false_positive_length,  # 冗余长度（未命中真值的输入值长度）
            "mis_match_length": false_negative_length,  # 漏报长度（未被命中的真值长度）
            "precision": precision,  # 精确率
            "recall": recall,  # 召回率
            "total_truth_length": total_truth_length,  # 真值线总长度
            "total_input_length": total_input_length,  # 输入线总长度
            # 返回 GDF 副本 ---
            "truth_segments_gdf": truth_segments_gdf,
            "input_lines_gdf": self.input_lines_gdf,
        })

        logger.info(f"指标总结如下: {evaluation_results.get_dict()}")

        end_time = time.time()
        logger.info(f"处理耗时共{end_time - start_time:.2f}秒")

        return evaluation_results

    @staticmethod
    def angle_difference_between_lines(line1: LineString, line2: LineString, epsilon=1e-9) -> float:
        """
        计算两个LineString的方向角差（0°~180°）

        参数：
        - line1, line2: shapely LineString对象，代表两条线段
        - epsilon: 浮点数容差，小于此长度视为无效线段

        返回：
        - 两线方向夹角，单位度，范围[0, 180]
          如果任意一个线段无效（长度太短或单点），函数返回None

        说明：
        - 方向向量由线段起点指向终点
        - 方向角差是两向量夹角的最小角度（不考虑方向），即0~180度
        """

        def get_vector(line: LineString):
            if not line or line.is_empty or len(line.coords) < 2:
                return None
            start = np.array(line.coords[0], dtype=float)
            end = np.array(line.coords[-1], dtype=float)
            vec = end - start
            if np.linalg.norm(vec) < epsilon:
                return None
            return vec

        def calculate_angle_between_vectors(v1, v2):
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            if norm1 < epsilon or norm2 < epsilon:
                return None
            dot_prod = np.dot(v1, v2)
            # 夹角余弦，限制在[-1,1]
            cos_theta = np.clip(dot_prod / (norm1 * norm2), -1.0, 1.0)
            angle_rad = np.arccos(cos_theta)
            angle_deg = math.degrees(angle_rad)
            # 夹角总是0~180度，已保证
            return angle_deg

        v1 = get_vector(line1)
        v2 = get_vector(line2)

        if v1 is None or v2 is None:
            # 无法定义方向向量，无法计算角度差
            raise ValueError("计算角度错误")

        angle_diff = calculate_angle_between_vectors(v1, v2)
        return angle_diff

    @staticmethod
    def get_line_vector(line: LineString):
        """获取 LineString 的起始点到结束点的向量"""
        coords = list(line.coords)
        if len(coords) < 2: return np.array([0.0, 0.0])
        start = coords[0];
        end = coords[-1]
        return np.array([end[0] - start[0], end[1] - start[1]])

    @staticmethod
    def strong_match(intersection: (LineString, MultiLineString),
                     truth_segment_geom: LineString,
                     min_overlap_ratio: float = 0.3,
                     max_angle_diff_deg: float = 5.0,
                     epsilon: float = 1e-9) -> bool:
        """
            检查交集是否满足严格匹配条件：最小重叠和方向一致性。

            Args:
                intersection: 输入线与真值缓冲区的交集几何。
                truth_segment_geom: 对应的原始真值线段几何。
                min_overlap_ratio: 交集长度与真值段长度的最小比例阈值。
                max_angle_diff_deg: 交集方向与真值段方向的最大允许夹角（度）。
                epsilon: 零值判断的容差。

            Returns:
                bool: 如果满足所有条件则返回 True，否则返回 False。
        """
        # --- 基础检查 ---
        if intersection is None or intersection.is_empty or \
                truth_segment_geom is None or truth_segment_geom.is_empty:
            return False

        intersection_length = intersection.length
        truth_segment_length = truth_segment_geom.length

        # --- 条件 0: 长度检查 ---
        if intersection_length < epsilon or truth_segment_length < epsilon:
            return False  # 避免后续除零或无意义比较

        # --- 条件 1: 最小重叠长度检查 ---
        overlap_ratio = intersection_length / truth_segment_length
        # 添加一个小的容差来处理 intersection 可能比 truth 短一点点的情况
        if overlap_ratio < (min_overlap_ratio - epsilon):
            return False

        # --- 条件 2: 方向一致性检查 (使用余弦) ---
        truth_vector = LaneLineEvaluator.get_line_vector(truth_segment_geom)
        intersection_vector = np.array([0.0, 0.0])

        if intersection.geom_type == 'LineString':
            intersection_vector = LaneLineEvaluator.get_line_vector(intersection)
        elif intersection.geom_type == 'MultiLineString':
            # 使用最长段作为代表方向
            max_len = -1.0
            longest_segment = None
            for seg in intersection.geoms:
                if seg.geom_type == 'LineString' and not seg.is_empty:
                    if seg.length > max_len: max_len = seg.length; longest_segment = seg
            if longest_segment:
                intersection_vector = LaneLineEvaluator.get_line_vector(longest_segment)
            else:
                return False  # MultiLineString 中没有有效的 LineString 段

        # 检查向量长度
        norm_truth = np.linalg.norm(truth_vector)
        norm_intersection = np.linalg.norm(intersection_vector)

        if norm_truth < epsilon or norm_intersection < epsilon:
            # 如果向量长度接近零，认为方向匹配（或无法判断，按匹配处理）
            return True

        # 计算余弦值
        dot_product = np.dot(truth_vector, intersection_vector)
        norm_prod = norm_truth * norm_intersection
        # 这里 norm_prod 理论上不应为零，因为上面检查了 norm > epsilon
        cos_theta = dot_product / norm_prod

        # 计算最大允许角度对应的余弦阈值
        # 角度越小，余弦值越接近 1
        # 角度差在 [0, max_angle_diff_deg] 之内意味着 cos_theta >= cos(max_angle_diff_deg)
        cos_max_angle = math.cos(math.radians(max_angle_diff_deg))

        # 检查余弦值是否满足条件
        if cos_theta < (cos_max_angle - epsilon):  # 添加 epsilon 容差
            # 为了调试，可以计算实际角度
            # actual_angle = math.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
            # logger.trace(f"[{debug_id}] --> Actual angle: {actual_angle:.4f} deg > {max_angle_diff_deg} deg.")
            return False

        # --- 所有条件都满足 ---
        return True

    def save_as_gkpg(self, results: LineMatchResult, gpkg_path: str, csv_output=True, is_to_4326=True):
        """
        将评测结果保存为GeoPackage文件，并可选同时输出CSV格式的结果。

        Args:
            results (LineMatchResult): 评测结果对象。
            gpkg_path (str): 输出 GeoPackage 文件路径。
            csv_output (bool): 是否同时导出 CSV 文件，默认为 True。
            is_to_4326 (bool): 是否将数据转换为 EPSG:4326 坐标系，默认为 True。
        """
        # 确保输出目录存在
        output_dir = os.path.dirname(gpkg_path)
        os.makedirs(output_dir, exist_ok=True)

        # 根据需要转换CRS并防止属性为None，也就是元素数据中没有这一类要素的文件导致最后处理出错
        if is_to_4326:
            stop_lines = self.truth_intersection.stop_lines.to_crs(
                epsg=4326) if self.truth_intersection.stop_lines is not None else None
            crosswalks = self.truth_intersection.crosswalks.to_crs(
                epsg=4326) if self.truth_intersection.crosswalks is not None else None
            input_lines = self.input_lines_gdf.to_crs(epsg=4326) if self.input_lines_gdf is not None else None
            truth_segments = results.truth_segments_gdf.to_crs(
                epsg=4326) if results.truth_segments_gdf is not None else None
            arrows = self.truth_intersection.arrows.to_crs(
                epsg=4326) if self.truth_intersection.arrows is not None else None
            algo_box = self.truth_intersection.algo_box.to_crs(
                epsg=4326) if self.truth_intersection.algo_box is not None else None
        else:
            stop_lines = self.truth_intersection.stop_lines
            crosswalks = self.truth_intersection.crosswalks
            input_lines = self.input_lines_gdf
            truth_segments = results.truth_segments_gdf
            arrows = self.truth_intersection.arrows
            algo_box = self.truth_intersection.algo_box

        # 构造图层名称与对象的映射字典
        layers = {
            "truth_buffer": truth_segments,
            "input_lines": input_lines,
            "crosswalks": crosswalks,
            "stop_lines": stop_lines,
            "arrows": arrows,
            # "algo_box": algo_box,
        }

        # 过滤掉值为None的图层，避免to_file报错
        layers = {layer_name: gdf for layer_name, gdf in layers.items() if gdf is not None}

        # 逐一将GeoDataFrame写入同一个gpkg中
        for layer_name, gdf in layers.items():
            try:
                logger.info(f"写入图层：{layer_name}")
                gdf.to_file(gpkg_path, layer=layer_name, driver='GPKG')
            except Exception as e:
                logger.error(f"写入图层{layer_name}失败: {e}")

        # 偷个懒
        if csv_output:
            if truth_segments is not None:
                csv_path = gpkg_path.replace(".gpkg", ".csv")
                try:
                    truth_segments.to_csv(csv_path, index=False)
                    logger.info("CSV 文件写入成功")
                except Exception as e:
                    logger.error(f"CSV 文件写入失败: {e}")
            else:
                logger.error("没有匹配的结果，无法输出CSV统计文件！")

    def plot(self, title: str, results: LineMatchResult,
             is_to_4326: bool = True,
             is_visualize: bool = False,  # 调试时可选参数
             vector_file_path: Optional[str] = None):
        """
        绘制评测结果图形。

        Args:
            title: 标题。
            results: 评测结果对象。
            is_to_4326: 是否将数据转换为 EPSG:4326 坐标系，默认为 True。
            is_visualize: 是否显示可视化窗口，默认为 False。
            vector_file_path: 如果提供，则保存图形为矢量文件（SVG）。
        """
        fig, ax = plt.subplots(figsize=(24, 18))  # 创建绘图窗口

        # -------------------------------
        # 1. 坐标系转换
        # -------------------------------
        if is_to_4326:
            input_lines = self.input_lines_gdf.to_crs(epsg=4326)
            truth_segments = results.truth_segments_gdf.to_crs(epsg=4326)
            algo_box = self.truth_intersection.algo_box.to_crs(epsg=4326)
            try:
                stop_lines = self.truth_intersection.stop_lines.to_crs(epsg=4326)
                crosswalks = self.truth_intersection.crosswalks.to_crs(epsg=4326)
                arrows = self.truth_intersection.arrows.to_crs(epsg=4326)
            except:
                pass

        else:
            input_lines = self.input_lines_gdf
            truth_segments = results.truth_segments_gdf
            algo_box = self.truth_intersection.algo_box
            try:
                stop_lines = self.truth_intersection.stop_lines
                crosswalks = self.truth_intersection.crosswalks
                arrows = self.truth_intersection.arrows
            except:
                pass

        # -------------------------------
        # 2. 绘制参考要素
        # -------------------------------
        # algo_box.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
        try:
            stop_lines.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
            crosswalks.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
            arrows.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
        except:
            pass

        # -------------------------------
        # 3. 绘制输入线
        # -------------------------------
        input_lines.plot(ax=ax, color="gray", alpha=0.5)

        # -------------------------------
        # 4. 绘制真值线段（区分匹配和未匹配）
        # -------------------------------
        gdf_matched = truth_segments[truth_segments['is_matched']]
        gdf_not_matched = truth_segments[~truth_segments['is_matched']]

        if gdf_matched.shape[0] != 0:
            gdf_matched.plot(ax=ax, color='green', linewidth=2, linestyle='-')
        if gdf_not_matched.shape[0] != 0:
            gdf_not_matched.plot(ax=ax, color='red', linewidth=1, linestyle='--')

        # -------------------------------
        # 5. 添加图例与标题
        # -------------------------------
        from matplotlib.lines import Line2D  # 局部导入
        handles = [
            Line2D([0], [0], color='green', lw=2, label='命中的真值线片段'),
            Line2D([0], [0], color='red', lw=2, linestyle='--', label='未命中的真值线片段'),
            Line2D([0], [0], color='gray', lw=1, label='输入线')
        ]
        ax.legend(handles=handles, loc='upper right')
        ax.set_title(title)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("Longitude")
        ax.set_ylabel("Latitude")
        plt.grid(True)

        # -------------------------------
        # 6. 添加统计信息
        # -------------------------------
        # stats_text = (
        #     f"缓冲区：真值线两侧各 {results.buffer_distance} 米\n"
        #     f"真值累计长度：{results.total_truth_length:.2f} 米\n"
        #     f"输入累计长度：{results.total_input_length:.2f} 米\n"
        #     f"准确率：{results.precision * 100:.2f}%\n"
        #     f"召回率：{results.recall * 100:.2f}%\n"
        #     f"F1分数：{results.f1_score:.2f} (无单位)\n"
        #     f"累计命中长度：{results.hit_length:.2f} 米\n"
        #     f"累计冗余误报长度：{results.extra_length:.2f} 米\n"
        #     f"累计漏识别长度：{results.mis_match_length:.2f} 米\n"
        # )
        stats_text = results.get_dict()
        stats_text_formatted = "\n".join([f"{key}: {value}" for key, value in stats_text.items()])

        ax.text(1.01, 0.5, stats_text_formatted, transform=ax.transAxes, fontsize=10,
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # -------------------------------
        # 7. svg 保存与可视化
        # -------------------------------
        if vector_file_path:
            logger.debug(f"保存 svg 到 {vector_file_path}")
            plt.savefig(vector_file_path, format='svg', bbox_inches='tight')
            plt.close(fig)
        if is_visualize:
            plt.show()
            plt.close(fig)

    def evaluate(self, task_name: Optional[str], buffer_distance: float,
                 resample_segment_length: float = 1) -> LineMatchResult:
        """
        启动评测任务，执行匹配覆盖计算并返回结果。

        Args:
            task_name (Optional[str]): 任务名称。
            buffer_distance (float): 缓冲区距离（单位：米）。
            resample_segment_length (float): 重采样切分的段长度（单位：米），默认为 1。当设置为 None 时不进行切分。

        Returns:
            LineMatchResult: 包含匹配评测的各项指标结果的对象。
        """
        if buffer_distance is None:
            buffer_distance = self.evaluator_setting.line_buffer_length_meter
        if resample_segment_length is None:
            resample_segment_length = self.segment_length

        merged_title = f"{task_name}"
        logger.info(f"开始执行评测任务：{merged_title}")
        result = self.compute_coverage(buffer_distance=buffer_distance, segment_length=resample_segment_length)

        return result


class VectorizeOnlyLaneLineEvaluator(LaneLineEvaluator):
    """
    仅数量化车道线评测类，用于仅数量化的评测道路线的真值数据与输入数据。

    :param truth_intersection: 真值路口对象。
    :param input_intersection: 带评测路口对象
    :param evaluator_setting: 评测工具配置。
    """

    def __init__(self, truth_intersection: Intersection, input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams):
        super().__init__(truth_intersection, input_intersection, evaluator_setting)

        self.truth_lines_gdf = self.truth_intersection.lane_lines
        self.input_lines_gdf = self.input_intersection.lane_boundary_vectorize

        if self.input_lines_gdf is None:
            raise DataEmpty("输入值未带有仅数量化车道线数据。无法进行评测")
        if self.truth_lines_gdf is None:
            raise DataEmpty("真值值未带有仅数量化车道线数据。无法进行评测")

        logger.debug("初始化完毕")


if __name__ == '__main__':
    setting: EvaluatorParams = load_this("../../tests/lane_line_eva_dev_resources/setting.pickle")
    setting.line_buffer_length_meter = 1
    le = LaneLineEvaluator(
        load_this("../../tests/lane_line_eva_dev_resources/test_truth_intersection.pickle"),
        load_this("../../tests/lane_line_eva_dev_resources/test_truth_intersection.pickle"), setting)

    le.compute_coverage()