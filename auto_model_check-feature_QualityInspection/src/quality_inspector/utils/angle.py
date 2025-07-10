# -*- coding: utf-8 -*-
# @Time    : 2025/4/23 22:30
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : is_smooth.py

"""0423重构"""
import math
from typing import Tuple, List, Optional

import numpy as np
from geopandas import GeoDataFrame
from matplotlib import pyplot as plt
from pydantic import BaseModel, ConfigDict
from shapely import Point, LineString

from src.logger import logger
from src.quality_inspector.utils.line_process import LinePreprocessor


class FlexPointRecord(BaseModel):
    """
    数据模型，用于记录折点及其在该点处的偏转角度信息。

    Attributes:
        point (Point): 折点的几何对象 (Shapely Point)。
        deflection_angle (float): 在该点处的偏转角度（单位：度）。
                                   偏转角定义为 180 度减去内角。
    """
    point: Point
    deflection_angle: float  # 偏转角度
    model_config = ConfigDict(arbitrary_types_allowed=True)


class AngleEvaluator:
    @classmethod
    def is_small_fluctuation_points(cls, p1: Point, p2: Point, p3: Point, threshold_m: float = 0.02) -> bool:
        # """
        # 检查是否只是一个小波动。
        #
        # Args:
        #     p1 (Point): 前一个点。
        #     p2 (Point): 当前顶点。
        #     p3 (Point): 后一个点。
        #     threshold_m (float): 定义“小波动”的最大垂直距离阈值（单位：米）。
        #
        # Returns:
        #     bool: 如果p2作为三角形的顶点，经过它他的高的长度小于等于 threshold_m则返回 True (是小波动)，否则 False。
        # """
        # # 检查 p1 和 p3 是否几乎重合
        # # 使用 Point.distance 比直接比较坐标更稳健
        # logger.debug("有容忍")
        # if p1.distance(p3) < 1e-9:  # 使用一个非常小的容差
        #     # 如果 p1 和 p3 重合，p2 肯定是一个波动（或尖刺）
        #     return True
        #
        # line_p1p3 = LineString([p1, p3])
        # dist_ = p2.distance(line_p1p3)
        # return dist_ <= threshold_m
        """
        快速波动检测
        """
        if p1.distance(p3) < 1e-9:
            return True
        area = abs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2)
        return (2 * area / p1.distance(p3)) <= threshold_m

    @classmethod
    def calculate_interior_angle(cls, p1, p2, p3):
        """
        计算由三个点 p1, p2, p3 构成的内角，内角，内角
        """
        # 将点转换为 numpy 数组以便于向量运算
        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(p3)

        # 创建向量 BA 和 BC
        vec_ba = p1 - p2
        vec_bc = p3 - p2

        # 计算向量长度
        norm_ba = np.linalg.norm(vec_ba)
        norm_bc = np.linalg.norm(vec_bc)

        # 检查是否有零向量（重合点），避免除以零
        if norm_ba == 0 or norm_bc == 0:
            # 可以返回特定值或抛出异常，这里返回 180 表示直线（无转角）
            # 或者返回 None/NaN 表示无法计算
            return 180.0

        # 计算点积
        dot_product = np.dot(vec_ba, vec_bc)

        # 计算夹角的余弦值，限制在 [-1, 1] 范围内防止浮点误差
        cos_theta = np.clip(dot_product / (norm_ba * norm_bc), -1.0, 1.0)

        # 使用 np.arccos 计算角度
        angle_rad = np.arccos(cos_theta)

        # 转换为度
        angle_deg = math.degrees(angle_rad)

        return angle_deg

    @classmethod
    def find_sharp_turns(cls, line: LineString, max_deflection_degrees: float = 15.0,
                         fluctuation_tolerance_meter: Optional[float] = None) -> Tuple[bool, List[FlexPointRecord]]:
        """
        检查 LineString 是否包含锐角转弯（偏转角 > max_deflection_degrees）。

        Args:
            line (LineString): 待检查的线段
            max_deflection_degrees (float): 允许的最大偏转角
            fluctuation_tolerance_meter: 针对小波动的容忍值

        Returns:
            (bool, List[FlexPointRecord]): 是否包含锐角转弯及其详细信息。
        """
        # 检查输入是否为 LineString
        if not isinstance(line, LineString) or line.is_empty:
            return False, []

        coords = list(line.coords)

        # 如果点数少于3个，证明这就是一条很直线，必然没有折角，提早返回
        if len(coords) < 3:
            return False, []

        sharp_turn_records: List[FlexPointRecord] = []
        epsilon = 1e-9  # 用于浮点比较的容差

        for i in range(1, len(coords) - 1):
            p1 = coords[i - 1]
            p2 = coords[i]
            p3 = coords[i + 1]

            # 可选：检查点是否几乎重合，避免无效计算 (calculate_interior_angle 已处理部分情况)
            if Point(p1).distance(Point(p2)) < epsilon or Point(p2).distance(Point(p3)) < epsilon:
                continue

            interior_angle = cls.calculate_interior_angle(p1, p2, p3)
            deflection_angle = 180.0 - interior_angle
            # logger.debug(interior_angle)
            # 检查偏转角是否大于阈值
            if deflection_angle > max_deflection_degrees + epsilon:
                vertex_point = Point(p2)

                if fluctuation_tolerance_meter:
                    p1_point = Point(p1)
                    p3_point = Point(p3)

                    if not cls.is_small_fluctuation_points(p1_point, vertex_point, p3_point,
                                                           threshold_m=fluctuation_tolerance_meter):
                        # 如果偏转角大 *且* 不是小波动，才记录为锐角转弯
                        record = FlexPointRecord(point=vertex_point, deflection_angle=deflection_angle)
                        sharp_turn_records.append(record)
                        continue

                # 创建 FlexPointRecord 实例
                record = FlexPointRecord(point=vertex_point, deflection_angle=deflection_angle)
                sharp_turn_records.append(record)

        has_sharp_turn = bool(sharp_turn_records)  # 直接判断sharp_turn列表是否非空

        return has_sharp_turn, sharp_turn_records

    @classmethod
    def process_gdf_for_sharp_turns(cls, segments_gdf: GeoDataFrame,
                                    max_deflection_degrees: float = 15.0,
                                    fluctuation_m: float = None) -> GeoDataFrame:  # 添加波动阈值参数
        # 应用锐角检测函数，传入波动阈值
        results = segments_gdf['geometry'].apply(
            lambda geom: cls.find_sharp_turns(geom,
                                              max_deflection_degrees=max_deflection_degrees,
                                              fluctuation_tolerance_meter=fluctuation_m)  # 传递阈值
        )

        # 记录是不是折角，记录折角信息
        segments_gdf['is_reflex_angle'] = results.apply(lambda x: x[0])
        segments_gdf['sharp_turn_details'] = results.apply(lambda x: x[1])

        return segments_gdf

    @classmethod
    def extract_sharp_turn_points(cls, gdf_segments: GeoDataFrame) -> GeoDataFrame:
        """
        从线段gdf中抽取所有折点及其折角，返回一个点型GeoDataFrame。
        每个点有字段：geometry, angle, segment_idx, 其它可选属性。
        """
        records = []
        for idx, row in gdf_segments.iterrows():
            details = row.get('sharp_turn_details', None)
            if details:
                for rec in details:
                    # 兼容两种结构
                    if hasattr(rec, 'point') and hasattr(rec, 'angle'):
                        point = rec.point
                        angle = rec.angle
                    elif isinstance(rec, dict):
                        point = rec['point']
                        angle = rec['angle']
                    else:
                        continue
                    records.append({
                        'geometry': point,
                        'angle': angle,
                        'segment_idx': idx,  # 方便回查来源segment
                        # 可以扩展其它属性
                    })
        if records:
            gdf_points = GeoDataFrame(records, geometry='geometry', crs=gdf_segments.crs)
        else:
            gdf_points = GeoDataFrame(columns=['geometry', 'angle', 'segment_idx'], geometry='geometry',
                                      crs=gdf_segments.crs)
        return gdf_points

    @classmethod
    def plot_segments_with_stats(cls, gdf_segments: GeoDataFrame):
        with_turn = gdf_segments[gdf_segments['is_reflex_angle']]
        without_turn = gdf_segments[~gdf_segments['is_reflex_angle']]

        # 长度统计
        total_len_reflex = with_turn.length.sum()
        total_len_normal = without_turn.length.sum()

        turn_point_gdf = cls.extract_sharp_turn_points(gdf_segments)
        gdf_segments = gdf_segments.to_crs(epsg=4326)
        if not turn_point_gdf.empty:
            turn_point_gdf = turn_point_gdf.to_crs(epsg=4326)

        # 分组
        with_turn = gdf_segments[gdf_segments['is_reflex_angle']]
        without_turn = gdf_segments[~gdf_segments['is_reflex_angle']]

        # 折角属性
        turn_angles = turn_point_gdf['angle'].tolist() if not turn_point_gdf.empty else []
        max_angle = max(turn_angles) if turn_angles else None
        min_angle = min(turn_angles) if turn_angles else None

        # 最大/最小折角点
        if turn_angles:
            max_idx = turn_point_gdf['angle'].idxmax()
            min_idx = turn_point_gdf['angle'].idxmin()
            max_point_row = turn_point_gdf.loc[max_idx]
            min_point_row = turn_point_gdf.loc[min_idx]
        else:
            max_point_row = min_point_row = None

        # 统计文本
        stats = {
            "有折角段总长(m)": f"{total_len_reflex:.2f}",
            "无折角段总长(m)": f"{total_len_normal:.2f}",
            "折点数量": turn_point_gdf.shape[0],
            "最大折角(°)": f"{max_angle:.2f}" if max_angle is not None else "-",
            "最小折角(°)": f"{min_angle:.2f}" if min_angle is not None else "-",
            "平均折角(°)": f"{(sum(turn_angles) / len(turn_angles)):.2f}" if turn_angles else "-"
        }
        stats_text_formatted = "\n".join([f"{key}: {value}" for key, value in stats.items()])

        fig, ax_main = plt.subplots(figsize=(24, 18))  # 创建绘图窗口

        # 主图绘制
        # gdf_segments.boundary.plot(ax=ax_main, color='lightgrey', linewidth=0.5, zorder=0)
        if not with_turn.empty:
            with_turn.plot(ax=ax_main, color='red', linewidth=2, label='有折角')
        without_turn.plot(ax=ax_main, color='green', linewidth=2, alpha=0.4, label='无折角')

        # 所有折点
        if not turn_point_gdf.empty:
            # turn_point_gdf.plot(ax=ax_main, color='blue', markersize=6, zorder=3, alpha=0.3, label='折点')
            # 只标注最大、最小折角
            if max_point_row is not None:
                ax_main.text(max_point_row.geometry.x, max_point_row.geometry.y, f"最大 {max_point_row.angle:.1f}°",
                             fontsize=12, color='blue', ha='center', va='bottom', weight='bold',
                             bbox=dict(boxstyle="round,pad=0.2", fc="yellow", alpha=0.7))
            if min_point_row is not None:
                ax_main.text(min_point_row.geometry.x, min_point_row.geometry.y, f"最小 {min_point_row.angle:.1f}°",
                             fontsize=12, color='purple', ha='center', va='top', weight='bold',
                             bbox=dict(boxstyle="round,pad=0.2", fc="yellow", alpha=0.7))

        # 统计信息
        ax_main.text(1.01, 0.5, stats_text_formatted, transform=ax_main.transAxes, fontsize=12,
                     va='center', ha='left', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

        ax_main.set_title("折角分布及统计（仅标注最大/最小折角）")
        ax_main.legend(loc='upper left')
        ax_main.set_xlabel("经度")
        ax_main.set_ylabel("纬度")

        plt.tight_layout()
        plt.show()


def get_angle_usability(line_gdf: GeoDataFrame, max_deflection_degrees: float = 15.0, fluctuation_m: float = None,
                        segment_length: float = 1.0) -> GeoDataFrame:
    """
    传入线要素df，最大可接受的折角度数，可接受的小波动范围，切分线段长度。
    返回一个包含折角情况的GDF
    """
    seg = LinePreprocessor.turn_line_to_segments(line_gdf, segment_length)
    res = AngleEvaluator.process_gdf_for_sharp_turns(segments_gdf=seg,
                                                     max_deflection_degrees=max_deflection_degrees,
                                                     fluctuation_m=fluctuation_m)
    return res
