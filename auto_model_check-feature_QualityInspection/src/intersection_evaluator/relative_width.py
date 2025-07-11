# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 20:41
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : relative_width.py
import pickle
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List

import numpy as np
from geopandas import GeoDataFrame, sjoin
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
from pandas import DataFrame, to_numeric
from shapely import Point, LineString

from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluatorParams


class Config:
    # populate_by_name = True
    arbitrary_types_allowed = True


def to_dict(self):
    return self.__analyze_results()


def __analyze_results(self):
    """
    分析车道宽度误差结果并输出两西格玛范围内的值。
    """
    # 首先提取 "width_diff" 非 NaN 的行
    errors_df = self.gdf[self.gdf["width_diff"].notna()]
    if errors_df.empty:
        raise ValueError("结果中有效的相对宽度差统计值为空")

    # 遍历每一行，检查 "width_diff" 的值是否为数字，如果不是则打印该行信息
    problematic_rows = []
    for idx, row in errors_df.iterrows():
        value = row["width_diff"]
        try:
            # 尝试将其转换为 float（例如如果已经是数字则不会出错）
            _ = float(value)
        except Exception as e:
            problematic_rows.append((idx, value))
            logger.error(f"第 {idx} 行数据转换错误：{value}，错误信息：{e}")

    if problematic_rows:
        logger.error("下列行的 width_diff 无法转换为数字：")
        for idx, value in problematic_rows:
            logger.error(f"  行 {idx}: {value}")
        # 如果你想中止处理，可以选择抛出异常
        raise TypeError("存在无法转换为数字的 width_diff 数据，请检查日志以获得详细信息。")

    # 如果数据均正常，则转换为数值类型（自动转换那些可转换的，例如字符串 "0.3" 等）
    width_diff_numeric = to_numeric(errors_df["width_diff"], errors="coerce")
    # 再次过滤掉转换过程中产生的 NaN（如值原本不合法的）
    valid_width_diff = width_diff_numeric.dropna()
    if valid_width_diff.empty:
        raise ValueError("过滤后没有有效的 width_diff 数据。")

    # 计算均值和标准差
    mean_error = np.mean(valid_width_diff)
    std_dev_error = np.std(valid_width_diff)
    two_sigma_threshold = 2 * std_dev_error

    # 对于两西格玛范围内的记录，需要注意遍历 DataFrame 的方式
    # 如果你的数据存储方式不是列表而是 DataFrame，建议使用 .itertuples() 或 .iterrows()
    two_sigma_results = []
    for idx, row in self.gdf.iterrows():
        try:
            # 提取 width_diff 并转换为 float（如果不能转换，则跳过）
            width_diff_val = float(row["width_diff"])
            if width_diff_val <= two_sigma_threshold:
                two_sigma_results.append(row)
        except Exception as e:
            logger.error(f"第 {idx} 行在比较时出错: {row['width_diff']}，错误信息：{e}")
            continue

    # 计算两西格玛范围内的误差平均值
    two_sigma_errors = []
    for idx, row in GeoDataFrame(two_sigma_results).iterrows():
        try:
            two_sigma_errors.append(float(row["width_diff"]))
        except Exception as e:
            logger.error(f"在两西格玛范围内结果，第 {idx} 行转换出错: {row['width_diff']}，错误信息：{e}")
            continue

    mean_two_sigma = np.mean(two_sigma_errors) if two_sigma_errors else None

    logger.debug(f"平均误差: {mean_error:.4f}")
    logger.debug(f"标准差: {std_dev_error:.4f}")
    logger.debug(f"两西格玛阈值: {two_sigma_threshold:.4f}")
    logger.debug(f"在两西格玛范围内的结果数量/总数: {len(two_sigma_results)} / {len(self.gdf)}")
    if mean_two_sigma is not None:
        logger.debug(f"两西格玛范围内结果的平均误差: {mean_two_sigma:.4f}")
    else:
        logger.debug("没有两西格玛范围内的结果数据。")

    return {
        "two_sigma_results": two_sigma_results,
        "two_sigma_threshold": two_sigma_threshold,
        "mean_error": mean_error,
        "std_dev_error": std_dev_error,
        "mean_two_sigma": mean_two_sigma,
    }


class LaneWidthEvaluator(object):
    """
    道路评测类，用于评测道路线的真值数据与输入数据。

    :param truth_lines_file_path: 真值线段文件路径。
    :param input_lines_file_path: 输入线段文件路径。
    :param truth_data_filter_conditions: 真值数据的筛选条件（可选）。
    :param input_data_filter_conditions: 输入数据的筛选条件（可选）。
    :param global_rect: 全局的范围框，优先级高于凸包（可选）。
    :param convex_hull: 用于计算的凸包（可选）。
    :param reference_material_file_path: 参考材料文件路径（可选）。
    :param reference_material_filter_conditions: 参考材料的筛选条件（可选）。
    """

    def __init__(self,
                 truth_intersection: Intersection,
                 input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams  # 阈值等设置，直接扔进来
                 ):
        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.evaluator_setting = evaluator_setting

        self.truth_lines_gdf = self.truth_intersection.lane_lines
        self.input_lines_gdf = self.input_intersection.lane_lines

        if self.truth_intersection.lane_lines is None or self.truth_intersection.intersection_area is None or self.truth_intersection.lane_centerlines is None:
            raise Exception("缺乏真值要素，无法评测相对宽度：需要真值车道线、真值路口面、真值中心线")

        self.usable_centerline = self.get_usable_centerline(self.truth_intersection.intersection_area,
                                                            self.truth_intersection.lane_centerlines)

    @staticmethod
    def get_usable_centerline(polygon_gdf: GeoDataFrame, linestring_gdf: GeoDataFrame) -> GeoDataFrame:
        """
        获取参与计算的中心线，目前的策略是，如果这个中心线与路口面相交，那么判断属于路口中心，不参与计算
        """

        # 确保几何图形的 CRS 相同
        if polygon_gdf.crs != linestring_gdf.crs:
            linestring_gdf = linestring_gdf.to_crs(polygon_gdf.crs)

        # 使用空间连接找到相交的 Linestring
        intersection = sjoin(linestring_gdf, polygon_gdf, how="inner", predicate="intersects")

        # 获取相交 Linestring 的索引
        intersection_index = intersection.index

        # 从原始 Linestring GeoDataFrame 中删除相交的 Linestring
        remaining_linestrings = linestring_gdf.drop(intersection_index)

        return remaining_linestrings

    def evaluate(self, step_length: float = 1) -> GeoDataFrame:
        """评测相对宽度的函数"""
        return self.relative_width_statistics(truth_lane_lines=self.truth_lines_gdf,
                                              truth_centerlines=self.truth_intersection.lane_centerlines,
                                              input_lane_lines=self.input_lines_gdf,
                                              step_length=self.evaluator_setting.resample_segment_length_meter)

    def visualize(self, result: GeoDataFrame):
        self.visualize_lane_width_relative_error_results(truth_lane_lines=self.truth_lines_gdf,
                                                         truth_centerlines=self.truth_intersection.lane_centerlines,
                                                         input_lane_lines=self.input_lines_gdf,
                                                         result=result,
                                                         is_plot_perp=False, is_plot_skipped_point=False)
        return plt

    @staticmethod
    def __create_perpendicular_line(centerline: LineString, point: Point, perpendicular_line_length):
        """
        传入一个Linestring创建垂直于中心线切线的线段。

        参数：
            centerline: 中心线的 LineString 对象。
            point: 中心线上的 Point 对象。
            length: 垂直线段的长度。
        返回：
            LineString: 垂直于中心线切线的线段。
        """
        # 找到最近的线段
        min_dist = float('inf')
        closest_segment = None
        if not isinstance(centerline, LineString):
            logger.error(f"! {type(centerline)}")
        for i in range(len(centerline.coords) - 1):
            segment = LineString([centerline.coords[i], centerline.coords[i + 1]])
            dist = point.distance(segment)
            # dist = segment.distance(point)
            if dist < min_dist:
                min_dist = dist
                closest_segment = segment

        # 计算切线角度 (使用最近线段的方向作为切线方向的近似)
        dx = closest_segment.coords[1][0] - closest_segment.coords[0][0]
        dy = closest_segment.coords[1][1] - closest_segment.coords[0][1]
        tangent_angle = np.arctan2(dy, dx)

        # 计算垂直角度
        perpendicular_angle = tangent_angle + np.pi / 2

        # 创建垂直线段
        perp_line = LineString([
            (point.x, point.y),
            (point.x + perpendicular_line_length * np.cos(perpendicular_angle),
             point.y + perpendicular_line_length * np.sin(perpendicular_angle))
        ])

        return perp_line

    @staticmethod
    def get_gdf_for_relative_width_statistics(data: List):
        """
        返回用以保存统计结果的gdf，预先设置好crs
        我感觉用有个独立的函数来表达这个过程，比较方便查看和维护gdf的结构
        """
        # empty_gdf = gpd.GeoDataFrame({'geometry': [],
        #                               'truth_lane_width': [], 'input_lane_width': [],
        #                               "left_perp_geom": [], "right_perp_geom": [],
        #                               "is_skipped": [], "width_diff": []},crs="EPSG:32650")  # 根据需要添加列
        # Extract data into separate lists
        df = DataFrame.from_records(data)
        gdf = GeoDataFrame(df, geometry='point', crs="EPSG:32650")

        return gdf

    def process_centerline(self, centerline: LineString, index, truth_lane_lines, bev_lane_lines, step_length,
                           buffer_length):
        results_ = []  # 存储当前中心线的比较结果

        for distance in np.arange(0, centerline.length, step_length):  # 沿中心线以指定步长移动
            point = centerline.interpolate(distance)  # 获取沿中心线当前距离的点

            left_perp = self.__create_perpendicular_line(centerline, point, perpendicular_line_length=buffer_length)
            right_perp = self.__create_perpendicular_line(centerline, point, perpendicular_line_length=-buffer_length)

            # 查找与真值车道线的交点
            gt_left_intersections = [left_perp.intersection(lane) for lane in truth_lane_lines.geometry if
                                     left_perp.intersects(lane)]
            gt_right_intersections = [right_perp.intersection(lane) for lane in truth_lane_lines.geometry if
                                      right_perp.intersects(lane)]

            # 检查是否在两侧都找到了交点
            if gt_left_intersections and gt_right_intersections:
                gt_left_closest = min(gt_left_intersections, key=lambda x: x.distance(point))
                gt_right_closest = min(gt_right_intersections, key=lambda x: x.distance(point))
                standard_width = gt_left_closest.distance(gt_right_closest)

                # 对输入的车道线执行相同操作
                bev_left_intersections = [left_perp.intersection(lane) for lane in bev_lane_lines.geometry if
                                          left_perp.intersects(lane)]
                bev_right_intersections = [right_perp.intersection(lane) for lane in bev_lane_lines.geometry if
                                           right_perp.intersects(lane)]

                # 检查是否在两侧都检测到了输入的车道线
                if bev_left_intersections and bev_right_intersections:
                    bev_left_closest = min(bev_left_intersections, key=lambda x: x.distance(point))
                    bev_right_closest = min(bev_right_intersections, key=lambda x: x.distance(point))
                    input_width = bev_left_closest.distance(bev_right_closest)

                    # 计算车道宽度差异作为横向相对误差
                    relative_error = abs(standard_width - input_width)
                    results_.append({
                        "point": point,  # Point类型
                        "left_perp_geom": left_perp,  # LineString类型
                        "right_perp_geom": right_perp,  # LineString类型
                        "is_skipped": False,  # Bool类型
                        "truth_lane_width": standard_width,  # Float类型
                        "input_lane_width": input_width,  # Float类型
                        "width_diff": relative_error
                    })
                else:
                    results_.append({
                        "point": point,
                        "left_perp_geom": left_perp,
                        "right_perp_geom": right_perp,
                        "is_skipped": True,
                    })
            else:
                results_.append({
                    "point": point,
                    "left_perp_geom": left_perp,
                    "right_perp_geom": right_perp,
                    "is_skipped": True,
                })

        return results_  # 返回此中心线的结果

    def relative_width_statistics(self, truth_lane_lines: GeoDataFrame, truth_centerlines: GeoDataFrame,
                                  input_lane_lines: GeoDataFrame, step_length: float = 1) -> GeoDataFrame:
        buffer_length = 4.5
        all_results = []
        from tqdm import tqdm

        logger.info(f"使用多线程处理每条车道中心线。以中心线向两侧增加{buffer_length}米探索两边车道线.")
        # 使用多线程处理每条中心线
        with ThreadPoolExecutor(max_workers=40) as executor:
            futures = {
                executor.submit(self.process_centerline, centerline, index, truth_lane_lines, input_lane_lines,
                                step_length,
                                buffer_length): index
                for index, centerline in enumerate(truth_centerlines.geometry)
            }

            # 使用tqdm包装futures对象，以显示进度条
            for future in tqdm(as_completed(futures), total=len(futures), desc="Processing centerlines"):
                index = futures[future]
                try:
                    result = future.result()
                    all_results.extend(result)  # 合并结果
                except Exception as e:
                    logger.exception(f"Centerline {index} generated an exception: {e}")

        logger.info("完成车道中心线遍历，正在汇总结果为GeoDataFrame。")
        # 将结果转换为 GeoDataFrame
        gdf = self.get_gdf_for_relative_width_statistics(all_results)
        return gdf

        # buffer_length = 4.5
        # all_results = []
        # from tqdm import tqdm
        #
        # logger.info("使用串行处理每条车道中心线")
        #
        # # 使用串行处理，使用线程池
        # for index, centerline in tqdm(enumerate(truth_centerlines.geometry),
        #                               total=len(truth_centerlines),
        #                               desc="Processing centerlines"):
        #     try:
        #         result = process_centerline(centerline, index, truth_lane_lines, input_lane_lines, step_length,
        #                                     buffer_length)
        #         all_results.extend(result)  # 合并结果
        #     except Exception as e:
        #         logger.exception(f"Centerline {index} generated an exception: {e}")
        #
        # logger.info("完成车道中心线遍历，正在汇总结果。")
        # # 将结果转换为 GeoDataFrame
        # gdf = get_gdf_for_relative_width_statistics(all_results)
        # return gdf

    @staticmethod
    def visualize_lane_width_relative_error_results(truth_lane_lines: GeoDataFrame,
                                                    truth_centerlines: GeoDataFrame,
                                                    input_lane_lines: GeoDataFrame,
                                                    result: GeoDataFrame,
                                                    is_plot_perp=False,
                                                    is_plot_skipped_point=False,
                                                    default_bins=None):
        """可视化车道宽度比较结果。"""
        logger.info(f"开始对基于车道中心线的车道相对宽度评测gdf进行可视化...坐标系={truth_lane_lines.crs}")
        # 设置默认的直方图分箱
        if default_bins is None:
            default_bins = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 1]

        # 创建主图和下方直方图的子图，调整高度
        fig, (ax_main, ax_hist) = plt.subplots(2, 1, figsize=(30, 20),
                                               gridspec_kw={'height_ratios': [3, 1]})  # 改为2行1列

        # 在创建完毕之后，调整下方子图的宽度
        pos = ax_hist.get_position()  # 获取当前ax_hist的位置 (Bbox格式：x0, y0, width, height)
        new_width = pos.width * 0.5  # 将宽度缩小为原来的50%
        delta = (pos.width - new_width) / 2.0  # 计算左侧需要右移的距离，使子图居中

        # 设置新的位置：[新的x0, y0, 新的宽度, 高度]，这里只调整横向宽度
        ax_hist.set_position([pos.x0 + delta, pos.y0, new_width, pos.height])

        # 绘制真值车道线
        truth_lane_lines.plot(ax=ax_main, color='orange', label='真值车道线')

        # 绘制输入车道
        input_lane_lines.plot(ax=ax_main, color='blue', label='待评测车道', alpha=0.5)

        # 绘制中心线
        truth_centerlines.plot(ax=ax_main, linestyle='-.', color='gray', label='中心线', alpha=0.3)

        added_labels = set()

        # 绘制所有评估点
        for idx, row in result.iterrows():
            point = row["point"]
            left_perp = row["left_perp_geom"]
            right_perp = row["right_perp_geom"]

            # 绘制抽样点
            if row["width_diff"] is not None:
                width_diff = float(row["width_diff"])
                if width_diff <= 0.2:
                    label = '相对距离小于0.2米抽样点'
                    ax_main.plot(point.x, point.y, marker='o', color='green', markersize=3, alpha=0.1,
                                 label=label if label not in added_labels else "")
                    added_labels.add(label)
                elif 0.2 < width_diff <= 0.5:
                    label = '相对距离0.2-0.5米抽样点'
                    ax_main.plot(point.x, point.y, marker='o', color='red', markersize=3, alpha=0.2,
                                 label=label if label not in added_labels else "")
                    added_labels.add(label)
                elif 0.5 < width_diff <= 1:
                    label = '相对距离0.5-1米抽样点'
                    ax_main.plot(point.x, point.y, marker='o', color='red', markersize=3, alpha=1,
                                 label=label if label not in added_labels else "")
                    added_labels.add(label)
                elif width_diff > 1:
                    label = '相对距离大于1米抽样点'
                    ax_main.plot(point.x, point.y, marker='x', color='black', markersize=3, alpha=1,
                                 label=label if label not in added_labels else "")
                    added_labels.add(label)

                # 绘制跳过点
                if row["is_skipped"] and is_plot_skipped_point:
                    label = '跳过点'
                    ax_main.plot(point.x, point.y, marker='x', color='yellow', markersize=5,
                                 label=label if label not in added_labels else "")
                    added_labels.add(label)

                    if is_plot_perp:
                        ax_main.plot(*left_perp.xy, color='purple', linestyle='--', linewidth=0.5, alpha=0.2)  # 左侧缓冲线
                        ax_main.plot(*right_perp.xy, color='purple', linestyle='--', linewidth=0.5, alpha=0.2)  # 右侧缓冲线

        # 绘制主图的统计信息
        relative_errors = result["width_diff"].dropna()  # 去除缺失值
        mean_error = np.mean(relative_errors)
        std_error = np.std(relative_errors)
        median_error = np.median(relative_errors)
        percentile_95 = np.percentile(relative_errors, 95)

        stats_text = (
            f"均值: {mean_error:.2f}\n"
            f"标准差: {std_error:.2f}\n"
            f"中位数: {median_error:.2f}\n"
            f"95% 分位数: {percentile_95:.2f}"
        )

        # 添加统计信息文本
        ax_main.text(0.05, 0.95, stats_text, transform=ax_main.transAxes, fontsize=12,
                     verticalalignment='top', horizontalalignment='left',
                     bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.5'))

        ax_main.set_title('横向误差--车道宽度统计')
        ax_main.legend()  # 显示图例

        # 绘制直方图，计算百分比
        counts, edges, patches = ax_hist.hist(relative_errors, bins=default_bins, color='skyblue', edgecolor='black',
                                              orientation='horizontal', density=True)

        # 使用桶的边界作为刻度
        ax_hist.set_yticks(edges)  # 设置y轴刻度为桶的边界
        ax_hist.set_yticklabels([f'{edge:.1f}' for edge in edges])  # 设置标签格式

        # 计算百分比并显示
        percentages = counts * 100 / np.sum(counts)  # 转换为百分比
        ax_hist.set_title('车道宽度误差分布')
        ax_hist.set_xlabel('百分比')
        ax_hist.set_ylabel('车道宽度误差 (m)')

        # 在每个直方图柱子上添加百分比
        for i in range(len(patches)):
            ax_hist.text(patches[i].get_width() + 0.5, patches[i].get_y() + patches[i].get_height() / 2,
                         f'{percentages[i]:.1f}%', va='center', ha='left', fontsize=10)

        # # 显示图像
        # plt.tight_layout()
        # plt.show()
        return plt

#
# if __name__ == '__main__':
#     t: LaneWidthEvaluateResult = pickle.load(open('../../tests/lane_line_eva_dev_resources/data.pickle', 'rb'))
#     print(t.to_dict())
