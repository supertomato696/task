# -*- coding: utf-8 -*-
# @Time    : 2025/3/18 10:41
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : trafficlight.py
import os
import copy
from typing import List, Optional, Dict

from geopandas import GeoDataFrame, GeoSeries
from matplotlib import pyplot as plt
from pydantic import BaseModel, ConfigDict
from shapely import Point, LineString
from shapely.ops import unary_union

from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluatorParams


class PointPairEvaluationResult:
    """评测结果类"""

    def __init__(self,
                 truth_point: Point,
                 input_point: Point):
        self.truth_point = truth_point
        self.input_point = input_point

    def __repr__(self):
        return f"Distance={self.distance:.4f} 米)"

    @property
    def relation_geom(self) -> LineString:
        return LineString([self.truth_point, self.input_point])

    @property
    def distance(self) -> float:
        # TODO 判空
        return self.truth_point.distance(self.input_point)


class TrafficLightEvaluateResult(BaseModel):
    truth_traffic_light_gdf: GeoDataFrame
    input_traffic_light_gdf: GeoDataFrame
    unmatch_input_list: List[Point] = None
    matched_pair_list: List[PointPairEvaluationResult] = None

    model_config = ConfigDict(arbitrary_types_allowed=True)

    @property
    def crs(self):
        return self.input_traffic_light_gdf.crs

    @property
    def recall(self):
        return len(self.matched_pair_list) / self.truth_traffic_light_gdf.shape[0] if \
            self.truth_traffic_light_gdf.shape[0] != 0 else 0

    @property
    def precision(self):
        return len(self.matched_pair_list) / self.input_traffic_light_gdf.shape[0] if \
            self.input_traffic_light_gdf.shape[0] != 0 else 0

    @property
    def f1_score(self) -> float:
        """
        计算F1分数，综合考虑精准率和召回率。

        :return: float F1分数
        """
        return 2 * (self.precision * self.recall) / (self.precision + self.recall) if (
                                                                                              self.precision + self.recall) > 0 else 0

    def get_dict(self) -> Dict:
        distances = [result.distance for result in self.matched_pair_list]

        stats_dict = {
            "准确率": f"{self.precision * 100:.2f}%",
            "召回率": f"{self.recall * 100:.2f}%",
            "F1分数": f"{self.f1_score: .4f}",
            "已匹配真值数量": f"{len(self.matched_pair_list)}",
            "未匹配真值数量": f"{self.truth_traffic_light_gdf.shape[0] - len(self.matched_pair_list)}",
            "已匹配输入值数量": f"{len(self.matched_pair_list)}",
            "未匹配输入值数量": f"{len(self.unmatch_input_list)}",
            "平均偏移距离(米)": f"{sum(distances) / len(distances): .2f}",
            "最大偏移距离(米)": f"{max(distances): .2f}",
            "最小偏移距离(米)": f"{min(distances): .2f}"
        }
        return stats_dict

    def to_gdf(self):
        """将 PointPairEvaluationResult 列表转换为 GeoDataFrame"""
        geometries = [result.relation_geom for result in self.matched_pair_list]
        distances = [result.distance for result in self.matched_pair_list]

        gdf = GeoDataFrame({'distance': distances}, geometry=geometries, crs=self.crs)
        return gdf

    @property
    def unmatch_input_gdf(self) -> GeoDataFrame:
        """将unmatch_input_list (Point 列表) 转换为 GeoDataFrame"""
        if self.unmatch_input_list is None:
            return GeoDataFrame()  # 或者 return None

        gdf = GeoDataFrame(geometry=self.unmatch_input_list, crs=self.crs)
        return gdf

    @property
    def match_input_gdf(self) -> GeoDataFrame:
        if self.matched_pair_list is None:
            return GeoDataFrame()

        gdf = GeoDataFrame(geometry=[item.input_point for item in self.matched_pair_list], crs=self.crs)
        return gdf


class TrafficlightEvaluator(object):
    """
    道路评测类，用于评测道路线的真值数据与输入数据。
    """

    def __init__(self,
                 truth_intersection: Intersection,
                 input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams  # 阈值等设置，直接扔进来
                 ):
        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.evaluator_setting = evaluator_setting

        self.truth_trafficlight_gdf = self.truth_intersection.traffic_lights
        self.input_trafficlight_gdf = self.input_intersection.traffic_lights

        # TODO 判空
        # if self.truth_trafficlight_gdf is None:
        #     raise Exception("缺乏真值交通灯")
        #
        # if self.input_trafficlight_gdf is None:
        #     raise Exception("缺乏待评测交通灯")
        # self.input_trafficlight_gdf = self.filter_input_trafficlights(truth_trafficlight_gdf=self.truth_trafficlight_gdf,
        #                                                               input_trafficlight_gdf=self.input_trafficlight_gdf_raw)

    def filter_input_trafficlights(self, truth_trafficlight_gdf: GeoDataFrame,
                                   input_trafficlight_gdf: GeoDataFrame) -> GeoDataFrame:
        # if not truth_trafficlight_gdf:
        #     truth_trafficlight_gdf = self.truth_trafficlight_gdf
        # if not input_trafficlight_gdf:
        # input_trafficlight_gdf = self.input_trafficlight_gdf

        gdf_truth = truth_trafficlight_gdf.copy().reset_index(drop=True)
        gdf_truth["buffer"] = gdf_truth.geometry.buffer(5)

        # 构建空间索引，提升相交查询效率
        sindex = gdf_truth.sindex

        # 2. 利用并查集（Union-Find）对真值多边形进行聚类
        # 初始化每个多边形为各自的根节点
        parent = {idx: idx for idx in gdf_truth.index}

        def find(i):
            """找到节点 i 的根，同时路径压缩"""
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

        # 遍历每个真值buffer多边形，根据缓冲区相交情况合并到同一聚类中
        for idx, row in gdf_truth.iterrows():
            buffered_geom = row["buffer"]
            # 利用空间索引查询与当前多边形可能相交的候选项（候选的边界框可能相交）
            possible_matches_index = list(sindex.intersection(buffered_geom.bounds))
            for other_idx in possible_matches_index:
                # 避免重复比较，可以要求只比较索引大于当前的
                if other_idx <= idx:
                    continue
                other_buffer = gdf_truth.loc[other_idx]["buffer"]
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

        for det_idx, det_row in input_trafficlight_gdf.iterrows():
            det_geom = det_row.geometry
            # 检查输入多边形是否与任一聚类区域相交
            for cluster_geom in cluster_geoms.values():
                if det_geom.intersects(cluster_geom):
                    selected_detection_ids.append(det_idx)
                    break

        # self.visualize_all(truth_gdf=self.truth_trafficlight_gdf, input_gdf=self.input_trafficlight_gdf,clusters=clusters,selected_ids=selected_detection_ids,buffer_radius=5)

        # 获取参与评测的输入多边形结果子集
        gdf_eval_input = input_trafficlight_gdf.loc[selected_detection_ids]

        return gdf_eval_input

    @staticmethod
    def match_traffic_lights(true_traffic_light_point: GeoSeries, input_traffic_light_point: GeoSeries,
                             max_distance: float = 3) -> tuple[list[tuple[Point, Point]], list[Point]]:
        """
        二维数组，以真值为基础找匹配
        """
        matched: dict[Point: Point] = {}

        unmatched_true_traffic_lights = copy.deepcopy(input_traffic_light_point.tolist())

        for true_point in true_traffic_light_point:
            # 候选列表
            candidates = []
            for input_point in input_traffic_light_point:
                distance = true_point.distance(input_point)
                if distance <= max_distance:
                    candidates.append((input_point, distance))

            if candidates:
                candidates.sort(key=lambda x: (x[1]))
                matched_point = candidates[0][0]
                matched[true_point] = matched_point
                # print(matched_point)
                if matched_point in unmatched_true_traffic_lights:
                    unmatched_true_traffic_lights.remove(matched_point)

        # 筛选出未匹配的输入点
        # matched将字典中的值收集到一个集合中，用于快速查找
        dict_values_set = set(matched.values())
        # 过滤出不在字典值集合中的ppint
        unmatched_input_traffic_light = [value for value in input_traffic_light_point if value not in dict_values_set]

        return list(matched.items()), unmatched_input_traffic_light

    def visualize_buffers_and_inputs(self, truth_gdf: GeoDataFrame, input_gdf: GeoDataFrame, buffer_radius: float):
        """
        可视化真值点、它们的缓冲区范围以及输入点。
        """
        # 创建缓冲区
        truth_gdf["buffer"] = truth_gdf.geometry.buffer(buffer_radius)

        # 绘制
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))

        # 绘制真值点
        truth_gdf.plot(ax=ax, color="blue", markersize=10, label="Truth Points")

        # 绘制缓冲区
        truth_gdf["buffer"].plot(ax=ax, color="blue", alpha=0.3, label="Truth Buffers")

        # 绘制输入点
        input_gdf.plot(ax=ax, color="red", markersize=10, label="Input Points")

        # 标题和图例
        ax.set_title("Truth Points, Buffers, and Input Points", fontsize=16)
        ax.legend()
        plt.show()

    def visualize_clusters_and_inputs(gdf_truth: GeoDataFrame, clusters: dict, input_gdf: GeoDataFrame):
        """
        可视化真值点的聚类结果以及输入点。
        """
        # 为每个点分配聚类标签
        cluster_labels = []
        for idx in gdf_truth.index:
            for root, indices in clusters.items():
                if idx in indices:
                    cluster_labels.append(root)
                    break

        # 将簇标签加入 GeoDataFrame
        gdf_truth["cluster"] = cluster_labels

        # 绘制
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))

        # 绘制聚类结果（每个簇用不同颜色）
        gdf_truth.plot(ax=ax, column="cluster", categorical=True, legend=True, markersize=10, cmap="tab20",
                       label="Truth Clusters")

        # 绘制输入点
        input_gdf.plot(ax=ax, color="red", markersize=10, label="Input Points")

        # 标题和图例
        ax.set_title("Truth Clusters and Input Points", fontsize=16)
        ax.legend()
        plt.show()

    def visualize_all(self, truth_gdf: GeoDataFrame, input_gdf: GeoDataFrame, clusters: dict, selected_ids: list,
                      buffer_radius: float):
        """
        综合可视化：真值点、缓冲区、聚类结果和输入点筛选情况。
        """
        # 添加缓冲区
        truth_gdf["buffer"] = truth_gdf.geometry.buffer(buffer_radius)

        # 初始化聚类标签为 -1（未分配到任何簇的点）
        truth_gdf["cluster"] = -1

        # 遍历 clusters 字典，将每个簇的标签分配到 truth_gdf 的对应索引
        for root, indices in clusters.items():
            truth_gdf.loc[indices, "cluster"] = root

        # 标记输入点的筛选情况
        input_gdf["selected"] = input_gdf.index.isin(selected_ids)

        # 绘图
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))

        # 绘制未筛选的输入点
        input_gdf[~input_gdf["selected"]].plot(ax=ax, color="red", markersize=10, label="Unselected Input Points")

        # 绘制已筛选的输入点
        input_gdf[input_gdf["selected"]].plot(ax=ax, color="green", markersize=10, label="Selected Input Points")

        # 绘制真值点（按簇分颜色）
        truth_gdf.plot(ax=ax, column="cluster", categorical=True, legend=True, markersize=20, cmap="tab20",
                       label="Truth Clusters")

        # 绘制缓冲区
        truth_gdf["buffer"].plot(ax=ax, color="blue", alpha=0.2, label="Truth Buffers")

        # 图例和标题
        ax.set_title("Truth Points, Buffers, Clusters, and Input Points", fontsize=16)
        ax.legend()
        plt.show()

    def evaluate(self, buffer_distance) -> TrafficLightEvaluateResult:

        filtered = self.filter_input_trafficlights(self.input_trafficlight_gdf, self.input_trafficlight_gdf)

        match_result, unmatch_input_list = self.match_traffic_lights(
            true_traffic_light_point=self.truth_trafficlight_gdf["geometry"],
            input_traffic_light_point=filtered["geometry"],
            max_distance=buffer_distance
        )

        match_list = []
        for _, tpl in enumerate(match_result):
            p_truth, p_input = tpl
            match_list.append(PointPairEvaluationResult(truth_point=p_truth, input_point=p_input))

        res = TrafficLightEvaluateResult(
            truth_traffic_light_gdf=self.truth_trafficlight_gdf,
            input_traffic_light_gdf=self.input_trafficlight_gdf,
            unmatch_input_list=unmatch_input_list,
            matched_pair_list=match_list
        )

        return res

    def plot(self, result: TrafficLightEvaluateResult,
             title: str,
             is_to_4326: bool = True,
             is_visualize: bool = False,  # 调试时可选参数
             vector_file_path: Optional[str] = None):

        fig, ax = plt.subplots(figsize=(24, 18))  # 创建绘图窗口
        relation_line_gdf = result.to_gdf()
        unmatch_input_gdf = result.unmatch_input_gdf
        match_input_gdf = result.match_input_gdf

        # -------------------------------
        # 1. 坐标系转换
        # -------------------------------
        if is_to_4326:
            # algo_box = self.truth_intersection.algo_box.to_crs(epsg=4326)
            try:
                ll = self.truth_intersection.lane_lines.to_crs(epsg=4326)
                stop_lines = self.truth_intersection.stop_lines.to_crs(epsg=4326)
                crosswalks = self.truth_intersection.crosswalks.to_crs(epsg=4326)
                arrows = self.truth_intersection.arrows.to_crs(epsg=4326)
                truth_trafficlight_gdf = self.truth_trafficlight_gdf.to_crs(epsg=4326)
                relation_line_gdf = relation_line_gdf.to_crs(epsg=4326)
                unmatch_input_gdf = unmatch_input_gdf.to_crs(epsg=4326)
                match_input_gdf = match_input_gdf.to_crs(epsg=4326)
            except:
                pass
        else:
            algo_box = self.truth_intersection.algo_box
            try:
                ll = self.truth_intersection.lane_lines
                stop_lines = self.truth_intersection.stop_lines
                crosswalks = self.truth_intersection.crosswalks
                arrows = self.truth_intersection.arrows
                truth_trafficlight_gdf = self.truth_trafficlight_gdf
            except:
                pass

        # -------------------------------
        # 2. 绘制参考要素
        # -------------------------------
        # algo_box.plot(ax=ax, edgecolor='black', facecolor='none')
        try:
            stop_lines.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
            crosswalks.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
            arrows.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
            ll.plot(ax=ax, color='#0f0f0f80', alpha=0.5)
        except:
            pass

        # -------------------------------
        # 3. 绘制点要素
        # -------------------------------
        point_size = 50  # 调整点的大小，放大以便可见
        if not unmatch_input_gdf.empty:
            unmatch_input_gdf.plot(
                ax=ax,
                color='red',
                alpha=0.8,
                markersize=point_size,
                label='未匹配的信号灯输入值'
            )
        if not match_input_gdf.empty:
            match_input_gdf.plot(
                ax=ax,
                color='green',
                alpha=0.8,
                markersize=point_size,
                label='已匹配的信号灯输入值'
            )
        if not truth_trafficlight_gdf.empty:  # 绘制基准点
            truth_trafficlight_gdf.plot(
                ax=ax,
                color='blue',
                alpha=0.8,
                markersize=point_size,
                label='基准信号灯'
            )

        # -------------------------------
        # 4. 绘制关系线
        # -------------------------------
        relation_line_gdf.plot(ax=ax, linestyle='--', edgecolor='blue', label='匹配关系线')

        # -------------------------------
        # 5. 添加图例与标题
        # -------------------------------
        ax.legend(loc='upper right')  # 自动生成图例
        ax.set_title(title, fontsize=16)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("经度", fontsize=12)
        ax.set_ylabel("纬度", fontsize=12)
        plt.grid(True)

        # -------------------------------
        # 6. 添加统计信息
        # -------------------------------
        # stats_text = result.format_text
        stats_text = result.get_dict()
        stats_text_formatted = "\n".join([f"{key}: {value}" for key, value in stats_text.items()])
        logger.info(f"\n评测任务:{title} 结论:\n{stats_text_formatted}")  # 记录评测结论
        ax.text(1.01, 0.5, stats_text_formatted, transform=ax.transAxes, fontsize=10,
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # -------------------------------
        # 7. svg 保存与可视化
        # -------------------------------
        if vector_file_path:
            logger.debug(f"保存svg到 {vector_file_path}")
            plt.savefig(vector_file_path, format='svg', bbox_inches='tight')
            plt.close(fig)
        if is_visualize:
            plt.show()
            plt.close(fig)

    def save_as_gkpg(self, results: TrafficLightEvaluateResult, gpkg_path):
        """
        将一TrafficLightEvaluateResult保存为gpkg文件。

        :param results: MatchResult实例列表
        :param gpkg_path: 保存gpkg文件的路径
        """
        # self.truth_intersection.algo_box.to_file(gpkg_path, layer="算法框几何", driver="GPKG")
        self.truth_intersection.traffic_lights.to_crs(epsg=4326).to_file(gpkg_path, layer="真值信号灯", driver="GPKG")
        if not results.match_input_gdf.empty:
            results.match_input_gdf.to_crs(epsg=4326).to_file(gpkg_path, layer="已匹配输入信号灯", driver="GPKG")
        if not results.unmatch_input_gdf.empty:
            results.unmatch_input_gdf.to_crs(epsg=4326).to_file(gpkg_path, layer="未匹配输入信号灯", driver="GPKG")
        self.truth_intersection.lane_lines.to_crs(epsg=4326).to_file(gpkg_path, layer="真值车道线参考图层",
                                                                     driver="GPKG")
        results.to_gdf().to_file(gpkg_path, layer="配对关系与记录测量结果", driver="GPKG")
        logger.info(f"保存成功: {gpkg_path}")

# if __name__ == '__main__':
#     truth_intersection = load_this("./test_truth_intersection.pickle")
#     input_intersection = load_this("./test_input_intersection.pickle")
#     setting: EvaluatorParams = load_this("./setting.pickle")
#
#     te = TrafficlightEvaluator(truth_intersection=truth_intersection, input_intersection=input_intersection,
#                                evaluator_setting=setting)
#
#     filtered = te.filter_input_trafficlights(te.input_trafficlight_gdf, te.input_trafficlight_gdf)
#     #
#     match_result, unmatch_input_list = te.match_traffic_lights(
#         true_traffic_light_point=te.truth_trafficlight_gdf["geometry"],
#         input_traffic_light_point=filtered["geometry"])
#
#     # res2 = te.evaluate(5)
#     # print(res2.recall)
#     # print(res2.accuracy)
#
#     match_list = []
#     for _, tpl in enumerate(match_result):
#         p_truth, p_input = tpl
#         match_list.append(PointPairEvaluationResult(truth_point=p_truth, input_point=p_input))
#
#     res2 = TrafficLightEvaluateResult(
#         truth_traffic_light_gdf=te.truth_trafficlight_gdf,
#         input_traffic_light_gdf=te.input_trafficlight_gdf,
#         unmatch_input_list=unmatch_input_list,
#         matched_pair_list=match_list
#     )
#     te.save_svg(res2)
#
#     # fig, ax = plt.subplots(figsize=(24, 18))  # 创建绘图窗口
#     # try:
#     #     te.truth_trafficlight_gdf.plot(ax=ax, edgecolor='green', alpha=0.06)
#     #     filtered.plot(ax=ax, edgecolor='red', alpha=0.06)
#     #     # arrows.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)
#     #     res2.to_gdf().plot(ax=ax, edgecolor='blue', alpha=0.06)
#     #     plt.show()
#     #     plt.close(fig)
#     # except:
#     #     pass
