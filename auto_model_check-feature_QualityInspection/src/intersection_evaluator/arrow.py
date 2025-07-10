# -*- coding: utf-8 -*-
# @Time    : 2025/3/13 13:42
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : arrow
import copy
import math
import os
from dataclasses import dataclass
from typing import Tuple, List, Dict

from geopandas import GeoDataFrame, GeoSeries
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
from pyproj import Transformer
from shapely import Polygon, LineString, Point
from shapely.ops import unary_union

from src.errors import DataEmpty
from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluatorParams

import matplotlib.patches as mpatches

from src.intersection_evaluator.crosswalk import PolygonPairEvaluationResult


class ArrowPairEvaluationResult(PolygonPairEvaluationResult):
    def __init__(self, iou: float, hausdorff_distance: float, centroid_distance: float, truth_arrow_tip_geom: Point,
                 input_arrow_tip_geom: Point, std_area=0, intersect_area=0, relation_geom: LineString = None,
                 arrow_tip_distance=float('inf')):
        super().__init__(iou, hausdorff_distance, centroid_distance, std_area, intersect_area, relation_geom,
                         arrow_tip_distance)
        self.truth_arrow_tip_geom = truth_arrow_tip_geom,
        self.input_arrow_tip_geom = input_arrow_tip_geom,


@dataclass
class ArrowEvaluateSummary:
    """
    Polygon校验的结果汇总
    Pydantic用累了换个口味...
    """
    title: str
    total_true_polygon_count: int
    total_input_polygon_count: int
    precision: float
    recall: float  # 召回率
    matched_pairs: list[Tuple[Polygon, Polygon]]
    unmatched_polygons: list[Polygon]
    evaluation_results: list[ArrowPairEvaluationResult]
    total_std_area: float  # 真值总面积
    total_intersection_area: float  # 总相交面积
    total_input_area: float  # 输入总面积
    intersect_percentage: float
    truth_polygon_count: int
    input_polygon_count: int
    
    @property
    def f1_score(self) -> float:
        """
        计算 F1 分数，综合考虑精确率和召回率。

        Returns:
            float: 计算得到的 F1 分数。如果精确率和召回率之和为 0，则返回 0。
        """
        return 2 * (self.precision * self.recall) / (self.precision + self.recall) if (self.precision + self.recall) > 0 else 0


    def get_dict(self) -> Dict:
        """
        获取评测结果的文本描述信息。

        Returns:
            Dict: 包含评测指标及相关信息的字典
        """
        # -------------------------------
        # 6. 添加统计信息
        # -------------------------------

        if len(self.evaluation_results):
            avg_hausdorff = sum([e.hausdorff_distance for e in self.evaluation_results]) / len(
                self.evaluation_results)
            avg_centroid_offset = sum([e.centroid_distance for e in self.evaluation_results]) / len(
                self.evaluation_results)
            # avg_iou = sum(e.iou for e in self.evaluation_results) / len(self.evaluation_results)
        else:
            avg_hausdorff = float('inf')
            avg_centroid_offset = float('inf')
            # avg_iou = float('inf')

        total_union_area = self.total_std_area + self.total_input_area - self.total_intersection_area
        IOU = self.total_intersection_area / total_union_area if (
                self.total_intersection_area != 0 and total_union_area != 0) else 0

        stats_dict = {
            "总真值面积(平米)": f"{self.total_std_area:.2f}",
            "参评输入面积(平米)": f"{self.total_input_area:.2f}",
            "相交面积(平米)": f"{self.total_intersection_area:.2f}",
            "准确率":  f"{self.precision * 100:.2f}%", # f"{len(self.matched_pairs) / self.input_polygon_count * 100:.2f}%",
            "召回率":  f"{self.recall * 100:.2f}%", # f"{len(self.matched_pairs) / self.truth_polygon_count * 100:.2f}%",
            "F1分数":  f"{self.f1_score:.2f}",
            "被匹配真值个数/输入个数/真值个数": f"{len(self.matched_pairs)} / {self.input_polygon_count} / {self.truth_polygon_count}",
            "整体IOU": f"{IOU:.4f}", 
            "整体平均H距离(米)": f"{avg_hausdorff:.4f}",
            "整体平均重心距离(米)": f"{avg_centroid_offset:.4f}"
        }
        return stats_dict


def create_evaluation_summary(title: str,
                              total_true_polygon_count: int,
                              total_input_polygon_count: int,
                              precision: float,
                              recall: float,
                              matched_pairs: List[Tuple[Polygon, Polygon]],
                              unmatched_polygons: List[Polygon],
                              evaluation_results: List[ArrowPairEvaluationResult],
                              total_std_area: float,
                              total_intersection_area: float,
                              total_input_area: float,
                              truth_polygon_count: int,
                              input_polygon_count: int
                              ) -> ArrowEvaluateSummary:
    """
    创建并返回一个 PolygonEvaluateSummary 实例。
    """
    summary = ArrowEvaluateSummary(
        title=title,
        total_true_polygon_count=total_true_polygon_count,
        total_input_polygon_count=total_input_polygon_count,
        precision=precision,
        recall=recall,
        matched_pairs=matched_pairs,
        unmatched_polygons=unmatched_polygons,
        evaluation_results=evaluation_results,
        total_std_area=total_std_area,
        total_intersection_area=total_intersection_area,
        total_input_area=total_input_area,
        intersect_percentage=(total_intersection_area / total_std_area * 100) if total_std_area else 0,
        truth_polygon_count=truth_polygon_count,
        input_polygon_count=input_polygon_count
    )
    return summary


class ArrowEvaluator:
    def __init__(self, truth_intersection: Intersection, input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams):
        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.tool_config = evaluator_setting
        self.truth_polygons_gdf = self.truth_intersection.arrows

        self.input_polygons_gdf = self.input_intersection.arrows

        if self.truth_polygons_gdf is None or self.input_polygons_gdf is None:
            raise DataEmpty(
                f"数据缺失: 真值非空{self.truth_polygons_gdf is not None}, 待评测值值非空{self.input_polygons_gdf is not None}")

        # 默认的crs同步自输入的数据
        self.standard_utm_crs = self.input_intersection.arrows.crs

        self.total_input_area = self.filtered_input_polygons["geometry"].area.sum()
        self.total_intersection_area = 0
        self.total_std_area = 0

        self.max_centroid_distance = self.tool_config.land_marking_match_distance_meter

        self.title = input_intersection.name


    def filter_input_polygons(self, truth_polygons_gdf: GeoDataFrame = None,
                              input_polygons_gdf: GeoDataFrame = None) -> GeoDataFrame:
        if not truth_polygons_gdf:
            truth_polygons_gdf = self.truth_polygons_gdf
        if not input_polygons_gdf:
            input_polygons_gdf = self.input_polygons_gdf

        gdf_truth = truth_polygons_gdf.copy().reset_index(drop=True)
        gdf_truth["buffer"] = gdf_truth.geometry.buffer(20)

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

        for det_idx, det_row in input_polygons_gdf.iterrows():
            det_geom = det_row.geometry
            # 检查输入多边形是否与任一聚类区域相交
            for cluster_geom in cluster_geoms.values():
                if det_geom.intersects(cluster_geom):
                    selected_detection_ids.append(det_idx)
                    break

        # 获取参与评测的输入多边形结果子集
        gdf_eval_input = input_polygons_gdf.loc[selected_detection_ids]

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
        self.total_intersection_area += intersection_area
        self.total_std_area += std_polygon.area
        union_area = std_polygon.union(test_polygon).area
        # union_area = std_polygon.area + test_polygon.area - intersection_area
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
        评测各项指标，计算IOU、Hausdorff距离、重心距离、等等

        returns:
        - PolygonPairEvaluationResult实例
        """
        # 计算各指标
        iou = self.calculate_iou(std_polygon, test_polygon)
        hausdorff = self.calculate_hausdorff_distance(std_polygon, test_polygon)
        centroid_dist = self.calculate_centroid_distance(std_polygon, test_polygon)

        # 增强计算：
        # 现在std_polygon是地面的箭头，箭头有方向，test_polygon是配对完毕之后的地面箭头。我们先认为他们的方向总是一致的。
        # 所以现在需要辨析std_polygon的方向。目前想到的就是，因为现在地面箭头做的是一个外接矩形，矩形肯定有短边。将短边的中点连接。即得到一条有向的矩形框中心线。
        # 在这条有向线的中点Mid做垂线。这条垂线的距离是10米。对于self.truth_intersection.lane_lines来说，两条有向的lane_line组成了一条车道。
        # 由于地面箭头必定在这个车道之内（有坑就认了），那么用这条垂线去找到相交的若干条lane_line，通过交点距离Mid的大小，即可筛选出与矩形框中心线距离最近的两条车道线。
        # 原则上来说这两个车道线都应该是同向的，那么他的方向也就应该是这个矩形框的方向，如果矩形框中心线与这个方向相同，那么不用调整，否则调整矩形框中心线的方向。
        # 至此，真值地面箭头的方向已经通过与它毗邻的车道线找到。箭头的顶点我们认为是矩形框中心线的终点。

        # 下一步，我们需要判断输入箭头语义上的顶点。
        # 同样，输入的地面箭头也是一个外接矩形。那么也就说明，纵使我们看不到实际的箭头，它语义上的顶点必定也就在两个短边的中点上，现在只是要确定方向。
        # 但是上一步我们已经找到了方向，我们总是假定匹配到的输入箭头方向一定跟真值方向是一致的，通过真值地面箭头的矩形框中心线的方向，我们就可以在两个短边顶点中，
        # 找到语义上的箭头顶点。

        # 再下一步，我们需要计算这真值箭头语义上的顶点和输入箭头语义上的顶点的距离。并记录下来。

        # 1. 对真值多边形，计算经过车道线方向校正后的中心线及其终点（语义箭头顶点）
        std_centerline, std_arrow_tip = self.get_std_polygon_centerline(std_polygon)
        std_direction = self.get_line_direction(std_centerline)

        # 2. 对测试多边形，根据真值中心线方向参考，确定其语义箭头顶点
        test_arrow_tip = self.get_test_polygon_arrow_tip(test_polygon, ref_direction=std_direction)

        try:
            arrow_tip_distance = std_arrow_tip.distance(test_arrow_tip)
        except Exception as err:
            logger.warning("语义顶点距离计算失败，arrow_top_distance 置为 None: %s", err)
            arrow_tip_distance = None

        # 构造关系线，后面把统计信息附在这条线上
        centroid_std = std_polygon.centroid
        centroid_test = test_polygon.centroid
        relation_line_geom = LineString([centroid_std, centroid_test])

        # 返回评测结果实例
        return ArrowPairEvaluationResult(iou=iou,
                                         hausdorff_distance=hausdorff, centroid_distance=centroid_dist,
                                         truth_arrow_tip_geom=std_arrow_tip,
                                         input_arrow_tip_geom=test_arrow_tip,
                                         arrow_tip_distance=arrow_tip_distance,  # 箭头尖尖的距离
                                         std_area=std_polygon.area,
                                         relation_geom=relation_line_geom,
                                         intersect_area=std_polygon.intersection(test_polygon).area
                                         )

    @staticmethod
    def match_polygons(true_polygons: GeoSeries, input_polygons: GeoSeries,
                       max_distance: float = 5) -> tuple[list[tuple[Polygon, Polygon]], list[Polygon]]:
        """
        二维数组，以真值为基础找匹配
        1. 距离真值多边形质心距离超过5米且与真值多边形没有任何交集的输入多边形视为无法匹配；
        2. 如果有多个多边形满足质心距离在5米内，那么覆盖度高的视为匹配。
        3. 如果覆盖度相同，那么质心距离最近的视为匹配。
        4. 如果覆盖度和质心距离最接近，那么任选一个视为匹配。
        5. 一个真值多边形只会被一个输入多边形匹配。也就是一个输入多边形只能被一个真值匹配成组。
        :param true_polygons: 真值多边形
        :param input_polygons:
        :param max_distance: 计算匹配的质心最大距离默认5米
        :return:
        """
        matched: dict[Polygon: Polygon] = {}

        unmatched_true_polygons = copy.deepcopy(input_polygons.tolist())

        for true_poly in true_polygons:
            # 候选列表
            candidates = []
            for input_poly in unmatched_true_polygons:
                true_centroid = true_poly.centroid
                input_centroid = input_poly.centroid
                distance = true_centroid.distance(input_centroid)
                if distance <= max_distance or true_poly.intersects(input_poly):
                    coverage = true_poly.intersection(input_poly).area / true_poly.area
                    candidates.append((input_poly, coverage, distance, input_centroid))

            if candidates:
                candidates.sort(key=lambda x: (-x[1], x[2]))
                matched_poly = candidates[0][0]
                matched[true_poly] = matched_poly
                unmatched_true_polygons.remove(matched_poly)

        # 筛选出未匹配的输入多边形
        # matched将字典中的值收集到一个集合中，用于快速查找
        dict_values_set = set(matched.values())
        # 过滤出不在字典值集合中的polygon
        unmatched_input_polygons = [value for value in input_polygons if value not in dict_values_set]

        return list(matched.items()), unmatched_input_polygons

    def evaluate(self) -> ArrowEvaluateSummary:
        """
        执行多边形匹配与评测，返回所有评测结果和平均分

        returns:
        - matched_pairs: List of tuples (真值多边形, 测试多边形)
        - evaluations: List of EvaluationResult 实例
        """
        matched_pairs, unmatched_list = self.match_polygons(
            true_polygons=self.truth_polygons_gdf["geometry"],
            input_polygons=self.filtered_input_polygons["geometry"],
            max_distance=self.max_centroid_distance)

        evaluation_results = []
        logger.debug("\n开始进行综合评测...")
        for idx, (std_poly, test_poly) in enumerate(matched_pairs, start=1):
            eval_result = self.evaluate_metrics(std_poly, test_poly)
            logger.debug(f"评测多边形对 {idx}:{eval_result}")
            evaluation_results.append(eval_result)

        return create_evaluation_summary(
            title=self.title,
            precision=len(matched_pairs) / self.filtered_input_polygons.shape[0],
            recall=len(matched_pairs) / self.truth_polygons_gdf.shape[0],# len(self.truth_polygons_gdf["geometry"]),
            total_true_polygon_count=len(self.input_polygons_gdf["geometry"]),
            total_input_polygon_count=len(self.filtered_input_polygons["geometry"]),
            matched_pairs=matched_pairs,
            unmatched_polygons=unmatched_list,
            evaluation_results=evaluation_results,
            total_std_area=self.total_std_area,
            total_input_area=self.total_input_area,
            total_intersection_area=self.total_intersection_area,
            truth_polygon_count=self.truth_polygons_gdf.shape[0],
            input_polygon_count=self.filtered_input_polygons.shape[0],
        )

    def save_as_gkpg(self, results: ArrowEvaluateSummary, gpkg_path):

        # 确保目标目录存在
        output_dir = os.path.dirname(gpkg_path)
        os.makedirs(output_dir, exist_ok=True)

        # 将结果组织为gdf，匹配pair的匹配情况附着在relation_geom线上。
        # 假设 results 是包含 PolygonPairEvaluationResult 对象的列表
        data = []
        res_list: list[ArrowPairEvaluationResult] = results.evaluation_results
        for res in res_list:
            # 如果某些结果的relation_geom有可能为 None，此处可做检查
            if res.relation_geom is None:
                continue
            data.append({
                "geometry": res.relation_geom,
                "tip_distance": res.arrow_tip_distance,
                "centroid_distance": res.centroid_distance,
                "iou": res.iou
            })

        # 生成GeoDataFrame其中"geometry"列作为几何列,同时设置crs
        gdf = GeoDataFrame(data, geometry="geometry", crs=self.standard_utm_crs)

        # self.standard_utm_crs现在应该是32650，这里统一做个转换
        gdf = gdf.to_crs(epsg=4326)
        # FIXME 不知道转了之后会不会影响原来的crs，待定。不过现在评测完一轮之后原始数据应该不会再用了影响不大。

        gdf.to_file(gpkg_path, layer="配对关系与独立评测结果", driver="GPKG")

        self.truth_intersection.arrows.to_crs(epsg=4326).to_file(gpkg_path, layer="真值地面箭头", driver="GPKG")
        self.input_intersection.arrows.to_crs(epsg=4326).to_file(gpkg_path, layer="输入地面箭头", driver="GPKG")
        self.truth_intersection.lane_lines.to_crs(epsg=4326).to_file(gpkg_path, layer="真值车道线参考图层",
                                                                     driver="GPKG")
        self.truth_intersection.stop_lines.to_crs(epsg=4326).to_file(gpkg_path, layer="真值停止线参考图层",
                                                                     driver="GPKG")

        logger.debug("地面箭头写gpkg搞定")

    # raise NotImplementedError("人行横道咋没有保存gpkg的方法？")

    def save_svg(self, title: str, results: ArrowEvaluateSummary,
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
            crosswalks = self.truth_intersection.crosswalks.to_crs(epsg=4326)
            lane_lines = self.truth_intersection.lane_lines.to_crs(epsg=4326)
            road_boundaries = self.truth_intersection.road_boundaries.to_crs(epsg=4326)
            truth_arrows = self.truth_polygons_gdf.to_crs(epsg=4326)
            input_arrows = self.input_polygons_gdf.to_crs(epsg=4326)
            filtered_arrows = self.filtered_input_polygons.to_crs(epsg=4326)
        else:
            stop_lines = self.truth_intersection.stop_lines
            crosswalks = self.truth_intersection.crosswalks
            lane_lines = self.truth_intersection.lane_lines
            road_boundaries = self.truth_intersection.road_boundaries
            truth_arrows = self.truth_polygons_gdf
            input_arrows = self.input_polygons_gdf
            filtered_arrows = self.filtered_input_polygons

        # -------------------------------
        # 2. 绘制参考要素：停止线、车道线、路界和人行横道
        # -------------------------------
        for feature in [stop_lines, lane_lines, road_boundaries, crosswalks]:
            feature.plot(ax=ax, edgecolor='#0f0f0f80', alpha=0.06)

        # -------------------------------
        # 3. 绘制真值箭头
        # -------------------------------
        truth_arrows.plot(ax=ax, alpha=0.8, color='#1b315e')

        # -------------------------------
        # 4. 绘制输入的箭头（分匹配和未匹配）
        # -------------------------------
        matched_inputs = []
        unmatched_inputs = []
        orphan_inputs = []

        # 遍历 input_crosswalks.geometry，假设 results.unmatched_polygons 中存储的均为 Shapely 几何对象
        for input_poly in filtered_arrows.geometry:
            if input_poly in results.unmatched_polygons:
                unmatched_inputs.append(input_poly)
            else:
                matched_inputs.append(input_poly)

        for input_cs in input_arrows.geometry:
            if (input_cs not in matched_inputs) or (input_cs not in unmatched_inputs):
                orphan_inputs.append(input_cs)

        # 绘制已匹配的输入地面箭头
        if matched_inputs:
            GeoSeries(matched_inputs, crs=input_arrows.crs) \
                .plot(ax=ax, facecolor='green', edgecolor='none', alpha=0.5)
        # 绘制未匹配的输入地面箭头
        if unmatched_inputs:
            GeoSeries(unmatched_inputs, crs=input_arrows.crs) \
                .plot(ax=ax, facecolor='#c63c26', edgecolor='none', alpha=0.9)

        # 绘制未参与匹配输入地面箭头
        if orphan_inputs:
            GeoSeries(orphan_inputs, crs=input_arrows.crs) \
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

        # if len(results.evaluation_results):
        #     avg_hausdorff = sum([e.hausdorff_distance for e in results.evaluation_results]) / len(
        #         results.evaluation_results)
        #     avg_centroid_offset = sum([e.centroid_distance for e in results.evaluation_results]) / len(
        #         results.evaluation_results)
        # else:
        #     avg_hausdorff = float('inf')
        #     avg_centroid_offset = float('inf')

        # total_union_area = results.total_std_area + sum(polygon.area for polygon in results.unmatched_polygons)
        # IOU = results.total_intersection_area / total_union_area if (
        #         results.total_intersection_area != 0 and total_union_area != 0) else 0

        # stats_text = (
        #     f"总基准面积{results.total_std_area:.2f}平米\n"
        #     f"相交面积{results.total_intersection_area:.2f}平米\n"
        #     f"输入面积准确率={results.total_intersection_area / results.total_input_area * 100:.2f}%\n"
        #     f"输入面积召回率={results.intersect_percentage:.2f}%\n"
        #     f"输入个数/真值个数：{results.input_polygon_count}/{results.truth_polygon_count}\n"
        #     f"整体IOU: {IOU:.4f}\n"
        #     f"整体平均H距离: {avg_hausdorff:.4f}米\n"
        #     f"整体平均重心距离: {avg_centroid_offset:.4f}米\n"
        # )

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
    def filtered_input_polygons(self) -> GeoDataFrame:
        return self.filter_input_polygons()

    def get_line_direction(self, line: LineString) -> float:
        """
        根据 LineString 的两个端点计算出方向角（弧度），返回第一点指向第二点的方向
        """
        coords = list(line.coords)
        if len(coords) < 2:
            raise ValueError("LineString 顶点数不足以计算方向")
        dx = coords[1][0] - coords[0][0]
        dy = coords[1][1] - coords[0][1]
        return math.atan2(dy, dx)

    def get_std_polygon_centerline(self, polygon: Polygon) -> Tuple[LineString, Point]:
        """
        针对真值多边形（已为四边形）计算中心线，并利用车道线数据校正方向，
        最终输出校正后的中心线及其终点（即语义箭头顶点）。

        实现步骤：
          1. 从 polygon 中直接读取四个顶点（不重复最后一个），按照顺序排列
          2. 计算四条边长度，取最短的两条边（真值箭头必为矩形）
          3. 计算这两条短边的中点，构造中心线（即两中点连线）
          4. 计算中心线方向，并在中心线上取中点作为基准点
          5. 根据 truth_intersection.lane_lines（GeoDataFrame）中车道线，
             选取基准点 mid_center 最近的两条车道线，通过其前两个点计算方向，取平均得到参考方向
          6. 比较中心线原始方向与参考方向，如果夹角大于 90°（余弦小于0），则反转中心线方向
          7. 返回校正后的中心线及其终点作为真值箭头语义顶点
        """
        try:
            # 1. 提取四边形顶点（去除重复的最后一点）
            coords = list(polygon.exterior.coords)[:-1]
            if len(coords) != 4:
                # raise ValueError(f"输入多边形顶点数不为4，而为 {len(coords)}")

                transformer = Transformer.from_crs("EPSG:32650", "EPSG:4326", always_xy=True)
                polygon_4326 = [transformer.transform(x, y) for x, y in coords]
                logger.warning(f"输入多边形顶点数不为4，而为 {len(coords)}. 坐标{polygon_4326}")

            # 2. 计算四条边长度
            edges = []
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i + 1) % 4]
                length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                edges.append((i, length))

            # 3. 选取两条最短的边，顺序排列即为它们在四边形中的位置
            sorted_edges = sorted(edges, key=lambda item: item[1])
            short_edge_indices = [sorted_edges[0][0], sorted_edges[1][0]]

            # 4. 计算两条短边的中点
            midpoints = []
            for idx in short_edge_indices:
                p1 = coords[idx]
                p2 = coords[(idx + 1) % 4]
                mid_x = (p1[0] + p2[0]) / 2.0
                mid_y = (p1[1] + p2[1]) / 2.0
                midpoints.append(Point(mid_x, mid_y))
            if len(midpoints) != 2:
                raise ValueError("计算短边中点失败，数量不足")

            # 5. 构造初步中心线（默认顺序为 midpoints[0] -> midpoints[1]）
            center_line = LineString([midpoints[0], midpoints[1]])
            center_angle = self.get_line_direction(center_line)
            # 基准点，用于比较车道线距离：取 center_line 的中点
            mid_center = center_line.interpolate(0.5, normalized=True)

            # 6. 利用 truth_intersection.lane_lines 中的车道线校正方向
            lane_gdf = self.truth_intersection.lane_lines.copy()
            if lane_gdf.empty:
                logger.info("车道线 GeoDataFrame 为空，略过方向校正")
            else:
                # 计算 mid_center 到每条车道线的距离
                lane_gdf["mid_distance"] = lane_gdf.geometry.apply(lambda geom: mid_center.distance(geom))
                lane_gdf_sorted = lane_gdf.sort_values("mid_distance")
                selected_lanes = []
                if len(lane_gdf_sorted) >= 2:
                    selected_lanes = lane_gdf_sorted.geometry.tolist()[:2]
                else:
                    logger.info("车道线数量不足2条，仅使用现有车道线进行方向参考")
                    selected_lanes = lane_gdf_sorted.geometry.tolist()

                lane_angles = []
                for lane in selected_lanes:
                    lane_coords = list(lane.coords)
                    if len(lane_coords) >= 2:
                        # 取前两个点计算方向
                        angle = math.atan2(lane_coords[1][1] - lane_coords[0][1],
                                           lane_coords[1][0] - lane_coords[0][0])
                        lane_angles.append(angle)
                if lane_angles:
                    avg_lane_angle = sum(lane_angles) / len(lane_angles)
                    # 7. 若中心线与车道线参考方向夹角大于90°，则反转中心线
                    if math.cos(center_angle - avg_lane_angle) < 0:
                        center_line = LineString(list(center_line.coords)[::-1])
                        center_angle = self.get_line_direction(center_line)
                        logger.info("真值中心线方向与车道线参考方向不一致，已反转中心线")

            # 8. 选取 center_line 的终点作为真值箭头语义顶点
            arrow_tip = Point(list(center_line.coords)[1])
            return center_line, arrow_tip

        except Exception as ex:
            logger.exception("计算真值箭头中心线失败: %s", ex)
            raise

    def get_test_polygon_arrow_tip(self, polygon: Polygon, ref_direction: float) -> Point:
        """
        针对输入的多边形，利用之前真值箭头得到的参考方向，
        通过两条短边中点构造候选中心线，选择方向与参考方向一致的一端作为箭头顶点。

        实现步骤：
          1. 从 polygon 中取出四个顶点（四边形）
          2. 计算四条边长度，选取两条最短边，计算各自中点
          3. 构造初步中心线（顺序暂定为 midpoints[0] -> midpoints[1]）
          4. 计算该中心线方向，取中点 mid_center
          5. 分别计算 center_line 两个端点与 mid_center 的向量，与参考方向构成的余弦，
             选择余弦值较大的端点作为测试箭头的语义顶点
        """
        try:
            # 1. 提取四边形顶点（去除重复的最后一点）
            coords = list(polygon.exterior.coords)[:-1]
            if len(coords) != 4:
                # raise ValueError(f"输入多边形顶点数不为4，而为 {len(coords)}")
                logger.warning(f"输入多边形顶点数不为4，而为 {len(coords)}. 坐标{coords}")

            # 2. 计算四条边长度
            edges = []
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i + 1) % 4]
                length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                edges.append((i, length))

            sorted_edges = sorted(edges, key=lambda item: item[1])
            short_edge_indices = [sorted_edges[0][0], sorted_edges[1][0]]

            # 计算这两条短边的中点
            midpoints = []
            for idx in short_edge_indices:
                p1 = coords[idx]
                p2 = coords[(idx + 1) % 4]
                midpoints.append(Point((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0))
            if len(midpoints) != 2:
                raise ValueError("计算测试多边形短边中点失败")

            # 构造初步中心线
            center_line = LineString([midpoints[0], midpoints[1]])
            # 计算中心线方向（初步）
            test_center_angle = self.get_line_direction(center_line)
            mid_center = center_line.interpolate(0.5, normalized=True)

            # 分别计算两端点相对于 mid_center 的向量并与参考方向计算余弦
            p0 = Point(list(center_line.coords)[0])
            p1 = Point(list(center_line.coords)[1])
            vec0 = (p0.x - mid_center.x, p0.y - mid_center.y)
            vec1 = (p1.x - mid_center.x, p1.y - mid_center.y)
            ref_vec = (math.cos(ref_direction), math.sin(ref_direction))

            # 归一化向量，防止夹角计算受到向量长度影响
            def norm(vec):
                return math.hypot(vec[0], vec[1])

            cos0 = (vec0[0] * ref_vec[0] + vec0[1] * ref_vec[1]) / (norm(vec0) or 1)
            cos1 = (vec1[0] * ref_vec[0] + vec1[1] * ref_vec[1]) / (norm(vec1) or 1)
            # 选择余弦值较大的，即方向和参考方向更一致的端点
            arrow_tip = p0 if cos0 > cos1 else p1
            return arrow_tip

        except Exception as ex:
            logger.exception("计算测试箭头语义顶点失败: %s", ex)
            raise Exception("计算测试箭头语义顶点失败: %s", ex)
