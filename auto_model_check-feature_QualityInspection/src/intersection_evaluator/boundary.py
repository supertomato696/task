# -*- coding: utf-8 -*-
# @Time    : 2025/3/5 15:48
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : boundary
from src.intersection_evaluator.init_intersection import Intersection
from src.intersection_evaluator.lane_line import LaneLineEvaluator
from src.logger import logger
from src.model.config import EvaluatorParams
from src.errors import DataEmpty


class RoadBoundaryEvaluator(LaneLineEvaluator):
    """
    道路边界评测类，

    :param truth_intersection: 真值路口对象。
    :param input_intersection: 带评测路口对象
    :param evaluator_setting: 评测工具配置。
    """

    def __init__(self, truth_intersection: Intersection, input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams):
        super().__init__(truth_intersection, input_intersection, evaluator_setting)

        self.truth_lines_gdf = self.truth_intersection.road_boundaries
        self.input_lines_gdf = self.input_intersection.road_boundaries
        logger.debug("道路边界评测类，初始化完毕")

class VectorizeOnlyRoadBoundaryEvaluator(LaneLineEvaluator):
    """
    仅矢量化道路边界评测类，用于仅矢量化道路边界的真值数据与输入数据。

    :param truth_intersection: 真值路口对象。
    :param input_intersection: 待评测路口对象
    :param evaluator_setting: 评测工具配置。
    """

    def __init__(self, truth_intersection: Intersection, input_intersection: Intersection,
                 evaluator_setting: EvaluatorParams):
        super().__init__(truth_intersection, input_intersection, evaluator_setting)

        self.truth_lines_gdf = self.truth_intersection.road_boundaries
        self.input_lines_gdf = self.input_intersection.road_boundary_vectorize

        if self.input_lines_gdf is None:
            raise DataEmpty("输入值未带有仅矢量化车道线数据。无法进行评测")
        if self.truth_lines_gdf is None:
            raise DataEmpty("真值值未带有仅矢量化车道线数据。无法进行评测")

        logger.debug("初始化完毕")
