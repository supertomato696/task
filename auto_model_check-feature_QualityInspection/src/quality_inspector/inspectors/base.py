from abc import ABC, abstractmethod
from enum import Enum
from pathlib import Path
from typing import Dict, Type

from geopandas import GeoDataFrame

from src.intersection_evaluator.init_intersection import Intersection


class BaseInspector(ABC):
    registry: Dict[str, Type['BaseInspector']] = {}
    alias: str = None

    @classmethod
    def register(cls, alias: str):
        def decorator(subcls: Type['BaseInspector']):
            if alias in cls.registry:
                raise KeyError(f"Alias: {alias} has been already registered.")
            cls.registry[alias] = subcls
            subcls.alias = alias
            return subcls
        return decorator

    def __init__(self, sd_link: GeoDataFrame, intersection: Intersection, data_dir: Path):
        # 待质检的目标
        self.intersection = intersection
        self.data_dir = data_dir
        # sd link对象
        self.sd_link = sd_link
        # 初始化必要属性
        self.feature_type = self.set_feature_type()
        self.level = self.set_level()
        self.result = {}

    @abstractmethod
    def set_feature_type(self):
        # 设置质检项的要素类型
        pass

    @abstractmethod
    def set_level(self):
        # 设置质检项等级
        pass

    @abstractmethod
    def inspect(self, *args, **kwargs):
        # 执行质检项，逻辑写这里
        pass

    @abstractmethod
    def get_result(self, *args, **kwargs):
        # 统计的相关逻辑
        pass


class InspectorLevel(Enum):
    P00 = 0
    P0 = 3
    P1 = 2
    P2 = 1


class InspectorFeatureType(Enum):
    line = 'line'
    point = 'point'
    polygon = 'polygon'
    relation = 'relation'
    others = 'others'
    # lane_lines = 'lane_lines'  # 车道线
    # lane_centerlines = 'lane_centerlines'  # 车道中心线
    # road_boundaries = 'road_boundaries' # 道路边界
    # crosswalks = 'crosswalks'  # 人行横道
    # stop_lines = 'stop_lines' # 停止线
    # traffic_lights = 'traffic_lights'  # 交通信号灯
    # arrows = 'arrows'  # 指示箭头
    # lanes = 'lanes'  # 车道
    # intersection_area = 'intersection_area'  # 路口面


class IssueType(Enum):
    intersection = '相交'
    capping = '压盖'
    suspend = '悬挂'
    connection = '前驱后继'
    zigzag = '折角'
    relation = '要素间关联关系'
    
