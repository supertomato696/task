# -*- coding: utf-8 -*-
# @Time    : 2025/6/6 15:15
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : task.py
from typing import Optional, List, Generator, Tuple

from geopandas import GeoDataFrame
from pydantic import BaseModel, field_validator, ConfigDict, Field
from shapely import MultiPolygon, wkt

from src.intersection_evaluator.init_intersection import IntersectionType
from src.model.config import EvaluateElementProfile
from src.quality_inspector.error import BadAlgoBoxGeom


class ElementDesc(BaseModel):
    type: IntersectionType = IntersectionType.input
    name: str
    version: str
    nickname: Optional[str] = None
    is_ignored_z: bool = True

    lane_lines: Optional[EvaluateElementProfile] = None
    road_boundaries: Optional[EvaluateElementProfile] = None
    lane_center: Optional[EvaluateElementProfile] = None
    crosswalks: Optional[EvaluateElementProfile] = None
    stop_lines: Optional[EvaluateElementProfile] = None
    arrows: Optional[EvaluateElementProfile] = None
    traffic_lights: Optional[EvaluateElementProfile] = None
    lane_boundary_vectorize: Optional[EvaluateElementProfile] = None
    road_boundary_vectorize: Optional[EvaluateElementProfile] = None
    lanes: Optional[EvaluateElementProfile] = None
    roads: Optional[EvaluateElementProfile] = None

    def get_element_fields(self) -> Generator[Tuple[str, EvaluateElementProfile], None, None]:
        """
        一个生成器函数，用于优雅地获取模型中所有非空的EvaluateElementProfile字段。

        Yields:
            一个元组 (字段名, 字段值)，其中字段值是 EvaluateElementProfile 的实例。
        """
        # 1. 遍历模型的字段定义 (model_fields 是一个字典 {field_name: FieldInfo})
        for field_name, field_info in self.model_fields.items():
            # 2. 检查字段的类型注解是否为我们想要的类型.为了简化，直接获取实例中的值并判断其类型
            field_value = getattr(self, field_name, None)

            # 3. 如果字段的值是 EvaluateElementProfile 的实例，则处理它
            if isinstance(field_value, EvaluateElementProfile):
                yield field_name, field_value


class QualitiInspectTask(BaseModel):
    # Part1 ======== 随着tasks.json传入
    expected_score: float = 0.8  # 及格分数
    check_integrity: bool = False  # 是否进行完整性强校验
    build_task_id: Optional[str] = 'UNKNOWN'  # 建图任务ID
    algo_box_id: str = Field(default=None, alias='frame_id')  # 算法框ID
    algo_box_geom: MultiPolygon = Field(alias='polygon')  # 算法框几何，实际传入WKT即可
    algo_version: Optional[str] = Field(default='UNKNOWN', alias='algorithm_version')  # 算法版本
    algorithm_result_directory: Optional[str] = None  # 待评测算法结果路径。默认在input下寻找符合正则规则的压缩包，也可指定
    inspect_items: List[str] = None  # 质检项列表

    # Part2 ======== 初始化时自动填充
    sd_link_gdf: GeoDataFrame = Field(default=GeoDataFrame())  # 默认空的Gdf
    intersection_config: Optional[ElementDesc] = None  # 路口要素描述
    model_config = ConfigDict(arbitrary_types_allowed=True)

    @field_validator("algo_box_geom", mode="before")
    def convert_to_geo_series(cls, value):
        """
        输入wkt转成geo_series。
        """
        try:
            return wkt.loads(value)
        except Exception:
            raise BadAlgoBoxGeom("Check input algobox")

    @field_validator("build_task_id", "algo_box_id", mode="before")
    def id_to_str(cls, value):
        return str(value)
