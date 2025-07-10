from typing import Optional, List, Literal

import yaml
from pydantic import BaseModel, field_validator, model_validator, validator, Field
from shapely import Polygon

from src.logger import logger
from src.tools.common import get_absolute_path
from src.tools.filter import FilterConditions


class Coord(BaseModel):
    lon: float
    lat: float


class Rect(BaseModel):
    left_upper: Coord
    right_bottom: Coord

    @property
    def geom(self):
        # TODO 判空
        return Polygon(
            [
                (self.left_upper.lon, self.left_upper.lat),  # 左上
                (self.right_bottom.lon, self.left_upper.lat),  # 右上
                (self.right_bottom.lon, self.right_bottom.lat),  # 右下
                (self.left_upper.lon, self.right_bottom.lat),  # 左下
            ]
        )

class EvaluateRectSetting(BaseModel):
    """设置全局的评测区域，这个区域的优先级高于具体的EvaluateArea"""
    is_limit_by_rects: bool = False
    rects: List[Rect]


class EvaluateArea(BaseModel):
    """
    定义评测区域的参数，包括评测的不同类型。

    :param name: 评测区域的名称。
    :param evaluate_laneline: 是否评测车道线。
    :param evaluate_boundary: 是否评测道路边界。
    :param evaluate_crosswalks: 是否评测人行横道。
    :param evaluate_landmarks: 是否评测地标。
    :param evaluate_stoplines: 是否评测停止线。
    """

    name: str
    evaluate_laneline: bool
    evaluate_boundary: bool
    evaluate_crosswalks: bool
    evaluate_landmarks: bool
    evaluate_stoplines: bool


class EvaluatorParams(BaseModel):
    """
    评测器参数配置，包括缓冲区长度和多边形匹配距离。
    :param resample_segment_length_meter: 线要素默认分段长度，单位米
    :param line_buffer_length_meter: 线段缓冲区长度，单位为厘米。
    :param polygon_match_distance: 多边形匹配的最大距离容忍值，单位米。
    :param lane_marking_match_distance_cm: 地面要素最大匹配距离cm
    :param traffic_light_match_distance_meter: 交通灯要素最大匹配距离，单位米。
    :param stopline_match_distance_meter: 停止线最大匹配距离，单位米。
    """
    resample_segment_length_meter: float = 1
    line_buffer_length_meter: float = 0.5
    polygon_match_distance_meter: float = 10
    land_marking_match_distance_meter: float = 1.5
    traffic_light_match_distance_meter: float = 5
    stopline_match_distance_meter: float = 1

    @model_validator(mode='before')
    def check_defaults_before(cls, data):
        """
        前置检查：在解析和转换字段之前检查原始输入数据，
        如果 'lane_marking_match_distance_cm' 未提供，则记录警告。

        :param data: 原始输入数据字典。
        :return: 原始输入数据，用于后续验证过程。
        """
        if 'resample_segment_length_meter' not in data:
            logger.warning(
                "resample_segment_length_meter线要素分段长度未提供，将使用默认值1米分段。"
            )
        if 'line_buffer_length_meter' not in data:
            logger.warning(
                "line_buffer_length_meter线要素评测缓冲区大小未提供，将使用默认值50cm。"
            )
        if 'polygon_match_distance_meter' not in data:
            logger.warning(
                "polygon_match_distance_meter停止线最大匹配距离未提供，将使用默认值10m。"
            )
        if 'land_marking_match_distance_meter' not in data:
            logger.warning(
                "land_marking_match_distance_meter地面标识最大匹配距离未提供，将使用默认值1.5m。"
            )
        if 'traffic_light_match_distance_meter' not in data:
            logger.warning(
                "traffic_light_match_distance_meter交通灯最大匹配距离未提供，将使用默认值5m。"
            )
        if 'stopline_match_distance_meter' not in data:
            logger.warning(
                "stopline_match_distance_meter停止线最大匹配距离未提供，将使用默认值1m。"
            )
        return data


class EvaluateTask(BaseModel):
    """
    定义评测区域的参数。

    :param name: 评测区域的名称。
    """

    name: str

    @field_validator("name", mode="before")
    def convert_to_str(cls, value):
        """
        如果输入的name 不是字符串，则转换为字符串
        """
        if not isinstance(value, str):
            return str(value)
        return value


class EvaluateElementProfile(BaseModel):
    """对于一种要素文件名、文件后缀、筛选条件的描述"""
    prefix: str  # 文件名
    suffix: Optional[str] = "shp"  # 后缀
    filter_conditions: Optional[FilterConditions] = None


class TruthFileProfile(BaseModel):
    """对于真值文件的描述"""
    lane_line: Optional[EvaluateElementProfile] = None
    road_boundary: Optional[EvaluateElementProfile] = None
    lane_centerline: Optional[EvaluateElementProfile] = None
    crosswalk: Optional[EvaluateElementProfile] = None
    stop_line: Optional[EvaluateElementProfile] = None
    lane_marking: Optional[EvaluateElementProfile] = None
    trafficlight: Optional[EvaluateElementProfile] = None
    intersection: Optional[EvaluateElementProfile] = None


class Version(BaseModel):
    """
    表示一个版本的信息，包括版本名称、标识及其要素文件前缀。

    :param version: 版本名称。
    :param nickname: 数据nickname标识，可以为数据源的区分、算法框的区分，也可以作为不同微调版本的区分标识。
    :param lane_line: 可选，用于如何识别车道线文件的信息。。
    :param road_boundary: 可选，用于如何识别道路边界文件的信息。
    :param crosswalk: 可选，用于如何识别人行横道文件的信息。
    :param stop_line: 可选，用于如何识别停止线文件的信息。
    :param lane_marking: 可选，用于如何识别车道标记文件的信息。
    :param trafficlight: 可选，用于如何识别信号灯文件的信息。
    lane_boundary_vectorize: 可选，用于如何识别仅矢量化车道边线的信息。
    road_boundary_vectorize: 可选，用于如何识别仅矢量化道路边线的信息。
    """

    version: str
    nickname: Optional[str] = None
    lane_line: Optional[EvaluateElementProfile] = None
    road_boundary: Optional[EvaluateElementProfile] = None
    lane_center: Optional[EvaluateElementProfile] = None
    crosswalk: Optional[EvaluateElementProfile] = None
    stop_line: Optional[EvaluateElementProfile] = None
    lane_marking: Optional[EvaluateElementProfile] = None
    trafficlight: Optional[EvaluateElementProfile] = None
    lane_boundary_vectorize: Optional[EvaluateElementProfile] = None
    road_boundary_vectorize: Optional[EvaluateElementProfile] = None

    @field_validator("version", "nickname", mode="before")
    def convert_to_str(cls, value):
        """
        如果输入的版本名称或nickname不是字符串，则转换为字符串。
        """
        if not isinstance(value, str):
            return str(value)
        return value


class DataPath(BaseModel):
    """
    存储数据路径信息，包括真实数据路径和输入数据路径。

    :param data_path: 数据路径。
    :param truth_data_path: 真实数据路径，用于验证。
    :param input_data_path: 输入数据路径。
    """

    data_path: str
    truth_data_path: str
    input_data_path: str
    output_data_path: str
    algorithm_box_data_path: str = "./data/algorithm_box"

    @field_validator("data_path", mode="before")
    def validate_datetime(cls, value):
        """
        在设置数据路径之前，将其转换为绝对路径。

        :param cls: 类自身（用于Pydantic的字段验证）。
        :param value: 输入的数据路径。
        :return: 转换后的绝对路径。
        :raises ValueError: 如果路径无效，则抛出异常。
        """
        try:
            return get_absolute_path(value)
        except (ValueError, TypeError):
            raise ValueError("无效路径")


class ToolConfig(BaseModel):
    """
    任务设置模型，包含可视化选项和评测参数。

    :param is_visualize: 是否启用可视化，默认为True。
    :param is_save_svg: 是否保存SVG文件，默认为True。
    :param params: 评测参数配置。
    :param truth_file_profile: 真值数据的形态。
    :param data_path: 数据路径的信息，由DataPath模型表示。

    # :param evaluate_areas: 包含评测区域的列表。
    """
    is_visualize: bool = True
    is_save_svg: bool = True
    evaluate_rect_setting: Optional[EvaluateRectSetting] = None
    params: EvaluatorParams
    # evaluate_areas: List[EvaluateArea]
    truth_file_profile: TruthFileProfile
    file_paths: DataPath


class Config(BaseModel):
    """
    """
    config: ToolConfig
    versions: List[Version]
    tasks: List[EvaluateTask]
    evaluation_actions: List[
        Literal[
            "lane_line_eval",
            "lane_line_offset_statistic",
            "vectorize_only_lane_line_eval",
            "vectorize_only_road_boundary_eval",
            "crosswalk_eval",
            "arrow_eval",
            "stopline_eval",
            "road_boundary_eval",
            "relative_width_eval",
            "traffic_light_eval",
        ]
    ] = Field(default=[
    ])

    class Config:
        arbitrary_types_allowed = True


if __name__ == '__main__':
    def load_config(file_path: str) -> Config:
        """ 加载工具配置 """
        with open(file_path, 'r', encoding='utf8') as file:
            config_data = yaml.safe_load(file)
            return Config(**config_data)


    a = load_config("../../task_setting_一号线路.yaml")
