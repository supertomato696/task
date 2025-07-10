import os
from enum import Enum
from pathlib import Path
from typing import Union, Optional, Dict

import geojson
from geojson import Polygon
from geopandas import GeoDataFrame, GeoSeries
from pydantic import BaseModel, ConfigDict
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from src.data_process.file_read import DataReader
from src.logger import logger
from src.model.config import ToolConfig, EvaluateElementProfile, Version
from src.tools.filter import GeoDataFrameFilter


def _transform_polygon(
        exact_area: Polygon,
        target_crs: Optional[str],
        polygon_crs: Optional[str] = None) -> Polygon:
    """
    根据给定的polygon_crs将Polygon投影到目标 CRS；
    若未指定polygon_crs，默认认为输入Polygon为 EPSG:4326。
    """
    # 默认polygon CRS为EPSG:4326
    effective_polygon_crs = polygon_crs or "EPSG:4326"

    if effective_polygon_crs == target_crs:
        return exact_area

    # 用 GeoSeries 包装 Polygon，并指定原始 CRS，再转换到目标 CRS
    poly_series = GeoSeries([exact_area], crs=effective_polygon_crs)
    poly_converted = poly_series.to_crs(target_crs)
    return poly_converted.iloc[0]


def prune_gdf(input_gdf: GeoDataFrame, exact_area: Polygon) -> GeoDataFrame:
    """
    根据exact_area裁剪输入的GeoDataFrame，只保留与该区域有交集的要素。
    针对线要素使用intersection进行裁剪，
    面要素则只判断交集存在时保留原几何，
    其它类型统一使用intersection裁剪。
    """
    pruned_rows = []

    for idx, row in input_gdf.iterrows():
        geom: BaseGeometry = row.geometry

        if geom is None or not geom.intersects(exact_area):
            continue

        geom_type = geom.geom_type
        new_geom = None

        if geom_type in ["LineString", "MultiLineString"]:
            new_geom = geom.intersection(exact_area)
            if new_geom.is_empty:
                continue
        elif geom_type in ["Polygon", "MultiPolygon"]:
            new_geom = geom  # 保留原始面要素
        else:
            new_geom = geom.intersection(exact_area)
            if new_geom.is_empty:
                continue

        new_row = row.copy()
        new_row.geometry = new_geom
        pruned_rows.append(new_row)

    pruned_gdf = GeoDataFrame(pruned_rows, crs=input_gdf.crs)
    return pruned_gdf


class IntersectionType(Enum):
    standard = 1
    input = 2


class Intersection(BaseModel):
    """
    路口类，使用Pydantic模型定义。

    属性:
        type: 路口的类型，用来区分是真值还是待评测的
        name: 名称，一般约定是算法框id，用于在评测任务重透传相关信息
        version: 可选的关联版本号，主要给input用的，用于在评测任务重透传相关信息
        nickname: 可选的特殊标识，主要给input用的，用于在评测任务重透传相关信息

        name: 路口的名称。
        lane_lines: 可选的车道线 GeoDataFrame。
        lane_centerlines: 可选的车道中心线 GeoDataFrame。
        road_boundaries: 可选的道路边界 GeoDataFrame。
        crosswalks: 可选的人行横道 GeoDataFrame。
        stop_lines: 可选的停止线 GeoDataFrame。
        traffic_lights: 可选的交通信号灯 GeoDataFrame。
        arrows: 可选的指示箭头 GeoDataFrame。
        junctions: 可选的交叉口 GeoDataFrame。
        lanes: 可选的车道 GeoDataFrame。
        intersection_area: 可选的车道面

        lane_boundary_vectorize: 可选的仅矢量化车道线。不久的将来会被移除。
        road_boundary_vectorize: 可选的仅矢量化道路边界。不久的将来会被移除。

    所有属性都是可选的，因为在某些情况下，路口可能缺少某些要素信息。
    """
    type: IntersectionType  # 路口的类型，用来区分是真值还是待评测的
    name: str  # 名称，一般约定是算法框id，用于在评测任务重透传相关信息
    version: Optional[str] = None  # 关联版本号，主要给input用的，用于在评测任务重透传相关信息
    nickname: Optional[str] = None  # 特殊标识，主要给input用的，用于在评测任务重透传相关信息

    __algo_box_geom: Optional[Polygon] = None  # 算法框几何

    is_ignored_z: bool  # 是否包含高程。必填项，防止加了高程和没加高程的数据混用

    lane_lines: GeoDataFrame = GeoDataFrame()  # 车道线
    lane_centerlines: GeoDataFrame = GeoDataFrame()  # 车道中心线
    road_boundaries: GeoDataFrame = GeoDataFrame()  # 道路边界
    crosswalks: GeoDataFrame = GeoDataFrame()  # 人行横道
    stop_lines: GeoDataFrame = GeoDataFrame()  # 停止线
    traffic_lights: GeoDataFrame = GeoDataFrame()  # 交通信号灯
    arrows: GeoDataFrame = GeoDataFrame()  # 指示箭头
    lanes: GeoDataFrame = GeoDataFrame()  # 车道面
    roads: GeoDataFrame = GeoDataFrame()  # 道路面
    intersection_area: GeoDataFrame = GeoDataFrame()  # 路口面

    # 兼容road_boundary_vectorize等数据类型
    lane_boundary_vectorize: GeoDataFrame = GeoDataFrame()  # 仅矢量化车道线
    road_boundary_vectorize: GeoDataFrame = GeoDataFrame()  # 仅矢量化道路边界

    model_config = ConfigDict(arbitrary_types_allowed=True)

    @property
    def boundary_convex_hull(self):
        """
        0411临时方案，返回车道线和边界组成的凸包
        """
        # fixme 临时方案，默认有车道线和边界
        hull1 = self.lane_lines.union_all().convex_hull
        hull2 = self.road_boundaries.union_all().convex_hull

        merged_hulls = unary_union([hull1, hull2])

        return merged_hulls


    @property
    def convex_hull(self):
        """
        0411临时方案，返回车道线、边界、停止线、人行横道组成的凸包
        """
        hull_list = []

        if not self.lane_lines.empty:
            hull_list.append(self.lane_lines.union_all().convex_hull)

        if not self.road_boundaries.empty:
            hull_list.append(self.road_boundaries.union_all().convex_hull)

        if not self.stop_lines.empty:
            hull_list.append(self.stop_lines.union_all().convex_hull.buffer(1))  # 多往外找一米

        if not self.crosswalks.empty:
            hull_list.append(self.crosswalks.union_all().convex_hull.buffer(1))  # 多往外找一米


        merged_hulls = unary_union(hull_list)

        # # 创建 GeoDataFrame
        # gdf = GeoDataFrame({'geometry': [merged_hulls]}, crs='EPSG:32650')
        #
        # # 保存为 GeoJSON 文件
        # gdf_transformed = gdf.to_crs('EPSG:4326').to_file('transformed_polygon.geojson', driver='GeoJSON')
        # logger.debug("transformed_polygon.geojson.geojson")

        return merged_hulls

    def set_algo_box_geom(self, algo_box_geom: Polygon):
        """设置关联算法框几何"""
        self.__algo_box_geom = algo_box_geom

    @property
    def algo_box(self) -> GeoDataFrame:
        """任务框GDF"""
        if self.__algo_box_geom:
            # 设置为车道线的crs
            res = GeoDataFrame(geometry=[self.__algo_box_geom], crs=4326)
            return res
        else:
            return GeoDataFrame(columns=['geometry'], crs="EPSG:4326")

    def prune_by(self, exact_area: Polygon, polygon_crs: Optional[str] = "EPSG:4326") -> "Intersection":
        """
        动态遍历所有属性，如果属性是 GeoDataFrame，则进行裁剪处理，
        只保留与 exact_area 相交的几何信息。

        参数:
            exact_area: 用来裁剪的多边形 (Polygon)
            polygon_crs: 指定输入 Polygon 的 CRS，默认为 "EPSG:4326"，
                         若输入 Polygon 所使用的 CRS 与此不一致，请明确传入正确的 CRS。
        返回:
            一个新的 Intersection 对象，其所有 GeoDataFrame 类型的属性均经过裁剪处理。
        """
        logger.info(f"{self.type}的ID={self.name}路口正在接受裁剪。")
        data = self.model_dump()

        # 动态处理所有GeoDataFrame类型的属性
        for key, value in data.items():
            if isinstance(value, GeoDataFrame):
                if value.empty:
                    logger.warning(f"{self.type}的ID={self.name}路口属性`{key}`没有有效值,直接设置为空GeoDataframe。下一位", exc_info=True)
                    data[key] = value
                    continue
                try:
                    target_crs = value.crs
                    transformed_area = _transform_polygon(exact_area, target_crs, polygon_crs)
                    pruned_value = prune_gdf(value, transformed_area)
                    data[key] = pruned_value
                    logger.debug(f"{self.type}的{self.name}属性`{key}`剪裁完成。剪裁后/前各：{pruned_value.shape[0]}/{value.shape[0]}项", exc_info=True)
                except Exception as e:
                    logger.error(f"裁剪属性 `{key}` 时出错: {type(e)}{type(e)}{e}", exc_info=True)
                    data[key] = value

        return Intersection(**data)

    def to_gpkg(self, output_path: str, layer_suffix: str = None):
        """
        将路口对象转换为 GeoPackage 文件。

        Args:
            output_path: 输出 GeoPackage 文件的路径。
            layer_suffix: 可选的图层名称后缀，用于区分不同版本或来源的数据。
        """

        # 创建一个字典来存储非空的 GeoDataFrame 及其对应的图层名称
        layers: Dict[str, GeoDataFrame] = {}
        if self.lane_lines is not None:
            layers["lane_lines"] = self.lane_lines
        if self.lane_centerlines is not None:
            layers["lane_centerlines"] = self.lane_centerlines
        if self.road_boundaries is not None:
            layers["road_boundaries"] = self.road_boundaries
        if self.crosswalks is not None:
            layers["crosswalks"] = self.crosswalks
        if self.stop_lines is not None:
            layers["stop_lines"] = self.stop_lines
        if self.traffic_lights is not None:
            layers["traffic_lights"] = self.traffic_lights
        if self.arrows is not None:
            layers["arrows"] = self.arrows
        if self.junctions is not None:
            layers["junctions"] = self.junctions
        if self.lanes is not None:
            layers["lanes"] = self.lanes

        # 确保输出路径是一个 Path 对象
        output_path_obj = Path(output_path)

        # 如果输出文件已经存在，先删除它
        if output_path_obj.exists():
            output_path_obj.unlink()

        # 自定义图层顺序将'lanes'和'junctions'放在特定位置，其他图层按字母顺序
        # 获取除了'lanes'和'junctions'之外的所有键,对其进行排序
        other_layers_keys = sorted([key for key in layers.keys() if key not in ['lanes', 'junctions']])

        # 创建新的图层顺序列表
        ordered_layers_keys = []

        # 在这里做一下判断，避免出现keyError
        if 'lanes' in layers:
            ordered_layers_keys.append('lanes')  # 'lanes' 总是在最底层
        if 'junctions' in layers:
            ordered_layers_keys.append('junctions')  # 倒数第二层
        ordered_layers_keys.extend(other_layers_keys)  # 添加其余部分

        # 按照指定的顺序写入图层
        for layer_name in ordered_layers_keys:
            # 构建带有后缀的图层名称（如果提供了后缀）
            if layer_suffix:
                suffixed_layer_name = f"{layer_name}_{layer_suffix}"
            else:
                suffixed_layer_name = layer_name

            # 写入 GeoPackage 文件
            layers[layer_name].to_file(output_path, driver="GPKG", layer=suffixed_layer_name)

    # 处理保存路径
    def get_result_save_sub_path(self):
        """输出子路径"""
        if self.type != IntersectionType.input:
            raise Exception("真值路口不需要保存哦")
        if self.nickname:
            return os.path.join(self.name, self.version, self.nickname)
        else:
            return os.path.join(self.name, self.version)

    @property
    def result_save_title_prefix(self):
        """输出标题前缀"""
        if self.type != IntersectionType.input:
            raise Exception("真值路口不需要保存哦")
        if self.nickname:
            return f"{self.name}_{self.version}_{self.nickname}"
        else:
            return f"{self.name}_{self.version}"


class TruthIntersectionDataGetter(object):
    def __init__(self, tool_config: ToolConfig, is_ignore_z: bool = True, to_utm: bool = True):
        self.tool_config = tool_config
        self.p = self.tool_config.truth_file_profile
        self.to_utm = to_utm
        self.is_ignore_z = is_ignore_z

    @staticmethod
    def __get_filtered_geometries(gdf: GeoDataFrame, filter_conditions: Dict = None):
        """
        获取过滤后的几何要素，支持坐标系转换。

        :param gdf: 输入的 GeoDataFrame。
        :param filter_conditions: 过滤条件字典（可选）。
        :return: 过滤后的 GeoDataFrame。
        """
        logger.debug(f"传入的筛选条件为{filter_conditions}")
        gdf_filter = GeoDataFrameFilter(gdf)  # 创建 GeoDataFrameFilter 实例

        if filter_conditions:
            # 过滤 GeoDataFrame
            gdf = gdf_filter.filter_gdf(filter_conditions)

        # FIXME 这里可能存在Multi几何体引起的问题。如果碰到无法解决的ug，反注释这行代码看看
        # return ensure_single_geometries(gdf)  # 确保只有单一几何体
        return gdf

    def get_local_truth_data_by_algorithm_range(self, algo_box_id: Union[str, int]) -> Intersection:
        """读取储存在本地的真值、根据ID获取算法框内的真值。前提是真值用任务框的id做的目录名称"""

        lane_lines_data = self.__get_lane_lines(algo_box_id)
        road_boundaries_data = self.__get_road_boundaries(algo_box_id)
        crosswalks_data = self.__get_crosswalks(algo_box_id)
        stop_lines_data = self.__get_stop_lines(algo_box_id)
        traffic_lights_data = self.__get_traffic_lights(algo_box_id)
        arrows_data = self.__get_arrows(algo_box_id)
        intersection_area = self.__get_intersection_area(algo_box_id)
        lanes_data = self.__get_lanes(algo_box_id)
        centerline_data = self.__get_centerlines(algo_box_id)

        # 使用获取到的数据创建Intersection对象
        intersection = Intersection(
            type=IntersectionType.standard,
            name=algo_box_id,
            is_ignored_z=self.is_ignore_z,
            lane_lines=lane_lines_data,
            road_boundaries=road_boundaries_data,
            crosswalks=crosswalks_data,
            stop_lines=stop_lines_data,
            traffic_lights=traffic_lights_data,
            arrows=arrows_data,
            intersection_area=intersection_area,
            lanes=lanes_data,
            lane_centerlines=centerline_data
        )

        return intersection

    def __get_target_object_gdf(self, object_profile: EvaluateElementProfile, algo_box_id: int) -> Optional[
        GeoDataFrame]:

        local_truth_data_dir = os.path.join(self.tool_config.file_paths.data_path,
                                            self.tool_config.file_paths.truth_data_path,
                                            f"{algo_box_id}")
        # 读取真值要素
        object_file = os.path.join(local_truth_data_dir, f"{object_profile.prefix}.{object_profile.suffix}")

        if os.path.isfile(object_file):
            this_gdf = DataReader().get_this_file_ready(file_path=object_file, is_to_utm=self.to_utm,
                                                        is_ignore_z=self.is_ignore_z)
            object_gdf = self.__get_filtered_geometries(gdf=this_gdf,
                                                        filter_conditions=object_profile.filter_conditions.dict() if object_profile.filter_conditions else None)
            if not object_gdf.empty:
                object_gdf = object_gdf.reset_index(drop=True)

        else:
            logger.warning(f"真值目录下不存在此份数据{object_file}")
            object_gdf = None

        return object_gdf

    def __get_lane_lines(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值车道线")
        if self.p.lane_line is None:
            logger.warning("真值数据未带有车道线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.lane_line, algo_box_id=algo_box_id)

    def __get_road_boundaries(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值道路边界线")
        if self.p.road_boundary is None:
            logger.warning("真值数据未带有道路边界线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.road_boundary, algo_box_id=algo_box_id)

    def __get_crosswalks(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值人行横道")
        if self.p.crosswalk is None:
            logger.warning("真值数据未带有人行横道。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.crosswalk, algo_box_id=algo_box_id)

    def __get_stop_lines(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值停止线")
        if self.p.stop_line is None:
            logger.warning("真值数据未带有停止线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.stop_line, algo_box_id=algo_box_id)

    def __get_traffic_lights(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真知信号灯")
        if self.p.trafficlight is None:
            logger.warning("真值数据未带有测信号灯数据格式。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.trafficlight, algo_box_id=algo_box_id)

    def __get_arrows(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值箭头")
        if self.p.lane_marking is None:
            logger.warning("真值数据未带有地面要素。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.lane_marking, algo_box_id=algo_box_id)

    def __get_intersection_area(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值车道路口面")
        if self.p.lane_centerline is None:
            logger.warning("真值数据未带有路口面。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.intersection, algo_box_id=algo_box_id)

    def __get_lanes(self, algo_box_id: int) -> GeoDataFrame:
        # 待实现...
        return GeoDataFrame()

    def __get_centerlines(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取真值车道中心线")
        if self.p.lane_centerline is None:
            logger.warning("真值数据未带有车道中心线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.lane_centerline, algo_box_id=algo_box_id)


class InputIntersectionDataGetter(object):
    """专门用来构建输入路口的方法"""

    def __init__(self, tool_config: ToolConfig, version_config: Version, is_ignore_z: bool = True, to_utm=True):
        self.tool_config = tool_config
        self.p = version_config
        self.input_data_dir = os.path.join(self.tool_config.file_paths.data_path,
                                           self.tool_config.file_paths.input_data_path,
                                           self.p.version,
                                           self.p.nickname) if self.p.nickname else os.path.join(
            self.tool_config.file_paths.data_path,
            self.tool_config.file_paths.input_data_path,
            self.p.version)
        self.is_to_utm = to_utm
        self.is_ignore_z = is_ignore_z

    @staticmethod
    def __get_filtered_geometries(gdf: GeoDataFrame, filter_conditions: Dict = None):
        """
        获取过滤后的几何要素，支持坐标系转换。

        :param gdf: 输入的 GeoDataFrame。
        :param filter_conditions: 过滤条件字典（可选）。
        :return: 过滤后的 GeoDataFrame。
        """
        logger.debug(f"传入的筛选条件为{filter_conditions}")
        gdf_filter = GeoDataFrameFilter(gdf)  # 创建 GeoDataFrameFilter 实例

        if filter_conditions:
            # 过滤 GeoDataFrame
            gdf = gdf_filter.filter_gdf(filter_conditions)

        # return ensure_single_geometries(gdf)  # 确保只有单一几何体
        return gdf

    def get_local_input_data_in_algorithm_range(self, algo_box_id: Union[str, int], nickname=None) -> Intersection:
        """根据ID获取算法框内的输入数据"""

        lane_lines_data = self.__get_lane_lines(algo_box_id)
        road_boundaries_data = self.__get_road_boundaries(algo_box_id)
        crosswalks_data = self.__get_crosswalks(algo_box_id)
        stop_lines_data = self.__get_stop_lines(algo_box_id)
        traffic_lights_data = self.__get_traffic_lights(algo_box_id)
        arrows_data = self.__get_arrows(algo_box_id)
        intersection_area = self.__get_intersection_area(algo_box_id)
        lanes_data = self.__get_lanes(algo_box_id)
        lane_boundary_vectorize_data = self.__get_lane_boundary_vectorize(algo_box_id)
        road_boundary_vectorize_data = self.__get_road_boundary_vectorize(algo_box_id)

        intersection = Intersection(
            type=IntersectionType.input,
            name=algo_box_id,
            version=self.p.version,  # 补上版本号
            nickname=nickname,
            is_ignored_z=self.is_ignore_z,

            lane_lines=lane_lines_data,
            road_boundaries=road_boundaries_data,
            crosswalks=crosswalks_data,
            stop_lines=stop_lines_data,
            traffic_lights=traffic_lights_data,
            arrows=arrows_data,
            intersection_area=intersection_area,
            lanes=lanes_data,
            lane_boundary_vectorize=lane_boundary_vectorize_data,
            road_boundary_vectorize=road_boundary_vectorize_data
        )

        return intersection

    def __get_target_object_gdf(self, object_profile: EvaluateElementProfile, algo_box_id: int) -> Optional[
        GeoDataFrame]:

        local_input_data_dir = os.path.join(self.input_data_dir, f"{algo_box_id}")
        # 读取待评测要素
        object_file = os.path.join(local_input_data_dir, f"{object_profile.prefix}.{object_profile.suffix}")

        if os.path.isfile(object_file):
            this_gdf = DataReader().get_this_file_ready(file_path=object_file, is_to_utm=self.is_to_utm,
                                                        is_ignore_z=self.is_ignore_z)
            object_gdf = self.__get_filtered_geometries(gdf=this_gdf,
                                                        filter_conditions=object_profile.filter_conditions.dict() if object_profile.filter_conditions else None)
            logger.debug(f"在{object_file}路径中找到目标文件，要素数量为{object_gdf.shape[0]}")
        else:
            object_gdf = GeoDataFrame()
            logger.warning(f"{object_profile.prefix}文件未在路径{object_file}中找到")

        return object_gdf

    def __get_lane_lines(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取待评测车道线")
        if self.p.lane_line is None:
            logger.warning("数据未带有车道线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.lane_line, algo_box_id=algo_box_id)

    def __get_road_boundaries(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取待评测道路边界线")
        if self.p.road_boundary is None:
            logger.warning("数据未带有道路边界线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.road_boundary, algo_box_id=algo_box_id)

    def __get_crosswalks(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取待评测人行横道")
        if self.p.crosswalk is None:
            logger.warning("数据未带有人行横道。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.crosswalk, algo_box_id=algo_box_id)

    def __get_stop_lines(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取待评测停止线")
        if self.p.stop_line is None:
            logger.warning("数据未带有停止线。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.stop_line, algo_box_id=algo_box_id)

    def __get_traffic_lights(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取待评测信号灯")
        if self.p.trafficlight is None:
            logger.warning("输入数据未带有测信号灯数据格式。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.trafficlight, algo_box_id=algo_box_id)

    def __get_arrows(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取箭头")
        if self.p.lane_marking is None:
            logger.warning("输入数据未带有地面要素。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.lane_marking, algo_box_id=algo_box_id)

    def __get_intersection_area(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取待评测路口面")
        if self.p.trafficlight is None:
            logger.warning("数据未带有路口面。")
            return GeoDataFrame()
        logger.warning("路口面还没写哈")
        return GeoDataFrame()

    def __get_lanes(self, algo_box_id: int) -> GeoDataFrame:
        # ...
        return GeoDataFrame()

    def __get_lane_boundary_vectorize(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取仅矢量化车道线...")
        if self.p.lane_boundary_vectorize is None:
            logger.warning("输入数据未带有仅矢量化车道线数据格式。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.lane_boundary_vectorize, algo_box_id=algo_box_id)

    def __get_road_boundary_vectorize(self, algo_box_id: int) -> GeoDataFrame:
        logger.debug("正在获取仅矢量化道路边界线...")
        if self.p.lane_boundary_vectorize is None:
            logger.warning("输入数据未带有仅矢量化道路边界线数据格式。")
            return GeoDataFrame()
        return self.__get_target_object_gdf(self.p.road_boundary_vectorize, algo_box_id=algo_box_id)
