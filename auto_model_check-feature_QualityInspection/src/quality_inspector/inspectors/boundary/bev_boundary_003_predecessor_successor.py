from geopandas import GeoDataFrame
from shapely.geometry import Point, MultiPoint, LineString
from src.quality_inspector.inspectors.base_line import BaseLineInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, InspectorFeatureType, IssueType
from src.quality_inspector.utils.line_process import LineProcessor
from src.logger import logger


@BaseInspector.register('道路边界前驱后继关系检测')
class DetectPredecessorSuccessorBoundary(BaseLineInespector):
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        # 过滤非法的几何并拍平
        self.boundary_gdf = self.check_and_single_line_geom(self.intersection.road_boundaries)
        # self.result = {}
        self.total_count = 0
        self.buffer_distance = 1.5
    
    def set_feature_type(self):
        return InspectorFeatureType.line.value
    
    def set_level(self):
        return InspectorLevel.P0
    
    def inspect(self):
        """
        查找未合理挂接的车道线端点

        :param gdf: GeoDataFrame，包含线要素的几何信息
        :return: GeoDataFrame，未合理挂接的车道线端点
        """
        logger.info(f"开始检查：{self.alias}")
        # multi的元素拍平后ID会有重复，因此需要计算ID去重后的数量
        self.total_count = self.boundary_gdf['ID'].nunique()
        self.result['exist_disconnect'] = []
        self.result['potential_connection'] = []
        for idx, row in self.boundary_gdf.iterrows():
            line = row.geometry
            present_id = row['ID']
            predecessor_id = row['PRE_ID']
            successor_id = row['NEXT_ID']
            start_point = Point(line.coords[0])
            end_point = Point(line.coords[-1])
            start_buffer = start_point.buffer(self.buffer_distance)
            end_buffer = end_point.buffer(self.buffer_distance)

            # 验证前驱关系
            validate_ret = LineProcessor.validate_relation(self.boundary_gdf,
                                                           line,
                                                           present_id,
                                                           start_point, predecessor_id,
                                                           'predecessor',
                                                           start_buffer)
            self.result['exist_disconnect'].extend(validate_ret['exist_disconnect'])
            self.result['potential_connection'].extend(validate_ret['potential_connection'])

            # 验证后继关系
            validate_ret = LineProcessor.validate_relation(self.boundary_gdf,
                                                           line,
                                                           present_id,
                                                           end_point,
                                                           successor_id,
                                                           'successor',
                                                           end_buffer)
            self.result['exist_disconnect'].extend(validate_ret['exist_disconnect'])
            self.result['potential_connection'].extend(validate_ret['potential_connection'])

    def get_result(self):
        result_layer = {}
        log_count = 0
        if self.result['exist_disconnect']:
            exist_disconnect_gdf = GeoDataFrame(
                self.result['exist_disconnect'],
                geometry='geometry',
                crs=self.boundary_gdf.crs
            )
            exist_disconnect_gdf['issue'] = IssueType.relation.value
            result_layer["已记录的前驱后继线要素未正确挂接"] = exist_disconnect_gdf
            log_count += len(exist_disconnect_gdf)

        if self.result['potential_connection']:
            potential_connection_gdf = GeoDataFrame(
                self.result['potential_connection'],
                geometry='geometry',
                crs=self.boundary_gdf.crs
            )
            potential_connection_gdf['issue'] = IssueType.relation.value
            result_layer["未记录的潜在前驱后继关系"] = potential_connection_gdf
            log_count += len(potential_connection_gdf)

        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count":log_count   # TODO: 线性按里程统计是怎么统计里程
        }

        return stats_dict, result_layer
