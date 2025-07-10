from src.quality_inspector.inspectors.base_line import BaseLineInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, InspectorFeatureType, IssueType
from src.quality_inspector.utils.line_process import LineProcessor
from src.tools.common import ensure_single_geometries_v2
from src.logger import logger

@BaseInspector.register('车道线交叉检测')
class DetectIntersectLaneLine(BaseLineInespector):
    
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.lanelines_gdf = self.check_and_single_line_geom(self.intersection.lane_lines)
        self.total_count = self.lanelines_gdf['ID'].nunique() if not self.lanelines_gdf.empty else 0
        self.log_count = 0

    def set_feature_type(self):
        return InspectorFeatureType.line.value
    
    def set_level(self):
        return InspectorLevel.P0
    
    def inspect(self):
        """
        查找相交的线要素和交叉点。

        :param gdf: GeoDataFrame，包含线要素的几何信息
        :return: 两个 GeoDataFrame，分别包含相交的线要素和交叉点
        """
        logger.info(f"开始检查：{self.alias}")
        if self.total_count == 0:
            logger.warning('当前路口没有车道线，不需要检查，BYE！')
            return
        
        lines, intersect_pts = LineProcessor.check_line_intersection(self.lanelines_gdf)
        intersect_lines = lines[lines.is_intersection].copy()
        intersect_lines['issue'] = IssueType.intersection.value
        self.log_count = intersect_lines['ID'].nunique() if not intersect_lines.empty else 0
        self.result = {
            "相交车道线": intersect_lines, 
            "车道线交点": intersect_pts
        }

    def get_result(self):
        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count": self.log_count   # TODO: 线性按里程统计是怎么统计里程
        }

        return stats_dict, self.result

    