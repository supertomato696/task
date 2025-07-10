from src.quality_inspector.inspectors.base_polygon import BasePolygonInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, IssueType
from src.logger import logger
from src.quality_inspector.utils.polygon_process import PolygonGeomChecker


@BaseInspector.register('路口面自相交检测')
class InspectIntersectCrosswalk(BasePolygonInespector):

    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        # 不用特意拍平
        self.polygons_gdf = self.check_polygon_geom_type(self.intersection.intersection_area)
        self.total_count = self.polygons_gdf['ID'].nunique() if not self.polygons_gdf.empty else 0
        self.log_count = 0

    def set_level(self):
        return InspectorLevel.P0

    def inspect(self):
        logger.info(f"开始检查：{self.alias}")
        if self.total_count == 0:
            logger.warning('当前路口没有路口面，不需要检查，BYE！')
            return

        result_poly_gdf, intersection_pts = PolygonGeomChecker.check_self_intersect(self.polygons_gdf)
        self_intersect_poly = result_poly_gdf[result_poly_gdf.self_intersect].copy()
        self_intersect_poly['issue'] = IssueType.intersection.value
        self.log_count = self_intersect_poly['ID'].nunique()
        
        logger.info(f"自相交多边形数量: {self.log_count}, 多边形总数: {self.total_count}")
        self.result['自相交路口面'] = self_intersect_poly
        self.result['自相交交点'] = intersection_pts


    def get_result(self):
        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name": self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.total_count, 
            "log_count": self.log_count
        }
        return stats_dict, self.result
