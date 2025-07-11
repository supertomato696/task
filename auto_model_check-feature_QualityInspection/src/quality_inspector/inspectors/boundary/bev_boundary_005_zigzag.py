from src.quality_inspector.inspectors.base_line import BaseLineInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, IssueType
from src.logger import logger
from src.quality_inspector.utils.angle import get_angle_usability


@BaseInspector.register('道路边界折角检测')
class InspectZigzagBoundary(BaseLineInespector):

    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.boundary_gdf = self.check_and_single_line_geom(self.intersection.road_boundaries)
        self.total_count = self.boundary_gdf['ID'].nunique() if not self.boundary_gdf.empty else 0
        self.zigzag_count = 0
        self.max_deflection_degrees = 10.0

    def set_level(self):
        return InspectorLevel.P1

    def inspect(self):
        logger.info(f"开始检查：{self.alias}")
        if self.total_count == 0:
            logger.warning('当前路口没有道路边界，不需要检查，BYE！')
            return

        # 初始化工具
        angle_dectect_result = get_angle_usability(
            self.boundary_gdf,
            max_deflection_degrees=self.max_deflection_degrees
        )
        # 只保留折角的
        lg_threhold_result = angle_dectect_result[angle_dectect_result.is_reflex_angle].copy()
        sq_threhold_result = angle_dectect_result[~angle_dectect_result.is_reflex_angle].copy()
        sq_threhold_result['issue'] = IssueType.zigzag.value
        if lg_threhold_result.empty:
            logger.info(f"DONE! 未发现折角，角度阈值：{self.max_deflection_degrees}°")
        else:
            self.zigzag_count = lg_threhold_result['ID'].nunique()
            logger.info(f"DONE! 发现折角数量: {lg_threhold_result.shape[0]}，角度阈值：{self.max_deflection_degrees}°")
        self.result['折角异常道路边界'] = lg_threhold_result
        self.result['符合要求道路边界'] = sq_threhold_result

    def get_result(self):
        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name": self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count": self.zigzag_count  # TODO: 线性按里程统计是怎么统计里程
        }
        return stats_dict, self.result
