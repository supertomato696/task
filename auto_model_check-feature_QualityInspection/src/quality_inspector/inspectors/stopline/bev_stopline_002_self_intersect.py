from src.quality_inspector.inspectors.base_line import BaseLineInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, InspectorFeatureType, IssueType
from src.tools.common import ensure_single_geometries_v2
from src.logger import logger
from src.quality_inspector.utils.line_process import LineProcessor

@BaseInspector.register('停止线自相交检测')
class InspectSelfIntersectStopline(BaseLineInespector):
    
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.stopline = ensure_single_geometries_v2(self.check_line_geom_type(self.intersection.stop_lines))
        self.total_count = self.stopline['ID'].nunique() if not self.stopline.empty else 0
        self.log_count = 0

    def set_feature_type(self):
        return InspectorFeatureType.line.value
    
    def set_level(self):
        return InspectorLevel.P0
    
    def inspect(self):
        logger.info(f"开始检查：{self.alias}")
        if self.total_count == 0:
            logger.warning('当前路口没有停止线，不需要检查，BYE！')
            return
        lines = self.stopline.copy()
        # 检查自相交，并提取自相交的交点
        lines, intersect_pts = LineProcessor.check_self_intersect(lines)
        # 过滤出自相交的线
        intersect_lines = lines[lines.self_intersect].copy()
        # 增加问题类型
        intersect_lines['issue'] = IssueType.intersection.value
        # 计算自相交的车道数数量
        self.log_count = intersect_lines['ID'].nunique()
        logger.info(f"自相交线数量: {self.log_count}, 线总数: {self.total_count}")
        # 保存结果
        self.result['自相交停止线'] = intersect_lines
        self.result['自相交交点'] = intersect_pts

    def get_result(self):

        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count":self.log_count   # TODO: 线性按里程统计是怎么统计里程
        }

        return stats_dict, self.result

    