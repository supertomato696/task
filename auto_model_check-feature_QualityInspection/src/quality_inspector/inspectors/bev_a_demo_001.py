from src.logger import logger
from src.quality_inspector.inspectors.base import (BaseInspector,
                                                   InspectorFeatureType,
                                                   InspectorLevel, IssueType)


@BaseInspector.register('示例质检')
class InspectDemo(BaseInspector):

    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)

    def set_level(self):
        return InspectorLevel.P00
    
    def set_feature_type(self):
        return InspectorFeatureType.others.value
    
    def inspect(self, *args, **kwargs):
        logger.info("示例，执行质检")
    
    def get_result(self, *args, **kwargs):
        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": 0,                   # TODO: 线性按里程统计是怎么统计里程
            "log_count": 0                      # TODO: 线性按里程统计是怎么统计里程
        }
        return stats_dict, self.result