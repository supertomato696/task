from geopandas import GeoDataFrame
from pandas import notnull
from shapely.geometry import Polygon
from src.logger import logger
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, InspectorFeatureType, IssueType
from src.quality_inspector.inspectors.base_polygon import BasePolygonInespector
from src.quality_inspector.inspectors.base_line import BaseLineInespector


@BaseInspector.register('道路面内车道与车道线数量差异过大检测')
class InspectLaneBoundaryLengthDiff(BasePolygonInespector,BaseLineInespector):
    
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.road_gdf = self.check_and_single_polygon_geom(self.intersection.roads)
        self.laneline_gdf = self.check_and_single_line_geom(self.intersection.lane_lines)
        self.lane_gdf = self.check_and_single_polygon_geom(self.intersection.lanes)
        self.laneline_total_count = self.laneline_gdf['ID'].nunique() if not self.laneline_gdf.empty else 0
        self.lane_total_count = self.lane_gdf['ID'].nunique() if not self.lane_gdf.empty else 0
        self.road_total_count = self.road_gdf['ID'].nunique() if not self.road_gdf.empty else 0
        self.log_count = 0
    
    def set_level(self):
        return InspectorLevel.P0
    
    def set_feature_type(self):
        return InspectorFeatureType.relation.value
    
    def inspect(self):
        if self.laneline_total_count == 0:
            logger.warning("当前路口没有车道线, 无需检查, BYE!")
            return
        
        logger.info(f"开始检查：{self.alias}")
        if self.lane_total_count == 0:
            logger.warning('当前路口没有车道, 不需要检查, BYE!')
            return

        if self.road_total_count == 0:
            logger.warning("当前路口没有道路面, 无需检查, BYE!")
            return
        
        road_diff_result,lane_diff_result,laneline_diff_result = self.check_laneline_lane_in_road_diff(self.road_gdf, self.laneline_gdf, self.lane_gdf)

        road_diff_result['issue'] = IssueType.relation.value
        self.log_count = road_diff_result['ID'].nunique() if not road_diff_result.empty else 0
        self.result = {
            "车道与车道线数量差异多大的道路面": road_diff_result,
            "道路面内车道（属性关联）": lane_diff_result, 
            "道路面内车道线（属性关联）": laneline_diff_result
        }

    def get_result(self):
        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.road_total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count": self.log_count   # TODO: 线性按里程统计是怎么统计里程
        }

        return stats_dict, self.result
    
    def check_laneline_lane_in_road_diff(self, road_gdf: GeoDataFrame, laneline_gdf: GeoDataFrame, lane_gdf: GeoDataFrame, diff: int = 3):
        road_res_gdf = road_gdf.copy()
        lane_res_gdf = lane_gdf.copy()
        laneline_res_gdf = laneline_gdf.copy()
        # 增加标记位
        road_res_gdf['is_diff'] = False
        lane_res_gdf['count_diff'] = None
        lane_res_gdf['is_diff'] = False
        laneline_res_gdf['is_diff'] = False

        for idx, row in road_res_gdf.iterrows():
            road_id = row['ID']
            lane_id_in_road = row['LANE_ID'].split(',')
            lane_in_road = []
            laneline_id_in_road = []
            for lane_id in lane_id_in_road:
                lane_feature,lane_idx = self.__find_feature_by_id(lane_res_gdf,'lane','ID',lane_id)
                if not lane_feature.empty:
                    if lane_idx not in lane_in_road:
                        lane_in_road.append(lane_idx)
                    left_laneline,left_idx = self.__find_feature_by_id(laneline_res_gdf,'lane_boundary','ID',lane_feature['LEFT_BID'].values[0])
                    right_laneline,right_idx = self.__find_feature_by_id(laneline_res_gdf,'lane_boundary','ID',lane_feature['RIGHT_BID'].values[0])
                    if not left_laneline.empty and left_idx not in laneline_id_in_road:
                        laneline_id_in_road.append(left_idx)
                    if not right_laneline.empty and right_idx not in laneline_id_in_road:
                        laneline_id_in_road.append(right_idx)
            lanes_count = len(lane_id_in_road)
            lanelines_count = len(laneline_id_in_road)
            count_diff = abs(lanelines_count - lanes_count)
            if count_diff > diff:
                road_res_gdf.loc[idx,'is_diff'] = True
                road_res_gdf.loc[idx,'count_diff'] = count_diff
                for idx in lane_in_road:
                    lane_res_gdf.loc[idx,'is_diff'] = True
                for idx in laneline_id_in_road:
                    laneline_res_gdf.loc[idx,'is_diff'] = True
                logger.debug(f"检测出道路面 {road_id} 内 车道线数量({lanelines_count}) - 车道数量({lanes_count}) = {count_diff}，高于阈值{diff}")
            else:
                continue
        road_diff_result = road_res_gdf[road_res_gdf.is_diff]
        lane_diff_result = lane_res_gdf[lane_res_gdf.is_diff]
        laneline_diff_result = laneline_res_gdf[laneline_res_gdf.is_diff]
        return road_diff_result,lane_diff_result,laneline_diff_result
            
    @staticmethod
    def __find_feature_by_id(gdf: GeoDataFrame, gdf_type, col_name, target_id):
        feature = gdf[gdf[col_name] == target_id]
        # 获取对应的索引
        if not feature.empty:
            index = feature.index[0]  # 获取第一个匹配项的索引
        else:
            index = None  # 如果未找到，返回 None
        if feature.empty:
            logger.error(f"ID为：{target_id}的要素在图层{gdf_type}中不存在！")
        
        return feature,index