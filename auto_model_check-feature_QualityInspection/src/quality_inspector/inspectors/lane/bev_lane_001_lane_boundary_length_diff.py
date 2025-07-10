from geopandas import GeoDataFrame
from pandas import notnull
from shapely.geometry import Polygon
from src.logger import logger
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, InspectorFeatureType, IssueType
from src.quality_inspector.inspectors.base_polygon import BasePolygonInespector
from src.quality_inspector.inspectors.base_line import BaseLineInespector


@BaseInspector.register('车道左右边线长度差异过大检测')
class InspectLaneBoundaryLengthDiff(BasePolygonInespector,BaseLineInespector):
    
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.laneline_gdf = self.check_and_single_line_geom(self.intersection.lane_lines)
        self.lane_gdf = self.check_and_single_polygon_geom(self.intersection.lanes)
        self.laneline_total_count = self.laneline_gdf['ID'].nunique() if not self.laneline_gdf.empty else 0
        self.lane_total_count = self.lane_gdf['ID'].nunique() if not self.lane_gdf.empty else 0
        self.log_count = 0
    
    def set_level(self):
        return InspectorLevel.P0
    
    def set_feature_type(self):
        return InspectorFeatureType.relation.value
    
    def inspect(self):
        logger.info(f"开始检查：{self.alias}")
        if self.lane_total_count == 0:
            logger.warning('当前路口没有车道, 不需要检查, BYE!')
            return
        
        if self.laneline_total_count == 0:
            logger.warning("当前路口没有车道线, 无需检查, BYE!")
            return
        
        lane_diff_result,laneline_diff_result = self.check_lane_boundary_length_diff(self.laneline_gdf, self.lane_gdf)

        lane_diff_result['issue'] = IssueType.relation.value
        self.log_count = lane_diff_result['ID'].nunique() if not lane_diff_result.empty else 0
        self.result = {
            "左右边线长度差异过大的车道": lane_diff_result, 
            "长度差异较大的车道线": laneline_diff_result
        }

    def get_result(self):
        # 所有的dict都保持这种格式
        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.lane_total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count": self.log_count   # TODO: 线性按里程统计是怎么统计里程
        }

        return stats_dict, self.result
    
    def check_lane_boundary_length_diff(self, laneline_gdf: GeoDataFrame, lane_gdf: GeoDataFrame, ratio: float = 0.8, diff: float = 3.0):
        lane_res_gdf = lane_gdf.copy()
        laneline_res_gdf = laneline_gdf.copy()
        # 增加标记位
        lane_res_gdf['is_diff'] = False
        lane_res_gdf['length_diff'] = None
        laneline_res_gdf['is_diff'] = False

        # 车道挂接左右边线错误，或未挂接。

        for i, row in lane_res_gdf.iterrows():
            # lane_geom: Polygon = row.geomtry
            lane_id = row[['ID']]
            # 获取左边线和右边线的几何信息，如果挂接错误可能没有左右边线
            left_laneline,left_idx = self.__find_feature_by_id(laneline_res_gdf,lane_id,'ID',row['LEFT_BID'])
            right_laneline,right_idx = self.__find_feature_by_id(laneline_res_gdf,lane_id,'ID',row['RIGHT_BID'])
        
            # 计算左边线和右边线的长度
            left_length = float(left_laneline.geometry.length.item())
            right_length = float(right_laneline.geometry.length.item())

            # 找到长边和短边
            if left_length > right_length:
                long_edge = left_length
                short_edge = right_length
            else:
                long_edge = right_length
                short_edge = left_length
            # 检查短边/长边的是否小于阈值ratio
            if short_edge / long_edge < ratio:
                # 计算长边与短边的差
                length_diff = long_edge - short_edge
                # 检查长边与短边的差是否大于阈值diff
                if length_diff > diff:
                    lane_res_gdf.loc[i,'is_diff'] = True
                    lane_res_gdf.loc[i,'length_diff'] = round(length_diff, 2)
                    laneline_res_gdf.loc[left_idx,'is_diff'] = True
                    laneline_res_gdf.loc[right_idx,'is_diff'] = True
                    logger.debug(f"检测出车道：{lane_id} 的左边线：{left_laneline['ID'].values[0]} 和右边线：{right_laneline['ID'].values[0]} 长度差异为：{length_diff:.2f} 米，不符合质检要求。")
                else:
                    continue
            else:
                continue
        lane_diff_result = lane_res_gdf[lane_res_gdf.is_diff]
        laneline_diff_result = laneline_res_gdf[laneline_res_gdf.is_diff]
        return lane_diff_result,laneline_diff_result
            

    @staticmethod
    def __find_feature_by_id(laneline_gdf: GeoDataFrame, lane_id, col_name, target_id):
        feature = laneline_gdf[laneline_gdf[col_name] == target_id]
        # 获取对应的索引
        if not feature.empty:
            index = feature.index[0]  # 获取第一个匹配项的索引
        else:
            index = None  # 如果未找到，返回 None
        if feature.empty:
            logger.error(f"该车道的左右边线在车道线图层中不存在, lane ID: {lane_id}, laneline ID: {target_id}")
        
        return feature,index