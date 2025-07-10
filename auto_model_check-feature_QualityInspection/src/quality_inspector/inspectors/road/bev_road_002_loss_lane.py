from geopandas import GeoDataFrame
from pandas import notnull
from shapely.geometry import Polygon
from src.logger import logger
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, IssueType
from src.quality_inspector.inspectors.base_polygon import BasePolygonInespector
from src.quality_inspector.utils.utils import validate_geometry


@BaseInspector.register('道路面挂接车道缺失检测')
class InspectRoadLossLane(BasePolygonInespector):
    
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.road_gdf = self.check_and_single_polygon_geom(self.intersection.roads)
        self.lane_gdf = self.check_and_single_polygon_geom(self.intersection.lanes)
        self.road_total_count = self.road_gdf['ID'].nunique() if not self.road_gdf.empty else 0
        self.lane_total_count = self.lane_gdf['ID'].nunique() if not self.lane_gdf.empty else 0
        self.log_count = 0
    
    def set_level(self):
        return InspectorLevel.P0
    
    def inspect(self):
        logger.info(f"开始检查：{self.alias}")
        if self.road_total_count == 0:
            logger.warning('当前路口没有道路面, 不需要检查, BYE!')
            return
        
        if self.lane_total_count == 0:
            logger.warning("当前路口没有车道面, 无需检查, BYE!")
            return
        
        roads_loss_lanes, lanes_with_incorrect_roads = self.__check_road_loss_lane(self.road_gdf, self.lane_gdf)

        roads_loss_lanes['issue'] = IssueType.intersection.value
        self.log_count = roads_loss_lanes['ID'].nunique() if not roads_loss_lanes.empty else 0
        self.result = {
            "缺失车道道路面": roads_loss_lanes, 
            "未正确挂接车道": lanes_with_incorrect_roads
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
    
    @staticmethod
    def __check_road_loss_lane(road_gdf: GeoDataFrame, lane_gdf: GeoDataFrame, ratio: float = 0.5):
        road_result = road_gdf.copy()
        lane_result = lane_gdf.copy()
        # 增加标记位
        road_result['is_loss'] = False
        road_result['loss_lanes'] = None
        # 增加结果信息
        # lane_result['expected_road'] = None
        lanes_expected_roads = {}

        # 建立空间索引
        lane_sindex = lane_result.sindex
        
        for r_idx, r_row in road_result.iterrows():
            road_geom: Polygon = validate_geometry(r_row.geometry)
            r_id = r_row['ID']
            # 无法自修复的多边形跳过
            if not road_geom.is_valid:
                logger.error(f"Road Topo异常要素: {r_id}")
                continue

            
            # 当前已绑定的lane id
            lane_ids_in_road = str(r_row['LANE_ID']).split(',') if notnull(r_row['LANE_ID']) else []
            # 查找可能的lane
            possible_lane_ids = list(lane_sindex.intersection(road_geom.bounds))
            if not possible_lane_ids:
                logger.debug(f"该道路面无车道交集 road ID: {r_id}")
                if possible_lane_ids:
                    logger.error(f"该道路面无车道交集但有挂接车道, road ID: {r_id}, lanes: {lane_ids_in_road}")
                continue
            logger.debug(f"Possible lane ids: {possible_lane_ids}")
            
            # 找到所有可能相交的lane数据
            loss_ids = set()
            possible_lanes = lane_result.iloc[possible_lane_ids]
            for l_idx, l_row in possible_lanes.iterrows():
                if not lanes_expected_roads.get(l_idx, None):
                    lanes_expected_roads[l_idx] = set()
                
                lane_geom: Polygon = validate_geometry(l_row.geometry)
                l_id = l_row['ID']
                # 如果无法自修复，跳过
                if not lane_geom.is_valid:
                    logger.error(f'Lane Topo异常要素: {l_id}')
                    continue

                if not lane_geom.intersection(road_geom):
                    # 车道没有和道路面相交
                    continue

                intersection = lane_geom.intersection(road_geom)
                if intersection.is_empty:
                    continue                # 计算车道相交道路面的面积占比
                intersection_area = intersection.area
                lane_area = lane_geom.area
                intersection_ratio = intersection_area / lane_area
                logger.debug(f"road id: {r_id}, lane id: {l_id}, intersection area ratio: {intersection_ratio}")
                # 未超过阈值，不需要检查
                if intersection_ratio <= ratio:
                    continue

                if l_id not in lane_ids_in_road:
                    loss_ids.add(l_id) # 记录缺失
                    # lane_result.at[l_idx, 'expected_road'] = r_id
                    lanes_expected_roads[l_idx].add(r_id)
            
            if loss_ids:
                road_result.at[r_idx, 'is_loss'] = True
                road_result.at[r_idx, 'loss_lanes'] = ','.join(sorted(list(loss_ids)))
        
        roads_loss_lanes = road_result[road_result.is_loss]

        lane_result['expected_roads'] = None
        for l_idx, road_ids in lanes_expected_roads.items():
            lane_result.at[l_idx, 'expected_roads'] = ",".join(sorted(list(road_ids))) if road_ids else None
        lanes_with_incorrect_roads = lane_result[notnull(lane_result.expected_roads)]

        return roads_loss_lanes, lanes_with_incorrect_roads
            


                

    