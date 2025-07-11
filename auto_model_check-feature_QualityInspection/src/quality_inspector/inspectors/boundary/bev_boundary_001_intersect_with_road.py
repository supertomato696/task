from shapely.wkb import loads as wkb_loads
from geopandas import GeoDataFrame
from src.quality_inspector.inspectors.base_line import BaseLineInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, IssueType
from src.quality_inspector.utils.utils import extract_line_intersection_points
from src.logger import logger

@BaseInspector.register('道路边界与道路面交叉检测')
class InspectBoundaryInRoad(BaseLineInespector):
    
    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
        self.boundary_gdf = self.check_and_single_line_geom(self.intersection.road_boundaries)
        self.road_gdf = self.intersection.roads
        self.total_count = self.boundary_gdf['ID'].nunique() if not self.boundary_gdf.empty else 0
        self.log_count = 0
    
    def set_level(self):
        return InspectorLevel.P0
    
    def inspect(self):
        logger.info(f"开始检查：{self.alias}")
        if self.total_count == 0:
            logger.warning('当前路口没有道路边界，不需要检查，BYE！')
            return
        
        roads_total_cnt = self.road_gdf['ID'].nunique() if not self.boundary_gdf.empty else 0
        if roads_total_cnt == 0:
            logger.warning("当前路口没有道路面")
            return
        
        boundaries, intersect_pts = self.__check_lines_in_polygon(self.boundary_gdf, self.road_gdf)
        # 保存结果
        if intersect_pts:
            intersect_pts_gdf = GeoDataFrame(intersect_pts, geometry='geometry', crs=self.boundary_gdf.crs)
        else:
            intersect_pts_gdf = GeoDataFrame(columns=['bid', 'rid', 'geometry'], geometry='geometry', crs=self.boundary_gdf.crs)
        
        lines_in_road_gdf = boundaries[boundaries.in_road].copy()
        lines_in_road_gdf['issue'] = IssueType.intersection.value
        self.log_count = lines_in_road_gdf['ID'].nunique() if not lines_in_road_gdf.empty else 0
        self.result = {
            "异常道路边界": lines_in_road_gdf, 
            "道路边界交点": intersect_pts_gdf
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
    
    @staticmethod
    def __check_lines_in_polygon(line_gdf: GeoDataFrame, polygon_gdf: GeoDataFrame):
        line_result = line_gdf.copy()
        line_result['in_road'] = False
        road_geom_dict = dict(zip(polygon_gdf['ID'], polygon_gdf.geometry))

        intersect_pts = []

        for idx, row in line_result.iterrows():
            boundary = row.geometry
            boundary_id = row['ID']
            road_id = row['ROAD_ID']
            road = road_geom_dict.get(road_id, None)

            if not road:
                logger.warning(f"绑定道路面不存在. boundary_id: {boundary_id}, road_id: {road_id}")
                continue

            intersect_pts_wkb = extract_line_intersection_points(boundary, road)
            if not intersect_pts_wkb:
                logger.debug(f"BID: {boundary_id} 和 RID: {road_id} 没有交点")
                continue
            # 有交集，先标记
            line_result.at[idx, 'in_road'] = True
            for pt_wkb in intersect_pts_wkb:
                pt = wkb_loads(pt_wkb)
                intersect_pts.append({
                    "bid": boundary_id,
                    "rid": road_id,
                    "geometry": pt
                })
        return line_result, intersect_pts
    