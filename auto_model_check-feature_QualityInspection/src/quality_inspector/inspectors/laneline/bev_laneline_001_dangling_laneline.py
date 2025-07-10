from geopandas import GeoDataFrame
from shapely.geometry import Point, MultiPoint, LineString
from src.quality_inspector.inspectors.base_line import BaseLineInespector
from src.quality_inspector.inspectors.base import BaseInspector, InspectorLevel, InspectorFeatureType, IssueType
from src.logger import logger

@BaseInspector.register('车道线悬挂点检测')
class DetectDanglingNode(BaseLineInespector):
    def __init__(self, sd_link, intersection, data_dir, buffer_distance=1.5):
        super().__init__(sd_link, intersection, data_dir)
        # 过滤非法的几何并拍平
        self.lanelines_gdf = self.check_and_single_line_geom(self.intersection.lane_lines)
        self.dangling_nodes = []
        self.total_count = 0
        self.buffer_distance = buffer_distance
    
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
        # multi的元素拍平后ID会有重复，因此需要计算ID去重后的数量
        self.total_count = self.lanelines_gdf['ID'].nunique()
        # 建立R-tree空间索引
        sindex = self.lanelines_gdf.sindex

        for idx, row in self.lanelines_gdf.iterrows():
            line = row.geometry
            if isinstance(line, LineString):
                start_point = Point(line.coords[0])
                end_point = Point(line.coords[-1])
                sp_buffer,start_pml = self.__get_buffered_lines(start_point,sindex)
                ep_buffer,end_pml = self.__get_buffered_lines(end_point,sindex)
                is_start_dangling = self.__compare_nodes(idx,start_point,end_point,sp_buffer,start_pml)
                is_end_dangling = self.__compare_nodes(idx,start_point,end_point,ep_buffer,end_pml)
                if is_start_dangling:
                    self.dangling_nodes.append({
                        'line_id':row['ID'],
                        'issue': IssueType.suspend.value,
                        'geometry': start_point
                    })
                if is_end_dangling:
                    self.dangling_nodes.append({
                        'line_id':row['ID'],
                        'issue': IssueType.suspend.value,
                        'geometry': end_point
                    })

    def get_result(self):
        if self.dangling_nodes:
            dangling_nodes_gdf = GeoDataFrame(
                self.dangling_nodes,
                geometry='geometry',
                crs=self.lanelines_gdf.crs
            )
        else:
            dangling_nodes_gdf = GeoDataFrame(
            columns=['line_id', 'geometry'],
            geometry='geometry',
            crs=self.lanelines_gdf.crs
        )
        
        result_layer = {
            "车道线悬挂点": dangling_nodes_gdf
        }

        stats_dict = {
            "inspector_name":self.alias,
            "inspector_level": self.level.name,
            "level_weight": self.level.value,
            "feature_type": self.feature_type,
            "total_count": self.total_count,  # TODO: 线性按里程统计是怎么统计里程
            "log_count":len(self.dangling_nodes)   # TODO: 线性按里程统计是怎么统计里程
        }

        return stats_dict, result_layer


    def __get_buffered_lines(self,point,sindex):
        point_buffer = point.buffer(self.buffer_distance)
        possible_matches_index = list(sindex.intersection(point_buffer.bounds))
        possible_matches_lines = self.lanelines_gdf.iloc[possible_matches_index]
        return point_buffer,possible_matches_lines


    def __compare_nodes(self,idx,sp,ep,buffer,pml):
        flag = False
        for _,line_candidate in pml.iterrows():
            if idx == line_candidate.name:
                continue
            elif line_candidate.geometry.intersects(buffer):
                sp_candidate = Point(line_candidate.geometry.coords[0])
                ep_candidate = Point(line_candidate.geometry.coords[-1])
                # 对向车道的两条车道线过近时，需增加起终点一致判断，若一致直接break
                if ep_candidate == sp: 
                    break
                elif sp_candidate == ep:
                    break
                # shapely Point虽然可哈希，但其哈希值是基于对象的内存地址，而不是基于点的几何坐标。
                # 为了防止意外发生，先把Point转成tuple再用哈希。
                elif len({(sp.x,sp.y),(ep.x,ep.y),(sp_candidate.x,sp_candidate.y),(ep_candidate.x,ep_candidate.y)}) == 4:
                    flag = True
                    break
        return flag


