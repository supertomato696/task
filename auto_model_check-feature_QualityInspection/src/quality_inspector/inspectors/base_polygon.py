from geopandas import GeoDataFrame
from src.quality_inspector.inspectors.base import BaseInspector, InspectorFeatureType
from src.logger import logger
from src.tools.common import ensure_single_geometries_v2


class BasePolygonInespector(BaseInspector):

    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
    
    def set_feature_type(self):
        return InspectorFeatureType.polygon.value
    
    def check_polygon_geom_type(self, polygons: GeoDataFrame):
        if polygons.empty:
            logger.debug(f"Empty Data")
            return polygons

        gdf = polygons.copy()
        legal = gdf.geometry.geom_type.isin(['Polygon', 'MultiPolygon'])
        if not legal.all():
            for idx, row in gdf[~legal].iterrows():
                logger.warning(f'包含非Polygon数据, ID={row["ID"]}, type={row.geometry.geom_type}')
            return gdf[legal]
        return gdf
    
    def check_and_single_polygon_geom(self, polygons: GeoDataFrame):
        return ensure_single_geometries_v2(self.check_polygon_geom_type(polygons))