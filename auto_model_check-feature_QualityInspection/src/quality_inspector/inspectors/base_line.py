from geopandas import GeoDataFrame
from src.quality_inspector.inspectors.base import BaseInspector, InspectorFeatureType
from src.logger import logger
from src.tools.common import ensure_single_geometries_v2


class BaseLineInespector(BaseInspector):

    def __init__(self, sd_link, intersection, data_dir):
        super().__init__(sd_link, intersection, data_dir)
    
    def set_feature_type(self):
        return InspectorFeatureType.line.value
    
    def check_line_geom_type(self, lines: GeoDataFrame):
        if lines.empty:
            logger.debug(f"Empty Data")
            return lines

        gdf = lines.copy()
        legal = gdf.geometry.geom_type.isin(['LineString', 'MultiLineString'])
        if not legal.all():
            for idx, row in gdf[~legal].iterrows():
                logger.warning(f'Not linestring in geometry at idx={row["ID"]}, type={row.geometry.geom_type}')
            return gdf[legal]
        return gdf
    
    def check_and_single_line_geom(self, lines: GeoDataFrame):
        return ensure_single_geometries_v2(self.check_line_geom_type(lines))