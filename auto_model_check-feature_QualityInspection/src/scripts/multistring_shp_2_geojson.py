# -*- coding: utf-8 -*-
# @Time    : 2025/3/1 14:27
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : process
import geopandas as gpd
from shapely.geometry import LineString


def breakdown_multistring(input_shp_path, output_geojson_path):
    # 读取 MultiLineString 的 Shapefile 文件

    # 加载 Shapefile
    gdf = gpd.read_file(input_shp_path)

    # 初始化一个列表用于存储单线段
    lines = []

    # 遍历 GeoDataFrame 中的每一行
    for index, row in gdf.iterrows():
        geom = row['geometry']

        # 检查 geom 是否为 None
        if geom is not None:
            if geom.geom_type == 'MultiLineString':
                # 如果是 MultiLineString，分解为单个 LineString
                for line in geom:
                    lines.append(LineString(line.coords))
            elif geom.geom_type == 'LineString':
                # 如果是 LineString，直接添加
                lines.append(geom)

    # 创建一个新的 GeoDataFrame
    lines_gdf = gpd.GeoDataFrame(geometry=lines, crs=gdf.crs)

    # 将单线段保存为 GeoJSON
    lines_gdf.to_file(output_geojson_path, driver='GeoJSON')

    print(f'Successfully converted to GeoJSON and saved at {output_geojson_path}')

if __name__ == '__main__':
    input_shp = "./road_boundary_vectorize.shp"
    output_geojson = 'road_boundary_vectorize.geojson'