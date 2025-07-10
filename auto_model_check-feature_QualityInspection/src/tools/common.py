import math
import json
import os
from typing import Dict, Optional
import yaml
import geopandas as gpd
from geopandas import GeoDataFrame
from pyproj import CRS
from shapely import MultiPolygon, MultiLineString, Polygon, LineString, Point


# 获取给定相对路径的绝对路径
def get_absolute_path(relative_path: str) -> str:
    """
    计算并返回指定相对路径的绝对路径。

    :param relative_path: 相对路径字符串，指向文件或目录。
    :return: 输入相对路径的绝对路径。
    """
    return os.path.abspath(relative_path)


def transform_to_utm(gdf) -> GeoDataFrame:
    # 默认50
    utm_crs: CRS = CRS.from_epsg(32600 + 50)

    # 转成UTM坐标系并返回
    return gdf.to_crs(utm_crs)


def check_all_instance(gdf: gpd.GeoDataFrame) -> bool:
    """Checks if all geometries in a GeoDataFrame are LineStrings using isinstance."""
    if gdf.empty:
        return True
    return gdf.geometry.apply(lambda geom: isinstance(geom, (LineString, Polygon, Point))).all()


def ensure_single_geometries(gdf) -> gpd.GeoDataFrame:
    """
    将 GeoDataFrame 中的几何对象转换为单一的 Polygon 和 LineString。

    :param gdf: GeoDataFrame, 包含可能的 MultiPolygon 和 MultiLineString 几何对象
    :return: GeoDataFrame, 仅包含 Polygon 和 LineString，保留原始属性和CRS
    """

    # 使用 explode 方法分解复杂几何
    # 使用 explode(index_parts=False) 分解 MultiPolygon 和 MultiLineString 对象。index_parts=False 可以保持原始索引而不分解为多层索引。
    exploded_gdf = gdf.explode(index_parts=False)

    # 创建新的几何列表和属性列表
    single_geometries = []
    attributes = []

    for idx, row in exploded_gdf.iterrows():
        geom = row['geometry']
        if isinstance(geom, (MultiPolygon, MultiLineString)):
            for part in geom:
                # 对于每个分解后的几何对象，保留原始行的属性值（去除 geometry 列），并将单一几何对象附加到 single_geometries 列表
                single_geometries.append(part)
                attributes.append(row.drop('geometry'))
        elif isinstance(geom, (Polygon, LineString, Point)):
            single_geometries.append(geom)
            attributes.append(row.drop('geometry'))

    # 创建新的 GeoDataFrame，一次性添加所有行.设置几何列为活动几何列.
    final_gdf = gpd.GeoDataFrame(attributes, geometry=single_geometries)

    # 重置索引
    final_gdf.reset_index(drop=True, inplace=True)

    # 赋予新的 GeoDataFrame 原始 CRS
    final_gdf.crs = gdf.crs

    if check_all_instance(final_gdf):
        return final_gdf
    else:
        return ensure_single_geometries(final_gdf)


def ensure_single_geometries_v2(gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """
    将 GeoDataFrame 中的几何对象转换为单一的 Polygon 和 LineString。

    :param gdf: GeoDataFrame, 包含可能的 MultiPolygon 和 MultiLineString 几何对象
    :return: GeoDataFrame, 仅包含 Polygon 和 LineString，保留原始属性和CRS
    """

    target_crs = gdf.crs
    src = gdf.copy()
    # 拍平并展开所有multi几何
    # gdf['geometry'] = gdf['geometry'].apply(
    #     lambda x: [geom for geom in x.geoms] if x.geom_type in ('MultiLineString', 'MultiPolygon') else [x]
    # )
    # 展开所有展开所有符合的要素
    # gdf = gdf.explode(index_parts=False).reset_index(drop=True)
    exploded = src.explode(ignore_index=True)
    exploded.set_crs(target_crs, allow_override=True, inplace=True)
    # 不加这句会变成dataframe
    # gdf = gpd.GeoDataFrame(gdf, geometry=gdf['geometry'], crs=target_crs)

    return exploded


def lat_lon_to_tile_id(lat, lon, zoom=13):
    """
    根据给定的经纬度计算对应瓦片 ID（x_y 格式）

    :param lat: 纬度 (float)
    :param lon: 经度 (float)
    :param zoom: 瓦片缩放级别 (int)，默认为 13

    :return: str: 瓦片 ID，格式为 x_y
    """
    # 计算 x 行列号
    n = 2 ** zoom
    x = int((lon + 180) / 360 * n)

    # 计算 y 行列号
    y = int((1 - (math.log(math.tan(math.radians(lat)) +
                           (1 / math.cos(math.radians(lat))))) / math.pi) / 2 * n)

    return f"{x}_{y}"


def get_neighbors(tile_id) -> list[str]:
    """
    根据给定的瓦片 ID 计算该瓦片的八邻域的瓦片 ID

    :param tile_id: 瓦片 ID (str)，格式为 x_y
    :return: 邻域瓦片 ID 列表list[str]，格式为 x_y
    """
    # 解析瓦片 ID
    x, y = map(int, tile_id.split('_'))

    # 计算八邻域的 (x, y)
    neighbors = [
        f"{x - 1}_{y - 1}",  # 左上
        f"{x}_{y - 1}",  # 上
        f"{x + 1}_{y - 1}",  # 右上
        f"{x - 1}_{y}",  # 左
        f"{x + 1}_{y}",  # 右
        f"{x - 1}_{y + 1}",  # 左下
        f"{x}_{y + 1}",  # 下
        f"{x + 1}_{y + 1}"  # 右下
    ]

    return neighbors


def point_to_tile_id(point, zoom=13):
    """
    根据 Point 对象计算对应的瓦片 ID

    :param point: Shapely Point 对象
    :param zoom: 瓦片缩放级别，默认为 13
    :return: 瓦片 ID，格式为 x_y
    """
    if not isinstance(point, Point):
        raise ValueError("输入必须是一个 Point 对象")

    lat = point.y  # 从 Point 获取纬度
    lon = point.x  # 从 Point 获取经度

    return lat_lon_to_tile_id(lat, lon, zoom)


def extract_filename(filepath, with_extension=False):
    """
    从文件路径中提取文件名。
    :param  filepath: 文件路径字符串。
    :param  with_extension: 布尔值，指示是否包含文件扩展名。默认为 False。

    :return: 文件名字符串，如果路径无效则返回 None。
    """
    try:
        filename = os.path.basename(filepath)
        if not with_extension:
            filename = os.path.splitext(filename)[0]
        return filename
    except:
        return None


def yaml_safe_load(file_path):
    with open(file_path, 'r', encoding='utf8') as file:
        content = yaml.safe_load(file)
    return content


def json_dump(data: Dict, filepath: str):
    with open(filepath, 'w+', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


def find_version_number(path_string: str) -> Optional[str]:
    import re
    pattern = r"(v\d+\.\d+\.\d+)"

    match = re.search(pattern, path_string)

    if match:
        # match.group(0) 会返回整个匹配到的字符串，即 "v3.05.02_"
        # match.group(1) 只会返回第一个捕获组内的内容，即 "3.05.02"
        return match.group(1)

    return None


def find_build_task_id(path_string: str) -> Optional[str]:
    import re
    pattern = r"ID(\d+)\.zip$"

    match = re.search(pattern, path_string)

    if match:
        return match.group(1)

    return None
