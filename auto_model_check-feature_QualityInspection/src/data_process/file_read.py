from math import floor
from typing import List

import geopandas as gpd
from shapely import force_2d
from shapely.geometry.base import BaseGeometry
from shapely.geometry import Point, LineString, Polygon, MultiPoint, MultiLineString, MultiPolygon, GeometryCollection

from src.logger import logger


# def remove_z(geom):
#     """
#     去除几何对象的 Z 坐标。
#
#     如果 geom 不是有效的 Shapely 几何对象，则直接返回原对象。
#     使用通用函数以支持任意维数的输入。
#     """
#     if geom is None:
#         return None
#     if not isinstance(geom, BaseGeometry):
#         # 如果传入的并不是 Shapely 几何，则直接返回
#         return geom
#
#     # 如果几何对象没有 Z 坐标，直接返回
#     if not getattr(geom, "has_z", False):
#         return geom
#
#     try:
#         # 新的转换函数只接收一个坐标元组，然后返回前两个维度
#         return transform(lambda coord: coord[:2], geom)
#     except Exception as e:
#         logger.error(f"去除Z坐标时出错: {e}", exc_info=True)
#         # 出错时，直接返回原几何
#         return geom


def remove_z(geometry):
    """
    忽视几何对象的Z坐标，只返回XY坐标。

    :param geometry: 输入的几何对象（LineString、Polygon、Point或GeometryCollection）。
    :return: 忽略Z坐标后的几何对象。
    """
    if geometry is None or geometry.is_empty:
        return geometry  # 直接返回 None 或 空几何

    if geometry.has_z:
        if isinstance(geometry, Point):
            # Point: 只取前两个坐标值 (x, y)
            return Point(geometry.coords[0][:2])
        elif isinstance(geometry, LineString):
            # LineString: 对每个点取前两个坐标值
            return LineString([coords[:2] for coords in geometry.coords])
        elif isinstance(geometry, Polygon):
            # Polygon: 处理外部环和所有内部环
            exterior_2d = [coords[:2] for coords in geometry.exterior.coords]
            interiors_2d = [[coords[:2] for coords in interior.coords]
                            for interior in geometry.interiors]
            return Polygon(exterior_2d, interiors_2d)
        elif isinstance(geometry, MultiPoint):
            # MultiPoint: 对每个 Point 递归调用 remove_z
            points_2d = [remove_z(p) for p in geometry.geoms]
            return MultiPoint([p for p in points_2d if p is not None and not p.is_empty])  # 过滤空结果
        elif isinstance(geometry, MultiLineString):
            # MultiLineString: 对每个 LineString 递归调用 remove_z
            lines_2d = [remove_z(line) for line in geometry.geoms]
            return MultiLineString([line for line in lines_2d if line is not None and not line.is_empty])  # 过滤空结果
        elif isinstance(geometry, MultiPolygon):
            # MultiPolygon: 对每个 Polygon 递归调用 remove_z
            polygons_2d = [remove_z(poly) for poly in geometry.geoms]
            return MultiPolygon([poly for poly in polygons_2d if poly is not None and not poly.is_empty])  # 过滤空结果
        elif isinstance(geometry, GeometryCollection):
            # GeometryCollection: 对每个成员几何递归调用 remove_z
            geoms_2d = [remove_z(g) for g in geometry.geoms]
            return GeometryCollection([g for g in geoms_2d if g is not None and not g.is_empty])  # 过滤空结果
        else:
            # 对于其他未知类型，可以选择记录警告或抛出错误
            logger.warning(f"remove_z 函数遇到未直接支持的几何类型: {type(geometry)}，尝试返回原对象。")
            return geometry  # 或者 raise TypeError(...)
    else:
        # 如果几何本身没有 Z 坐标，直接返回原对象
        return geometry


class DataReader(object):

    # ------------------------------------------------------------------------------
    # CRS 与几何处理函数
    # ------------------------------------------------------------------------------
    @staticmethod
    def calculate_utm_zone(gdf: gpd.GeoDataFrame) -> str:
        """
        更健壮的 UTM 带计算。使用 gdf.unary_union 计算所有几何的合并中心。
        参数:
            gdf: 输入 GeoDataFrame
        返回:
            EPSG 编号字符串，如 "EPSG:32633"
        """
        try:
            # 自相交多边形几何无法进行union计算
            centroid = gdf.union_all().centroid
            lon = centroid.x
            lat = centroid.y

            # UTM 带计算逻辑
            utm_zone = int(floor((lon + 180) / 6) + 1)
            northern = lat >= 0
            epsg_code = 32600 + utm_zone if northern else 32700 + utm_zone

            # 验证 EPSG 代码有效性：构造空 GeoDataFrame 并设置 CRS
            test_crs = f"EPSG:{epsg_code}"
            tmp = gpd.GeoDataFrame(geometry=[Point(0, 0)])
            tmp.set_crs(test_crs, inplace=True)
            logger.debug(f"计算得到 UTM 带 CRS：{test_crs}")
            return test_crs
        except Exception as e:
            logger.error(f"UTM 带计算失败: {e}", exc_info=True)
            return "EPSG:3857"  # 回退至 Web Mercator

    def handle_crs(self, gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
        """
        坐标系处理逻辑：
          1. 如果数据没有定义crs，则假设为84(实际上是02)，坐标系转换挖了好多坑；
          2. 如果crs不是投影坐标系，则尝试转成墨卡托；
             如失败则使用自定义等距圆锥投影，仍失败则使用墨卡托
        参数:
            gdf: 的输入GeoDataFrame
        返回:
            投影后的GeoDataFrame
        """
        if gdf.crs is None:
            logger.debug("数据未定义坐标系，假设为 WGS84")
            gdf = gdf.set_crs("EPSG:4326")

        if not gdf.crs.is_projected:
            # logger.debug("非投影坐标系，正在尝试转换...")
            # 尝试 UTM 转换
            target_crs = self.calculate_utm_zone(gdf)
            try:
                gdf_proj = gdf.to_crs(target_crs)
                # logger.debug(f"成功转换为 {target_crs}")
                return gdf_proj
            except Exception as e:
                logger.error(f"UTM 转换失败: {e}", exc_info=True)
                # 回退方案：使用等距圆锥投影
                try:
                    centroid = gdf.union_all().centroid
                    custom_crs = f"+proj=eqc +lat_ts={centroid.y} +lon_0={centroid.x}"
                    logger.debug(f"尝试使用自定义投影: {custom_crs}")
                    return gdf.to_crs(custom_crs)
                except Exception as e2:
                    logger.error(f"自定义投影失败，使用 Web Mercator: {e2}", exc_info=True)
                    return gdf.to_crs("EPSG:3857")
        return gdf

    def get_this_file_ready(self, file_path: str, is_to_utm: bool = True, valid_types: List = None,
                            is_ignore_z: bool = True) -> gpd.GeoDataFrame:
        """
        读取并处理车道线文件：
          - 读取 shapefile 文件；
          - 数据清洗（排除空几何、非线要素）；
          - 根据需要去除高程信息；
          - 坐标转换（保证单位为米，便于距离计算）。

        参数:
            file_path: shapefile 文件路径
            is_to_utm: 是否转换到 UTM 投影（默认 True）
            valid_types: 允许的几何类型列表，例如 ['LineString', 'MultiLineString'],默认为空
            is_ignore_z: 是否去除 Z 坐标（默认 True）

        返回:
            处理后的 GeoDataFrame；若读取或转换失败则抛出异常
        """
        try:
            gdf = gpd.read_file(file_path)
        except Exception as e:
            logger.exception(f"读取 shapefile 失败: {e}", exc_info=True)
            raise UserWarning(f"读取 shapefile 失败: {e}")

        gdf = gdf[gdf.geometry.notnull()].copy()
        if gdf.empty:
            logger.warning(f"{file_path}文件中有效要素为0,返回空GDF")
            return gdf
        else:
            logger.debug("正在进行数据清洗...")

        # 若指定有效几何类型，则过滤数据
        if valid_types:
            gdf = gdf[gdf.geometry.type.isin(valid_types)].copy()

        # 去除 Z 坐标（利用 Shapely 原生方法实现，执行效率高）
        if is_ignore_z:
            # gdf['geometry'] = gdf['geometry'].apply(remove_z)
            gdf.geometry = force_2d(gdf.geometry)

        # 坐标系处理与转换（保证投影单位为米，适合距离计算）
        if is_to_utm:
            gdf = self.handle_crs(gdf)

        return gdf
