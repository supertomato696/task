# -*- coding: utf-8 -*-
# @Time    : 2025/6/5 21:17
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : polygon_process.py

from typing import Tuple, Union, Optional, Iterator

from pydantic import BaseModel
from shapely.geometry import Polygon, MultiPolygon, LineString
from shapely.validation import explain_validity
from shapely.strtree import STRtree
from shapely.wkb import loads as wkb_loads
from geopandas import GeoDataFrame


from src.quality_inspector.error import InvalidGeometryTypeError, BadPolygonError
from src.quality_inspector.utils.utils import extract_line_intersection_points


class PolygonInspectInfo(BaseModel):
    """
    验证结果数据模型。
    """
    geom_type: str  # 输入几何的类型 ("Polygon"或"MultiPolygon")
    is_valid: bool   # 几何是否符合OGC标准（无自相交等）。
    has_holes: Optional[bool]  # 如果几何有效，True表示至少有一个洞，False 表示没有洞；如果几何无效，此字段为None。
    is_concave: Optional[bool]  # 如果几何有效，True表示凹多边形形，False表示不是凹多边形；如果几何无效，此字段为None。


class PolygonGeomChecker:
    """
    一个用于检查Polygon和MultiPolygon对象各种几何属性的类。
    """

    @staticmethod
    def _iterate_polygons(geometry: Union[Polygon, MultiPolygon]) -> Iterator[Polygon]:
        """
        遍历并返回所有的 Polygon 实例。
        """
        if isinstance(geometry, Polygon):
            yield geometry
        elif isinstance(geometry, MultiPolygon):
            yield from geometry.geoms
        else:
            raise InvalidGeometryTypeError(
                f"Input must be a Polygon or MultiPolygon, not {type(geometry).__name__}"
            )

    @staticmethod
    def has_holes(geometry: Union[Polygon, MultiPolygon]) -> bool:
        """
        检查Polygon或MultiPolygon是否包含洞。
        对于MultiPolygon如果其任何组成Polygon包含空洞则为True。
        """
        if geometry.is_valid:
            for poly in PolygonGeomChecker._iterate_polygons(geometry):
                if poly.interiors:
                    return True
            return False
        else:
            raise BadPolygonError("Bad Polygon")

    @staticmethod
    def is_concave(geometry: Union[Polygon, MultiPolygon]) -> bool:
        """
        检查几何图形是否为凹形 (即非凸形)。
        - 如果一个几何图形 (Polygon 或 MultiPolygon) 与其凸包不相等，则认为它是凹形的。
        - 此定义下，带空洞的多边形也被视为凹形。
        - 对于空的几何图形，通常认为它是凸的 (非凹)。
        """
        if geometry.is_valid:
            if geometry.is_empty:
                return False
            return not geometry.equals(geometry.convex_hull)
        else:
            raise BadPolygonError("Bad Polygon")

    @staticmethod
    def check(geometry: Union[Polygon, MultiPolygon]) -> PolygonInspectInfo:
        """
        检查并返回一份完整的ValidateInfo。
        1. 类型检查，非 Polygon/MultiPolygon 抛 InvalidGeometryTypeError。
        2. 记录 geometry.geom_type 与 geometry.is_valid。
        3. 如果有效，再调用其他检查项。否则直接跳过这两项。

        参数:
            geometry: 要检查的几何对象实例。

        返回:
            ValidateInfo: 校验结果。

        异常:
            InvalidGeometryTypeError: 输入类型错误时直接抛出。
        """
        if not isinstance(geometry, (Polygon, MultiPolygon)):
            raise InvalidGeometryTypeError(
                f"Input must be a Polygon or MultiPolygon, not {type(geometry).__name__}"
            )

        try:
            return PolygonInspectInfo(
                geom_type=geometry.geom_type,
                is_valid=geometry.is_valid,
                has_holes=PolygonGeomChecker.has_holes(geometry),
                is_concave=PolygonGeomChecker.is_concave(geometry)
            )
        except BadPolygonError:
            return PolygonInspectInfo(
                geom_type=geometry.geom_type,
                is_valid=False,
                has_holes=None,
                is_concave=None
            )

    @staticmethod
    def check_self_intersect(input: GeoDataFrame) -> Tuple[GeoDataFrame, GeoDataFrame]:
        """
        检查多边形是否有自相交，有则返回检查结果和交点数据

        Args:
            input: 包含Polygon或MultiPolygon的GeoDataFrame

        Raises:
            ValueError: GeoDataFrame中必须包含列ID

        Returns:
            polygons_result: 包含检查结果的输入GeoDataFrame(列self_intersect)
            self_intersect_pts: 包含所有自相交交点的GeoDataFrame
        """        
        if 'ID' not in input.columns:
            raise ValueError("输入的 GeoDataFrame 必须包含一个名为 'ID' 的列。")

        bad_rows = []
        intersection_points = []

        polygons = input.geometry
        ids = input['ID']
        # 减少内存开销，提高效率
        for idx, (geom, pid) in enumerate(zip(polygons, ids)):
            if geom is None or geom.is_empty:
                continue

            polygon_id = pid
            # 统一处理，增加兼容性
            parts = []
            if geom.geom_type == 'Polygon':
                parts = [geom]
            elif geom.geom_type == 'MultiPolygon':
                parts = list(geom.geoms)
            else:
                continue  # 其他类型跳过

            part_validity = []
            part_points = []

            for poly in parts:
                # 无效图形，且无效理由为自相交时处理
                if not poly.is_valid and "Self-intersection" in explain_validity(poly):
                    points = PolygonGeomChecker._find_intersect_points(poly)
                    if points:
                        part_validity.append(True)
                        part_points.extend(points)

            if part_validity:
                # 只要有一个多边形自相交，即认为整体有问题
                bad_rows.append(True)
                for pt in part_points:
                    intersection_points.append({'ID': polygon_id, 'geometry': pt})
            else:
                bad_rows.append(False)

        # 保存结果
        if intersection_points:
            self_intersect_pts = GeoDataFrame(intersection_points, geometry="geometry", crs=input.crs)
        else:
            self_intersect_pts = GeoDataFrame(columns=['ID', 'geometry'], geometry="geometry", crs=input.crs)

        polygons_result = input.copy()
        polygons_result['self_intersect'] = bad_rows
        return polygons_result, self_intersect_pts

    @staticmethod
    def _find_intersect_points(polygon: Polygon):
        coords = list(polygon.exterior.coords)
        n = len(coords) - 1  # 最后一个点和第一个点相同
        lines = [LineString([coords[i], coords[i+1]]) for i in range(n)]
        intersections = set()
        
        # 使用集合存储当前线的交点WKB，以确保唯一性
        cur_line_interect_pts_wkb: set[bytes] = set()

        # 使用STRtree加速非相邻线段的相交查询
        tree = STRtree(lines)

        for i, line1 in enumerate(lines):
            candidate_indices = tree.query(line1)
            for j in candidate_indices:
                # 排除首尾相邻线段，且只检查非相邻且唯一的线段对
                if (i == 0 and j == n-1) or (j <= i + 1):
                    continue
                line2 = lines[j]
                tmp_pts = extract_line_intersection_points(line1, line2)
                # 合并点集
                cur_line_interect_pts_wkb = cur_line_interect_pts_wkb | tmp_pts
            
            # 将收集到的唯一交点（从WKB转换回Point）添加到总列表
            for point_wkb in cur_line_interect_pts_wkb:
                intersections.add(wkb_loads(point_wkb))

        return set(intersections)
        

