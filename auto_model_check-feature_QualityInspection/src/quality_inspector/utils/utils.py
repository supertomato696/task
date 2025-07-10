# -*- coding: utf-8 -*-
# @Time    : 2025/4/24 10:47
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : utils.py

"""
工具类，提供纯粹的、无状态的几何计算方法。
"""
import math
import shutil
from pathlib import Path
from typing import List, Union

import numpy as np
from shapely import Point, LineString, MultiLineString, GeometryCollection, MultiPoint, Polygon, MultiPolygon, make_valid
from shapely.geometry.base import BaseGeometry
from shapely.wkb import dumps as wkb_dumps

from src.logger import logger


def calculate_interior_angle(p1_coords: tuple, p2_coords: tuple, p3_coords: tuple) -> float:
    """
    计算由三个连续坐标点 p1, p2, p3 构成的内角（角度在顶点 p2 处）。
    使用向量法计算 p1->p2 和 p3->p2 两个向量之间的夹角。

    Args:
        p1_coords (tuple): 第一个点的坐标 (x, y)。
        p2_coords (tuple): 第二个点（顶点）的坐标 (x, y)。
        p3_coords (tuple): 第三个点的坐标 (x, y)。

    Returns:
        float: 计算得到的内角，单位为度。如果 p1, p2 或 p2, p3 重合，
               则视为直线，返回 180.0 度。
    """
    p1 = np.array(p1_coords)
    p2 = np.array(p2_coords)
    p3 = np.array(p3_coords)

    vec_ba = p1 - p2
    vec_bc = p3 - p2

    norm_ba = np.linalg.norm(vec_ba)
    norm_bc = np.linalg.norm(vec_bc)

    # 检查是否有零向量（重合点），避免除以零
    if norm_ba == 0 or norm_bc == 0:
        return 180.0  # 重合点视为直线

    # 计算点积
    dot_product = np.dot(vec_ba, vec_bc)
    # 计算夹角的余弦值，限制在 [-1, 1] 范围内防止浮点误差
    cos_theta = np.clip(dot_product / (norm_ba * norm_bc), -1.0, 1.0)
    # 使用 np.arccos 计算角度
    angle_rad = np.arccos(cos_theta)
    return math.degrees(angle_rad)


def is_small_fluctuation(p1: Point, p2: Point, p3: Point, threshold_m: float) -> bool:
    """
    判断顶点 p2 相对于线段 p1-p3 是否构成一个小波动。

    通过计算点 p2 到直线段 p1-p3 的垂直距离（即以 p1-p3 为底边的三角形 p1-p2-p3 的高），
    并将其与阈值 threshold_m 进行比较来判断。

    Args:
        p1 (Point): 前一个点。
        p2 (Point): 当前顶点。
        p3 (Point): 后一个点。
        threshold_m (float): 定义“小波动”的最大垂直距离阈值（单位：米）。
                             如果 p2 到线段 p1-p3 的距离小于等于此阈值，
                             则认为是小波动。

    Returns:
        bool: 如果 p2 到线段 p1-p3 的距离小于等于 threshold_m，返回 True (是小波动)，
              否则返回 False。如果 p1 和 p3 重合，也返回 True。
    """
    # 如果 p1 和 p3 重合，无法形成有效底边，视为小波动（或特殊情况）
    if p1.distance(p3) < 1e-9:
        return True
    # 利用海伦公式或向量叉积计算三角形面积的两倍
    area = abs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2)
    # 计算高 = 2 * 面积 / 底边长
    height = (2 * area / p1.distance(p3))
    return height <= threshold_m


def extract_single_lines(g: BaseGeometry) -> List[LineString]:
    """
    从一个几何对象（可能是 LineString/MultiLineString/GeometryCollection）中
    提取出所有 LineString 部分，扁平化为列表。
    """
    lines: List[LineString] = []
    if isinstance(g, LineString):
        lines.append(g)
    elif isinstance(g, MultiLineString):
        lines.extend(g.geoms)
    elif isinstance(g, GeometryCollection):
        for part in g.geoms:
            if isinstance(part, (LineString, MultiLineString, GeometryCollection)):
                lines.extend(extract_single_lines(part))
    return lines


def get_line_endpoints(line: LineString):
    return [Point(line.coords[0]), Point(line.coords[-1])]


def is_endpoint(line: LineString, pt: Point, tolerance: float = 1e-8):
    for ep in get_line_endpoints(line):
        if pt.distance(ep) <= tolerance:
            return True
    return False


def extract_line_intersection_points(line: LineString, geom: Union[LineString, Polygon, MultiPolygon]):
    interect_pts_wkb: set[bytes] = set()
    # 全都先检查一遍是否为合法集合
    geom = validate_geometry(geom)
    line = validate_geometry(line)
    # 没有交集
    if not line.intersects(geom):
        return interect_pts_wkb

    intersection_result = line.intersection(geom)
    # 交集误判
    if intersection_result.is_empty:
        return interect_pts_wkb

    if isinstance(intersection_result, Point):
        # 相交是点
        # if not (is_endpoint(line, intersection_result) and is_endpoint(geom, intersection_result)):
        #     # 交点不是两根线的正常端点
        interect_pts_wkb.add(wkb_dumps(intersection_result))
    elif isinstance(intersection_result, MultiPoint):
        # 相交是多个点
        for point in intersection_result.geoms:  # .geoms for MultiPoint
            if isinstance(point, Point):
                interect_pts_wkb.add(wkb_dumps(point))
    # 注意: 如果交集是LineString (重叠)，当前逻辑不将其视为"相交点"。
    # 如果需要处理重叠线段的端点作为交点，需要额外逻辑。
    elif isinstance(intersection_result, LineString):
        # 相交是线
        coords = list(intersection_result.coords)
        if len(coords) >= 2:
            # 把线的端点假如到交集中
            interect_pts_wkb.add(wkb_dumps(Point(coords[0])))
            interect_pts_wkb.add(wkb_dumps(Point(coords[-1])))
    elif isinstance(intersection_result, MultiLineString):
        # 相交是多根线
        for subline in intersection_result.geoms:
            coords = list(subline.coords)
            if len(coords) >= 2:
                # 把线的端点假如到交集中
                interect_pts_wkb.add(wkb_dumps(Point(coords[0])))
                interect_pts_wkb.add(wkb_dumps(Point(coords[-1])))

    return interect_pts_wkb


def archive_inspect_output_files(source_dir_path: Union[str, Path]) -> List[Path]:
    """
    后处理函数：将指定目录中除 'output.json' 外的所有文件移动到该目录下的 'upload' 子目录中。

    Args:
        source_dir_path: 要处理的源目录路径。

    Returns:
        List[Path]: 一个包含了所有被成功移动的文件新路径的列表。
    """
    if not isinstance(source_dir_path, Path):
        source_dir = Path(source_dir_path)
    else:
        source_dir = source_dir_path
    archive_dir = source_dir / "upload"
    file_to_keep = "output.json"
    moved_files: List[Path] = []

    # 1. 健壮性检查：确保源目录存在
    if not source_dir.is_dir():
        logger.warning(f"[后处理] 源目录不存在，跳过后处理: '{source_dir}'")
        return moved_files

    # 2. 准备归档目录：如果不存在，则创建它
    try:
        archive_dir.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        logger.error(f"[后处理] 无法创建归档目录 '{archive_dir}': {e}")
        raise Exception(f"[后处理] 无法创建归档目录 '{archive_dir}': {e}")

    # 3. 遍历源目录中的所有条目
    logger.debug(f"[后处理] 开始扫描目录: '{source_dir}'")
    for item_path in source_dir.iterdir():
        # 4. 应用过滤规则跳过我们想保留的文件
        if item_path.name == file_to_keep:
            logger.debug(f"[后处理] 跳过保留文件: '{item_path.name}'")
            continue

        # 跳过 'upload' 目录本身，防止无限递归或错误
        if item_path.is_dir() and item_path.samefile(archive_dir):
            continue

        # 我们只处理文件，跳过任何其他子目录
        if not item_path.is_file():
            logger.debug(f"[后处理] 跳过非文件条目: '{item_path.name}'")
            continue

        # 5. 执行移动操作
        try:
            logger.info(f"[后处理] 准备移动: '{item_path.name}' -> '{archive_dir}'")
            # shutil.move(源文件, 目标目录)
            destination_path = archive_dir / item_path.name
            shutil.move(str(item_path), str(destination_path))   #  [后处理] 移动文件 '63762_5661615_output.json' 失败: Destination path '/mnt/tasks/model/label_cover/output/upload/63762_5661615_output.json' already exists
            moved_files.append(destination_path)
        except Exception as e:
            logger.error(f"[后处理] 移动文件 '{item_path.name}' 失败: {e}")

    if moved_files:
        logger.success(f"[后处理] 成功移动 {len(moved_files)} 个文件到 '{archive_dir}'")
    else:
        logger.info("[后处理] 确认没有需要移动的文件。")

    return moved_files

def validate_geometry(geom: BaseGeometry) -> BaseGeometry:
    if geom.is_valid:
        return geom
    
    try:
        logger.info('无效geometry, 尝试自修复')
        # 先用buffer(0)自修复
        fixed_geom = make_valid(geometry=geom)
    except Exception as e:
        logger.error(f'自修复geometry异常: {e}')
        return geom
    
    if fixed_geom.is_valid:
        return fixed_geom
    else:
        logger.warning("自修复后geometry依旧无效, 无法自修复")
        return geom