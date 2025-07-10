# -*- coding: utf-8 -*-
# @Time    : 2025/3/4 21:20
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : cut_by_algo_box

"""
71951   氦
71952   锂
72377   铍
72001   硼
71699   氖
"""

import os
import re
import sys
import logging
from pathlib import Path
from typing import Union
import geopandas as gpd
from shapely.geometry import Polygon, MultiPolygon

from src.app_path import resource_path
from src.logger import logger

#
# 获取当前文件所在目录的绝对路径
current_dir = os.path.abspath(os.path.dirname(__file__))

# 构建 _data_dir 的路径
__data_dir = resource_path("data")


def process_shapefile(shp_file: Path, algo_geom: Union[Polygon, MultiPolygon]) -> gpd.GeoDataFrame:
    """
    读取单个 shapefile 文件，并根据以下规则进行处理：
      - 线要素：与算法框相交则剪裁，只保留位于算法框内的部分；
      - 面要素：如果面要素的质心在算法框内，则保留该要素（不进行剪裁）。
    如果文件中的几何类型不为线或面，则跳过处理。

    :param shp_file: shapefile 文件路径（仅处理 .shp 文件，依赖文件需与之同名）
    :param algo_geom: 算法框几何（Polygon 或 MultiPolygon）
    :return: 处理后的 GeoDataFrame（如果没有符合条件的要素，则返回一个空的 GeoDataFrame）
    """
    # FIXME 目前默认坐标系一样。报错暂时忽略，出与健壮性考虑后面需要对齐坐标系在进行剪裁。
    # 默认坐标系一样。
    # UserWarning: Geometry is in a geographic CRS. Results from 'centroid' are likely incorrect.
    # Use 'GeoSeries.to_crs()' to re-project geometries to a projected CRS before this operation.
    # 的报错可暂时忽略。

    try:
        logger.debug(f"尝试读取文件: {shp_file}")
        gdf = gpd.read_file(shp_file)
    except Exception as e:
        logger.error(f"读取 {shp_file.name} 出错: {e}")
        return gpd.GeoDataFrame(columns=['geometry'])

    if gdf.empty:
        logger.info(f"{shp_file.name} 没有数据，跳过。")
        return gdf

    # 获取几何类型（假定一个文件中的所有要素几何类型一致）
    geom_type = gdf.geometry.geom_type.iloc[0]
    logger.debug(f"文件 {shp_file.name} 的几何类型为: {geom_type}")

    result = gpd.GeoDataFrame(columns=gdf.columns, crs=gdf.crs)
    # 处理线要素
    if geom_type in ("LineString", "MultiLineString"):
        try:
            clipped = gpd.clip(gdf, algo_geom)
            result = clipped
        except Exception as e:
            logger.error(f"剪裁 {shp_file.name} 时出错: {e}")
    # 处理面要素
    elif geom_type in ("Polygon", "MultiPolygon"):
        try:
            # 判断面要素的质心是否在算法框内
            within_mask = gdf.geometry.centroid.within(algo_geom)
            subset = gdf[within_mask]
            result = subset
        except Exception as e:
            logger.error(f"筛选 {shp_file.name} 中面要素时出错: {e}")

    # 处理点要素
    elif geom_type in ("Point", "MultiPoint"):
        try:
            # 对于点要素，直接判断点是否在算法框内
            within_mask = gdf.geometry.within(algo_geom)
            subset = gdf[within_mask]
            result = subset
        except Exception as e:
            logger.error(f"筛选 {shp_file.name} 中点要素时出错: {e}")

    else:
        logger.warning(f"文件 {shp_file.name} 的几何类型 {geom_type} 未定义剪裁规则，跳过处理。")

    return result


def process_directory(input_dir: Union[str, Path],
                      algo_geom: Union[Polygon, MultiPolygon],
                      id_: Union[str, int], output_dir=None) -> None:
    """
    遍历给定目录下所有 .shp 文件，对每个文件执行剪裁（或筛选）操作。
    如果至少有一个文件中的要素满足规则，则在脚本所在目录下创建以 id_ 命名的目录，
    将处理后（非空）的 shapefile 文件保存进去，文件名保持不变。

    :param input_dir: 待剪裁 shapefile 文件所在目录
    :param algo_geom: 算法框几何（Polygon 或 MultiPolygon）
    :param id_: 算法框 id（用于命名输出目录）
    """
    input_dir = Path(input_dir)
    if not input_dir.is_dir():
        logger.error(f"输入目录 {input_dir} 不存在或不是一个目录。")
        return

    # 获取所有 .shp 文件
    shp_files = list(input_dir.glob("*.shp"))
    if not shp_files:
        logger.info(f"目录 {input_dir} 下没有找到 .shp 文件。")
        return

    processed_files = {}  # 存储处理后非空数据 {文件名: GeoDataFrame}

    for shp_file in shp_files:
        logger.info(f"处理文件: {shp_file.name}")
        processed_gdf = process_shapefile(shp_file, algo_geom)
        if not processed_gdf.empty:
            processed_files[shp_file.name] = processed_gdf
        else:
            logger.info(f"文件 {shp_file.name} 中无要素落在算法框内，跳过保存。")

    if not processed_files:
        logger.info("没有任何文件的要素落在指定的算法框内，跳过创建输出目录和保存文件。")
        return

    if output_dir:
        if os.path.exists(output_dir):
            output_dir = Path(output_dir) / str(id_)
    else:
        # 定义输出目录：脚本所在目录下以 id_ 命名的文件夹
        output_dir = Path(os.path.abspath(__data_dir)) / str(id_)
        logger.info(f"剪裁后保存目录为：{output_dir}")

    if isinstance(output_dir, str):
        # 检查目录是否存在，如果不存在则创建
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            logger.info(f"创建输出目录: {output_dir}")
        output_dir = Path(output_dir)

    else:
        if not output_dir.exists():
            try:
                output_dir.mkdir(parents=True, exist_ok=True)
                logger.info(f"创建输出目录: {output_dir}")
            except Exception as e:
                logger.error(f"创建输出目录 {output_dir} 失败: {e}")
                return

    # 保存每个处理后的 GeoDataFrame 到输出目录下，保持文件原名
    for filename, gdf in processed_files.items():
        # 使用正则表达式检查文件名是否以数字开头，如果是则去除此部分
        new_filename = re.sub(r'^\d+', '', filename)
        if not new_filename:
            # 如果去除数字后文件名为空，则保留原文件名
            new_filename = filename
        out_path = output_dir / new_filename
        try:
            gdf.to_file(out_path)
            logger.info(f"保存剪裁后的文件: {out_path}")
        except Exception as e:
            logger.error(f"保存文件 {out_path} 失败: {e}")


def cut_and_save(id_, input_dir, output_dir=None):
    """
    示例使用：
      1. 通过算法框管理器获取算法框几何（algo_geom），
         这里假定已存在一个 AlgorithmBoxManager 类，并能够通过 get_algo_box_by_id 得到对应几何。

      2. 指定算法框 id 和待剪裁的 shp 文件所在目录，然后执行剪裁并保存结果。

      请根据实际环境调整路径和模块导入。
    """

    # 加载算法框几何
    try:
        from src.algorithm_range.manager import AlgorithmBoxManager  # 根据实际路径和模块名称替换
    except ImportError:
        logger.error("无法导入 AlgorithmBoxManager，请检查模块路径和名称。")
        sys.exit(1)

    algo_manager = AlgorithmBoxManager(os.path.join(__data_dir, "algorithm_boxes"))
    try:
        algo_geom = algo_manager.get_algo_box_by_id(id_)
    except Exception as e:
        logger.error(f"获取算法框（id={id_}）几何失败: {e}")
        sys.exit(1)

    # 检查几何类型
    if not isinstance(algo_geom, (Polygon, MultiPolygon)):
        logger.error("加载的算法框几何不是 Polygon 或 MultiPolygon 类型。")
        sys.exit(1)

    # 执行剪裁操作
    process_directory(input_dir=input_dir, algo_geom=algo_geom, id_=id_, output_dir=output_dir)


def perform_clipping(algo_id, input_dir, output_dir=None):
    cut_and_save(algo_id, input_dir, output_dir)


if __name__ == "__main__":
    # 测试如下

    algo_box_id_list = [
        401475
    ]

    input_dir = r"E:\66_众源评测相关\（AAA最新）【勿删！！】真值数据成果\20250219_考试路线1带高程成果（V2.0规格）\20250219考试路线1数据_GCJ02"

    for id in algo_box_id_list:
        algo_box_id = id
        cut_and_save(algo_box_id,
                     input_dir=input_dir,
                     output_dir=f"E:\\Code\\GeoEva\\data\\truth\{id}")
