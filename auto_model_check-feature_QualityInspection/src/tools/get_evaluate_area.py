import os.path
import shutil
from typing import Union

import geojson
import geopandas as gpd
from geopandas import GeoDataFrame
from shapely.geometry import LineString, Polygon, Point
from shapely.geometry import mapping
from shapely.ops import unary_union

from src.tools.common import ensure_single_geometries
from src.logger import logger


def generate_convex_hull(file_paths: list[str], output_file_path=None) -> Polygon:
    """
    根据传入的文件读取其中的geo对象并且计算凸包，注意文件必须是经纬度坐标系
    :param file_paths: 一个包含文件路径的列表，支持的文件格式包括shp和geojson
    :param output_file_path: （可选）输出结果保存的文件路径，如果提供，将以GeoJSON格式保存凸包结果
    :return: 返回合并所有几何对象后的凸包（Polygon类型）
    """

    # 将所有真值几何对象放入一个列表
    all_geometries = []

    for file_path in file_paths:
        if os.path.exists(file_path):
            # 读取任何支持的矢量文件格式（包括shp和geojson）
            gdf = gpd.read_file(file_path)
            all_geometries += gdf['geometry'].tolist()

    # 合并所有几何对象
    merged_geometry = unary_union(all_geometries)
    # 计算凸包
    convex_hull: Polygon = merged_geometry.convex_hull

    if output_file_path:
        logger.debug(f"保存生成凸包几何文件到：{output_file_path}")
        with open(output_file_path, 'w') as f:
            geojson.dump(geojson.Feature(geometry=mapping(convex_hull)), f)

    return convex_hull


def prune_by_exact_area(input_gdf: GeoDataFrame, exact_area: Polygon) -> GeoDataFrame:
    """
    根据指定的范围（exact_area）裁剪输入的GeoDataFrame，只保留与该范围相交的几何对象
    :param input_gdf: 输入的GeoDataFrame，包含需要进行裁剪的几何对象。支持Polygon、LineString和Point类型。
    :param exact_area: 作为裁剪区域的多边形对象（Polygon类型）。
    :return: 返回一个新的GeoDataFrame，其中只包含与exact_area相交的几何对象，保留原属性。
    """
    # 检查GeoDataFrame的CRS
    # 确保输入的GeoDataFrame有一个有效的CRS
    if input_gdf.crs is None:
        logger.warning("输入的GeoDataFrame缺少CRS信息，已设置为4326")
        input_gdf.set_crs(4326, allow_override=True)

    # 将 GeoDataFrame 中的几何对象转换为单一的 Polygon 和 LineString。
    input_gdf = ensure_single_geometries(input_gdf)

    # 将exact_area转换为GeoDataFrame以便处理CRS
    exact_area_gdf = gpd.GeoDataFrame(geometry=[exact_area], crs='EPSG:4326')

    # 如果输入的GeoDataFrame的CRS与EPSG:4326不同，则将exact_area转换到相同的CRS
    if input_gdf.crs.to_epsg() != 4326:
        exact_area_gdf = exact_area_gdf.to_crs(input_gdf.crs)
        exact_area = exact_area_gdf['geometry'].tolist()[0]

    # 创建一个新的GeoDataFrame用于存储处理后的数据，不然属性就丢了...
    processed_gdf = gpd.GeoDataFrame(columns=input_gdf.columns, crs=input_gdf.crs)

    # TODO 不知道为什么第二次循环，processed_gdf的CRS会被置空
    # 遍历GeoDataFrame中的每个索引和几何对象
    for idx, row in input_gdf.iterrows():
        geom = row.geometry
        if isinstance(geom,LineString):
            intersection = geom.intersection(exact_area)
            if not intersection.is_empty:
                # 保留这一个对象的属性，使用pandas内置copy方法
                new_row = row.copy()
                new_row.geometry = intersection
                # 此处有大坑，用loc为GeoDataframe新增行 ref:https://blog.csdn.net/2403_84491977/article/details/142345238
                processed_gdf.loc[len(processed_gdf)] = new_row
        elif isinstance(geom,Polygon):
            intersection = geom.intersection(exact_area)
            if intersection:
                new_row = row.copy()
                new_row.geometry = intersection
                # 此处有大坑，用loc为GeoDataframe新增行 ref:https://blog.csdn.net/2403_84491977/article/details/142345238
                processed_gdf.loc[len(processed_gdf)] = new_row
        elif isinstance(geom,Point):
            if geom.within(exact_area):
                new_row = row.copy()
                new_row.geometry = geom
                # 此处有大坑，用loc为GeoDataframe新增行 ref:https://blog.csdn.net/2403_84491977/article/details/142345238
                processed_gdf.loc[len(processed_gdf)] = new_row
        else:
            raise Exception(type(geom))

    # 防止被置空...显式地重新设置CRS...
    processed_gdf.set_crs(input_gdf.crs, inplace=True)
    # 返回处理后的GeoDataFrame
    return processed_gdf


def prune_and_save(input_file_path: str, convex_hull: Union[Polygon, str], pruned_file_save_path: str = None):
    """
    根据传入的凸包裁剪输入的文件，并将裁剪结果保存到指定路径。支持的文件格式仅为 .shp 和 .geojson。
    :param input_file_path: 输入文件路径（shp 或 geojson 格式）。
    :param convex_hull: 裁剪所使用的凸包，可以是 Polygon 对象或凸包文件路径（shp 或 geojson 格式）。
    :param pruned_file_save_path: 裁剪结果的保存路径。如果为 None，则默认保存至与输入文件相同的路径，文件名前缀加上 '_pruned'。
    :return: 无返回值，结果保存至文件。
    """

    # 读取输入文件并确保数据的有效性
    input_gdf = gpd.read_file(input_file_path)
    if input_gdf.empty:
        raise ValueError(f"输入的GeoDataFrame为空，无法处理文件: {input_file_path}")

    # 处理凸包
    if isinstance(convex_hull, str):
        # 如果传入的是凸包文件路径，则读取该文件
        logger.debug(f"已指定此路口凸包文件：{convex_hull}")
        convex_hull_gdf = gpd.read_file(convex_hull)
        if convex_hull_gdf.empty:
            raise ValueError(f"读取凸包文件失败或文件为空: {convex_hull}")
        convex_hull_geom = convex_hull_gdf['geometry'].iloc[0]
    else:
        # 如果传入的是 Polygon 对象，则直接使用
        logger.debug(f"已指定此路口凸包几何")
        if not isinstance(convex_hull, Polygon):
            raise TypeError(f"convex_hull 应为 Polygon 类型，当前类型为 {type(convex_hull)}")
        convex_hull_geom = convex_hull

    # 调用 prune_by_exact_area 函数进行裁剪
    pruned_gdf = prune_by_exact_area(input_gdf, convex_hull_geom)

    # 确定保存路径和文件格式
    if pruned_file_save_path:
        save_path = pruned_file_save_path
        file_extension = os.path.splitext(pruned_file_save_path)[1]
    else:
        # 如果没有指定保存路径，则根据输入文件路径生成默认路径
        file_extension = os.path.splitext(input_file_path)[1]
        base_name = os.path.splitext(input_file_path)[0]
        save_path = f"{base_name}_pruned{file_extension}"

    # 检查文件格式是否支持
    if file_extension.lower() not in ['.shp', '.geojson']:
        raise ValueError("仅支持 .shp 和 .geojson 格式进行保存。")

    # 根据文件格式保存裁剪后的结果
    if file_extension.lower() == '.geojson':
        pruned_gdf.to_file(save_path, driver='GeoJSON')
    else:
        # 默认为 .shp 格式
        pruned_gdf.to_file(save_path)

    # 输出日志信息，指示文件保存位置
    logger.info(f"裁剪后的文件已保存至：{save_path}")

def prune_files_in_directory(input_dir: str, convex_hull: Union[Polygon, str], output_dir: str = None):
    """
    遍历目录，将所有 .shp 和 .geojson 文件根据传入的凸包进行剪裁，并将结果保存。
    如果输出目录为空，则将结果保存到输入文件所在目录，文件名前加 '_pruned' 标签。
    只支持 .shp 和 .geojson 格式。
    """

    # 获取所有不带 '_pruned' 的 .shp 和 .geojson 文件。跳过带有pruned标签的文件
    file_list = [
        os.path.join(input_dir, f) for f in os.listdir(input_dir)
        if (f.endswith('.shp') or f.endswith('.geojson')) and '_pruned' not in f
    ]

    # 检查凸包类型
    if isinstance(convex_hull, str):
        logger.debug(f"已指定此路口凸包文件：{convex_hull}")
        convex_hull_gdf = gpd.read_file(convex_hull)
        convex_hull_geom = convex_hull_gdf['geometry'].tolist()[0]
    else:
        logger.debug(f"已指定此路口凸包几何")
        convex_hull_geom = convex_hull

    # 遍历文件并裁剪
    for file_path in file_list:
        try:
            logger.info(f"处理文件：{file_path}")

            # 读取地理数据文件
            input_gdf = gpd.read_file(file_path)
            # # 将 GeoDataFrame 中的几何对象转换为单一的 Polygon 和 LineString。
            # input_gdf = ensure_single_geometries(input_gdf)

            # 裁剪数据
            pruned_gdf = prune_by_exact_area(input_gdf, convex_hull_geom)

            # 确定保存路径和格式
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
                base_name = os.path.splitext(os.path.basename(file_path))[0]
                save_path = os.path.join(output_dir, f"{base_name}_pruned{os.path.splitext(file_path)[1]}")
            else:
                save_path = f"{os.path.splitext(file_path)[0]}_pruned{os.path.splitext(file_path)[1]}"

            # 保存裁剪后的文件
            file_extension = os.path.splitext(save_path)[1].lower()
            if file_extension not in ['.shp', '.geojson']:
                logger.warning(f"跳过不支持的文件格式：{file_path}")
                continue

            pruned_gdf.to_file(save_path, driver='GeoJSON' if file_extension == '.geojson' else None)
            logger.info(f"剪裁后的文件已保存至：{save_path}")

            # 将源文件移动到 origin 文件夹中
            origin_dir = os.path.join(os.path.dirname(file_path), "origin")
            os.makedirs(origin_dir, exist_ok=True)  # 新建 origin 文件夹（如果不存在）

            # 如果是 Shapefile，则移动所有相关文件
            if file_extension.lower() == '.shp':
                base_name = os.path.splitext(file_path)[0]
                extensions = ['.shp', '.shx', '.dbf', '.prj', '.cpg', '.qix', '.sbn', '.sbx', '.fbn', '.fbx', '.ain',
                              '.aih']
                for ext in extensions:
                    file_to_move = base_name + ext
                    if os.path.exists(file_to_move):
                        origin_file_path = os.path.join(origin_dir, os.path.basename(file_to_move))
                        shutil.move(file_to_move, origin_file_path)
                        logger.info(f"源文件已移动至：{origin_file_path}")
            else:
                # 对于非 Shapefile，将输入文件直接移动
                origin_file_path = os.path.join(origin_dir, os.path.basename(file_path))
                shutil.move(file_path, origin_file_path)
                logger.info(f"源文件已移动至：{origin_file_path}")

        except Exception as e:
            logger.error(f"处理文件 {file_path} 时出错：{e}")
