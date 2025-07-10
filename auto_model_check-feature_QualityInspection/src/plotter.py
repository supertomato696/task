# -*- coding: utf-8 -*-
# @Time    : 2025/6/9 11:35
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : plotter.py

import os
import secrets
import string
import platform
from typing import Optional, List, Union, Literal

# matplotlib.use('agg')
import matplotlib.pyplot as plt
from geopandas import GeoDataFrame
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from pydantic import BaseModel, Field

from src.data_process.file_read import DataReader
from src.logger import logger


# 根据操作系统设置字体
system = platform.system()  # 获取操作系统名称

if system == "Windows":
    # Windows系统字体设置
    plt.rcParams['font.sans-serif'] = [
        'Microsoft YaHei',  # 微软雅黑
        'SimHei'  # 黑体
    ]
elif system == "Linux":
    # Linux 系统字体设置
    plt.rcParams['font.sans-serif'] = [
        'WenQuanYi Zen Hei',  # 文泉驿正黑
        'Noto Sans CJK SC',  # 谷歌思源字体（需系统安装）
        'AR PL UMing CN',  # AR PL 字体（部分 Linux 发行版预装）
        'SimSun',  # 宋体，可能需要手动安装
        'DejaVu Sans'  # Matplotlib 默认字体
    ]
else:
    # 默认字体设置（其他未知系统）
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']


def generate_secure_random_string(length=8):
    """生成一个由字母组成的随机字符串"""
    letters = string.ascii_letters  # 包含所有字母
    return ''.join(secrets.choice(letters) for _ in range(length))


class BaseLayerConfig(BaseModel):
    """
    图层配置的基类，定义所有图层共有的属性。预设低调样式。

    Attributes:
        gdf (GeoDataFrame): 要绘制的地理数据本身，它是一个GeoDataFrame对象。
        name (str): 图层的名称，这个名称会显示在地图的图例（Legend）中。也作为导出gpkg的图层名。
        zorder (int): 图层的堆叠顺序 。zorder 值越大，图层越靠上。如果不提供，程序会自动分配，确保主要图层在参考图层之上。
        alpha (float): 图层的透明度，一个从 0.0 (完全透明) 到 1.0 (完全不透明) 的浮点数。
    """
    gdf: GeoDataFrame
    name: str = None
    zorder: Optional[int] = Field(default=0)
    alpha: Optional[float] = Field(default=0.3)  # 透明度是通用属性
    color: str = None

    facecolor: str = None
    edgecolor: str = None
    linewidth: float = 0.7

    class Config:
        arbitrary_types_allowed = True


# 专门用于点图层的子类
class PointConfig(BaseLayerConfig):
    """
    点图层的专属配置。
    Attributes:
        gdf (GeoDataFrame): 要绘制的地理数据本身，它是一个GeoDataFrame对象。
        name (str): 图层的名称，这个名称会显示在地图的图例（Legend）中。也作为导出gpkg的图层名。
        zorder (int): 图层的堆叠顺序 。zorder 值越大，图层越靠上。如果不提供，程序会自动分配，确保主要图层在参考图层之上。
        alpha (float): 图层的透明度，一个从 0.0 (完全透明) 到 1.0 (完全不透明) 的浮点数。

        color (str): 图层的主颜色。
        marker (str): 点的样式：默认为"o"。也可设置为其他标记样式如 'o', 's', '*'。
        markersize (float): 点的大小，默认为50。
        edgecolor (str): 描边颜色。默认无描边颜色。
    """
    color: str = '#EAEAEA'
    marker: str = 'o'
    markersize: float = 50
    edgecolor: Optional[str] = None  # 点也可以有边框色


# 专门用于线图层的子类
class LineConfig(BaseLayerConfig):
    """
    线图层的专属配置。
    Attributes:
        gdf (GeoDataFrame): 要绘制的地理数据本身，它是一个GeoDataFrame对象。
        name (str): 图层的名称，这个名称会显示在地图的图例（Legend）中。也作为导出gpkg的图层名。
        zorder (int): 图层的堆叠顺序 。zorder 值越大，图层越靠上。如果不提供，程序会自动分配，确保主要图层在参考图层之上。
        alpha (float): 图层的透明度，一个从 0.0 (完全透明) 到 1.0 (完全不透明) 的浮点数。

        color (str): 图层的主颜色。
        linestyle (str): 线的样式。默认实线，但必须是以下预设选项之一:
                            - '-' 或 'solid': 实线
                            - '--' 或 'dashed': 虚线
                            - '-.' 或 'dashdot': 点划线
                            - ':' 或 'dotted': 点线
        linewidth (float): 线的宽度，默认为1.5。
        merge_buffer_distance (float): 执行融合概化时的缓冲距离，单位为米，默认为None。
        # edgecolor (str): 描边颜色。默认无描边颜色。
    """
    color: str = '#EAEAEA'
    linewidth: float = 1.5
    # 使用 Literal 来定义一组预设的线型选项
    linestyle: Optional[Literal['-', '--', '-.', ':', 'solid', 'dashed', 'dotted', 'dashdot']] = '-'
    merge_buffer_distance: Optional[float] = Field(default=0.10)  # 执行融合概化时的缓冲距离，单位为米。


# 专门用于面图层的子类
class PolygonConfig(BaseLayerConfig):
    """
    面图层的专属配置。
    Attributes:
        gdf (GeoDataFrame): 要绘制的地理数据本身，它是一个GeoDataFrame对象。
        name (str): 图层的名称，这个名称会显示在地图的图例（Legend）中。也作为导出gpkg的图层名。
        zorder (int): 图层的堆叠顺序 。zorder 值越大，图层越靠上。如果不提供，程序会自动分配，确保主要图层在参考图层之上。
        alpha (float): 图层的透明度，一个从 0.0 (完全透明) 到 1.0 (完全不透明) 的浮点数。

        color (str): 图层的主颜色。对于面，它会同时设置填充色和边框色。
        facecolor (str): 设置面的填充颜色，默认为空。
        edgecolor (str): 设置几何图形的边框颜色，默认为空。
        linewidth (str): 设置边框的粗细，默认为空。
        hatch (str): 面的填充图案，默认为空，接受：['/', '\\', '|', '-', '+', 'x', 'o', 'O', '.', '*']

    """
    color: str = '#EAEAEA'
    facecolor: Optional[str] = None
    edgecolor: Optional[str] = '#BDBDBD'
    linewidth: Optional[float] = None
    # 使用 Literal 来定义一组预设的填充图案选项
    hatch: Optional[Literal['/', '\\', '|', '-', '+', 'x', 'o', 'O', '.', '*']] = None


class GeoEvaPlotter:
    """
    一个配置驱动的地理数据制图工具，用于简化和标准化地图绘制流程。
    """

    def __init__(self, figsize=(24, 18), crs=None, interactive_show=False,
                 line_generalization_threshold: Optional[int] = None):
        """
        初始化绘图工具。

        Args:
            figsize (tuple, optional): Matplotlib图形的尺寸。默认为 (16, 16)。
            crs (str, optional): 所有图层最终统一到的目标坐标系。默认为 "EPSG:4326" (WGS84)。
            interactive_show: 是否弹窗。
            line_generalization_threshold: 线要素融合概化的阈值。
        """
        self.figsize = figsize
        self.target_crs = crs if crs else "EPSG:4326"
        self.interactive_show = interactive_show
        self.line_generalization_threshold = line_generalization_threshold

        if self.line_generalization_threshold:
            logger.info(
                f"[Plotter]初始化完毕，默认crs=4326，同个图层超过{self.line_generalization_threshold}条线将会被合并为一个面")
        else:
            logger.info(f"[Plotter]初始化完毕，默认crs=4326")

    def __handle_main_layers(self, main_layers: List[BaseLayerConfig]) -> List[BaseLayerConfig]:

        all_validated_layers: List[BaseLayerConfig] = []

        for i, layer in enumerate(main_layers):
            if layer.zorder is None:
                layer.zorder = i + 1  # 保证在参考图层之上
            if layer.gdf.empty:
                logger.warning(f"[Plotter]图层{layer.name}传入为空，不做渲染")
                continue

            # 假如需要融合概化
            if isinstance(layer, LineConfig) and self.line_generalization_threshold is not None:
                # 保证墨卡托
                if not layer.gdf.crs.coordinate_operation.method_name == "Transverse Mercator":
                    if not layer.gdf.crs.equals("EPSG:4326"):
                        raise ValueError("不明坐标系！")
                    else:
                        layer.gdf = DataReader().handle_crs(layer.gdf)

                logger.info("[Plotter]Merging Lines!")
                # 步骤 1: 将所有线缓冲成面
                buffered_geometries = layer.gdf.buffer(layer.merge_buffer_distance)

                # 步骤 2: 将所有缓冲后的面合并成一个单一的几何对象
                merged_geom = buffered_geometries.union_all()

                # 步骤 3: 用这个新的单一几何对象创建一个新的 GeoDataFrame
                # 这个新的 GDF 只有一个要素，就是我们融合后的结果
                layer.gdf = GeoDataFrame(geometry=[merged_geom], crs=layer.gdf.crs)

            # 统一坐标系，默认转4326坐标系
            if layer.gdf.crs != self.target_crs:
                layer.gdf = layer.gdf.to_crs(self.target_crs)
            all_validated_layers.append(layer)

        return all_validated_layers

    def __handle_ref_layers(self, ref_layers: Optional[List[Union[BaseLayerConfig, GeoDataFrame]]]) -> List[
        BaseLayerConfig]:

        all_validated_layers: List[BaseLayerConfig] = []

        # 处理参考图层
        if ref_layers:
            for i, layer_input in enumerate(ref_layers):
                if isinstance(layer_input, GeoDataFrame):
                    # 如果是GDF，默认按面图层样式处理
                    if layer_input.empty:
                        logger.warning(f"[Plotter]图层传入为空，不做渲染")
                        continue

                    # 统一坐标系，默认转4326坐标系
                    if layer_input.crs != self.target_crs:
                        layer_input = layer_input.to_crs(self.target_crs)
                    all_validated_layers.append(
                        BaseLayerConfig(
                            gdf=layer_input
                        )
                    )
                elif isinstance(layer_input, BaseLayerConfig):
                    # 如果已经是BaseLayerConfig，直接使用，但确保有zorder
                    if layer_input.zorder is None:
                        layer_input.zorder = 0  # 默认参考图层在最底层
                        # 如果是GDF，默认按面图层样式处理
                        if layer_input.empty:
                            logger.warning(f"[Plotter]图层传入为空，不做渲染")
                            continue

                        # 统一坐标系，默认转4326坐标系
                        if layer_input.crs != self.target_crs:
                            layer_input = layer_input.to_crs(self.target_crs)

                    all_validated_layers.append(layer_input)

        return all_validated_layers

    def plot(self,
             title: str,
             main_layers: List[BaseLayerConfig],
             ref_layers: Optional[List[Union[BaseLayerConfig, GeoDataFrame]]] = None,
             text: str = None,
             output_svg_path: Optional[str] = None):

        fig, ax = plt.subplots(figsize=self.figsize)
        legend_handles = []  # 初始化空列表，用于手动收集图例项
        gdfs_to_save = {}  # 用于存储处理后待保存的GDF

        # 1. 处理参考图层
        validated_ref_layers = self.__handle_ref_layers(ref_layers)

        # 2. 处理主要图层 (已经是LayerConfig，只需检查zorder)
        validated_main_layers = self.__handle_main_layers(main_layers)

        # 汇总处理完的图层
        all_validated_layers = validated_ref_layers + validated_main_layers

        # 按zorder排序后绘制
        sorted_list_by_zorder = sorted(all_validated_layers, key=lambda x: x.zorder)

        for index, layer in enumerate(sorted_list_by_zorder):
            _gdf = layer.gdf

            gdfs_to_save[layer.name] = _gdf
            plot_kwargs = layer.model_dump(exclude={'gdf', 'name', 'merge_buffer_distance'}, exclude_none=True)

            _gdf.plot(ax=ax, **plot_kwargs)
            layer_display_name = layer.name if layer.name else "参考图层"
            logger.debug(
                f"[Plotter] 绘图进度 {index+1}/{len(sorted_list_by_zorder)}: "
                f"{layer_display_name} (图层顺序={layer.zorder})"
            )

            # 手动创建图例handler
            geom_type = _gdf.geometry.geom_type.iloc[0] if not _gdf.empty else None
            handle = None
            if 'Polygon' in str(geom_type):
                handle = Patch(facecolor=plot_kwargs.get('facecolor', 'gray'),
                               edgecolor=plot_kwargs.get('edgecolor', 'black'), label=layer.name,
                               alpha=plot_kwargs.get('alpha', 1.0))
            elif 'LineString' in str(geom_type):
                handle = Line2D([0], [0], color=plot_kwargs.get('color', 'blue'),
                                linewidth=plot_kwargs.get('linewidth', 1.5), label=layer.name)
            elif 'Point' in str(geom_type):
                handle = Line2D([0], [0], marker=plot_kwargs.get('marker', 'o'), color=plot_kwargs.get('color', 'red'),
                                markersize=10, linestyle='None', label=layer.name)
            if handle and layer.name is not None:
                legend_handles.append(handle)

        # 设置标题和坐标轴标签
        ax.set_title(title, fontsize=20)
        ax.grid(True, linestyle='--', alpha=0.6)

        ax.text(1.01, 0.5, text, transform=ax.transAxes, fontsize=10,
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # 使用手动创建的 handles
        if legend_handles:
            ax.legend(handles=legend_handles, title="Legends")

        # 将所有产出逻辑移到最外层，确保总是执行
        if output_svg_path:
            try:
                logger.debug(f"[Plotter]开始保存数据到: {output_svg_path}")
                fig.savefig(output_svg_path, format='svg', bbox_inches='tight', dpi=150)
                logger.info(f"[Plotter]已成功保存为SVG: {output_svg_path}")
            except Exception as e:
                logger.error(f"[Plotter]保存SVG失败: {e}")

        if self.interactive_show:
            plt.show()

        plt.close(fig)
        logger.info("[Plotter]绘图完成")

    def save_gpkg(self, layers_raw: Optional[List[Union[BaseLayerConfig, GeoDataFrame]]], output_gpkg_path: str):
        validated_layers = self.__handle_ref_layers(layers_raw)

        # 确保目标目录存在
        output_dir = os.path.dirname(output_gpkg_path)
        os.makedirs(output_dir, exist_ok=True)

        # 逐一将GeoDataFrame写入同一个gpkg中
        for _, item in enumerate(validated_layers):
            if item.name is None:
                item.name = generate_secure_random_string
                logger.warning(f"[Plotter]尝试写入一个空名字的图层，将会分配随机名称{item.name}")

            try:
                logger.info(f"[Plotter]写入图层：{item.name}")
                item.gdf.to_file(output_gpkg_path, layer=item.name, driver='GPKG')
            except Exception as e:
                logger.error(f"写入图层{item.name}失败: {e}")

        logger.info(f"[Plotter]所有图层已成功打包到GeoPackage: {output_gpkg_path}")


if __name__ == '__main__':
    pass
    ## sample:
    #
    # ref_gdf = gdf = DataReader().get_this_file_ready(r"/data/input/KXF/400849/STOP_LINE.shp")
    #
    # main_layers_list = [
    #     LineConfig(
    #         gdf=DataReader().get_this_file_ready(r"/data/input/KXF/400849/ROAD_LINE.shp"),
    #         name='ROAD_LINE',
    #         color='blue',
    #         linewidth=1.5,
    #         zorder=3  # 放在最顶层
    #     ),
    #     PolygonConfig(
    #         gdf=DataReader().get_this_file_ready(r"/data/input/KXF/400849/CROSSWALK.shp"),
    #         name='CROSSWALK',
    #         facecolor='red',
    #         alpha=0.4,
    #         edgecolor='darkred',
    #         linewidth=2,
    #         zorder=2  # 在ROAD_LINE之下
    #     )
    # ]
    #
    # # 定义【参考图层】列表，也可以直接传入GeoDataFrame
    # ref_layers_list = [
    #     ref_gdf
    # ]
    #
    # p = GeoEvaPlotter(interactive_show=True, line_generalization_threshold=1)
    # p.plot(title="yahaha", main_layers=main_layers_list, ref_layers=ref_layers_list, text="yaaaaaaa",
    #        output_svg_path="./yahaha.svg")
