import os
from pathlib import Path
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
import geopandas as gpd
from typing import Union, List
from src.data_process.file_read import remove_z
from src.errors import AlgoboxNotfound
from src.logger.loggerController import logger
from src.algorithm_range.base import AlgorithmBoxProvider


class LocalAlgorithmBoxProvider(AlgorithmBoxProvider):
    def __init__(self, range_dir: Union[str, Path]):
        """
        :param range_dir: 存放算法框数据文件的目录
        """
        self.range_dir = Path(range_dir)
        # 尝试从目录中加载一个存储所有算法框数据的 shp 文件（仅在一级目录中查找，不递归查找）
        self.offline_algo_box_gdf = self._try_get_offline_algo_box_gdf()

    @staticmethod
    def _filter_valid_files(files: List[Path]) -> List[Path]:
        """
        过滤掉非几何数据文件，例如 shapefile 的辅助文件（如 .dbf）
        只保留 .shp 和 .geojson 文件
        """
        valid_extensions = {'.shp', '.geojson'}
        return [f for f in files if f.suffix.lower() in valid_extensions]

    def _try_get_offline_algo_box_gdf(self) -> Union[gpd.GeoDataFrame, None]:
        """
        尝试在本地指定目录的第一级（不递归搜索）获取 shp 文件，
        并且认为这个 shp 文件中存储了所有本地algo_box数据。
        如果存在多个符合条件的 shp 文件，则只使用第一个文件。
        Returns:
             一个包含算法框数据的 GeoDataFrame 或 None（如果未找到）
        """
        candidate_files = list(self.range_dir.glob("*.shp"))
        if candidate_files:
            if len(candidate_files) > 1:
                logger.warning(
                    f"在{self.range_dir}中找到多个 shp 文件，默认只使用第一个: {candidate_files[0].name}"
                )
            offline_file = candidate_files[0]
            logger.debug(f"加载离线算法框文件: {offline_file}")
            try:
                gdf = gpd.read_file(offline_file)
                if gdf.geometry.has_z.any():
                    gdf['geometry'] = gdf['geometry'].apply(remove_z)
                return gdf
            except Exception as e:
                logger.exception(f"读取离线算法框文件失败: {offline_file.name}, 错误：{e}")
                return None
        else:
            logger.info(f"在目录{self.range_dir}中未找到任何 shp 文件作为离线算法框数据")
            return None

    def get_algo_box_by_id(self, id_: Union[str, int]) -> Union[Polygon, MultiPolygon]:
        """
        根据传入的 id_ 查找本地对应的算法框文件或离线数据中的算法框几何。
        文件名格式示例：<id>.shp 或 <id>.geojson

        如果在目录中未找到单独文件，则尝试在离线加载的算法框数据中查找字段 "algorithm" 等于 id_ 的记录。

        :param id_: 算法框标识（可以是字符串或数字）
        :return: 合并后的 Polygon 或 MultiPolygon 对象
        :raises AlgoboxNotFound: 如果找不到相关的算法框数据
        """
        pattern = f"{id_}.*"
        candidate_files = list(self.range_dir.glob(pattern))
        # 过滤掉例如 .dbf 等非数据文件
        candidate_files = self._filter_valid_files(candidate_files)

        if not candidate_files:
            # 若未在目录中找到单独的算法框文件，则尝试从 offline_algo_box_gdf 中查找
            if self.offline_algo_box_gdf is not None and not self.offline_algo_box_gdf.empty:
                logger.debug(f"在目录中未找到独立的文件，尝试从离线加载的数据中查找算法框 id: {id_}")

                # 可能用带中文的来找..一般这种都是自测的时候随便取得名字..先规避一下
                try:
                    int(id_)
                    subset = self.offline_algo_box_gdf[self.offline_algo_box_gdf["algorithm_"] == int(id_)]
                except:
                    # raise AlgoboxNotfound(f"在离线数据中未找到算法框 id: {id_}")
                    subset = self.offline_algo_box_gdf[self.offline_algo_box_gdf["algorithm_"] == id_]

                # 0304拿到的文件中任务框ID是 "algorithm_"
                subset = self.offline_algo_box_gdf[self.offline_algo_box_gdf["algorithm_"] == int(id_)]
                if not subset.empty:
                    try:
                        roi_geom = unary_union(subset.geometry)
                        if isinstance(roi_geom, (Polygon, MultiPolygon)):
                            logger.debug("成功从离线数据中加载算法框几何信息")
                            return roi_geom
                        else:
                            raise ValueError("加载的算法框类型错误，不是 Polygon 或 MultiPolygon")
                    except Exception as e:
                        logger.exception(f"合并离线算法框几何时出错: {e}")
                        raise
                else:
                    raise AlgoboxNotfound(f"在离线数据中未找到算法框 id: {id_}")
            else:
                raise AlgoboxNotfound(f"在目录 {self.range_dir} 中找不到 id 为 {id_} 的算法框文件")

        # 如果有多个候选文件，记录警告信息，并默认使用第一个文件
        if len(candidate_files) > 1:
            logger.warning(
                f"找到多个算法框文件匹配 {id_}: {[f.name for f in candidate_files]}。将使用第一个文件: {candidate_files[0].name}"
            )

        range_file = candidate_files[0]
        logger.debug(f"加载本地算法框文件: {range_file}")
        try:
            gdf = gpd.read_file(range_file)
            if gdf.geometry.has_z.any():
                gdf['geometry'] = gdf['geometry'].apply(remove_z)
            # 使用全部几何（如果有多条记录，则进行合并）
            roi_geom = unary_union(gdf.geometry)
            if isinstance(roi_geom, (Polygon, MultiPolygon)):
                logger.debug("成功加载本地算法框，返回 Polygon 或 MultiPolygon 几何")
                return roi_geom
            else:
                raise ValueError("加载的算法框几何类型错误，不是 Polygon 或 MultiPolygon")
        except Exception as e:
            logger.exception(f"本地加载算法框失败: {e}")
            raise
