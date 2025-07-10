import socket
import logging
from shapely.geometry import Polygon, MultiPolygon, shape
from shapely.ops import unary_union
from typing import Union
from sqlalchemy import create_engine, text
from src.logger.loggerController import logger

from .base import AlgorithmBoxProvider
from ..errors import AlgoboxNotfound


class DatabaseAlgorithmBoxProvider(AlgorithmBoxProvider):
    def __init__(self, db_uri: str, algorithm_box_table: str=None):
        """
        :param db_uri: 数据库连接URI例如 "postgresql://user:password@1.2.3.4:5432/mydb"
        :param algorithm_box_table: 存放算法课数据的表名
        """
        self.db_uri = db_uri
        self.roi_table = "algorithm_box" if algorithm_box_table is None else algorithm_box_table

    def get_algo_box_by_id(self, id_: Union[str, int]) -> Union[Polygon, MultiPolygon]:
        """
        通过id查询数据库中的算法框数据，并合并返回
        """
        try:
            engine = create_engine(self.db_uri)
            with engine.connect() as conn:
                #FIXME 这里是暂时写死了查询语句。
                sql = text(f"SELECT geom FROM {self.roi_table} WHERE id=:id_")
                result = conn.execute(sql, {"id": id_})
                geometries = []
                for row in result:
                    geojson_geom = row[0]
                    try:
                        geom = shape(geojson_geom)
                        geometries.append(geom)
                    except Exception as inner_exc:
                        logger.error(f"解析id={id_}的算法框几何失败: {inner_exc}", exc_info=True)
                if not geometries:
                    raise AlgoboxNotfound(f"未在数据库中查询到id={id_}的算法框数据")
                
                # 这里合并成MultiPolygon试下先
                algo_geom = unary_union(geometries)
                if isinstance(algo_geom, (Polygon, MultiPolygon)):
                    logger.debug(f"成功加载id={id_}的算法框from数据库")
                    return algo_geom
                else:
                    raise ValueError(f"数据库查询返回的id={id_}的算法框几何不符合类型要求")
        except Exception as e:
            logger.exception(f"[DatabaseProvider] 获取id={id_}的算法框失败: {e}")
            raise
