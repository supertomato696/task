import os
import socket
from pathlib import Path
from typing import Union, Optional
from shapely.geometry import Polygon, MultiPolygon

from src.algorithm_range.local import LocalAlgorithmBoxProvider
from src.algorithm_range.database import DatabaseAlgorithmBoxProvider
from src.algorithm_range.base import AlgorithmBoxProvider
from src.logger.loggerController import logger
from src.model.config import ToolConfig


class AlgorithmBoxManager:
    def __init__(self,
                 local_algo_dir: str,
                 # local_range_dir: Union[str, Path],
                 db_uri: Optional[str] = None,
                 algorithm_box_table: Optional[str] = None,
                 db_ip: Optional[str] = None,
                 db_port: int = 5432) -> None:
        """
        :param local_algo_dir: 本地文件的算法框几何数据目录
        :param db_uri: 数据库连接 URI
        :param algorithm_box_table: 数据库中存储算法框几何的表名
        :param db_ip: 数据库的IP
        :param db_port: pgsql端口号，默认为 5432
        """
        self.local_algo_box_dir = local_algo_dir
        # FIXME 数据库信息从配置文件里读取，后面补上
        self.db_uri = db_uri
        self.algorithm_box_table = algorithm_box_table
        self.db_ip = db_ip
        self.db_port = db_port

        self.provider = self._choose_provider()

    def _can_ping_db(self) -> bool:
        if not self.db_ip:
            return False
        try:
            with socket.create_connection((self.db_ip, self.db_port), timeout=3):
                logger.debug(f"成功连接到数据库IP: {self.db_ip}:{self.db_port}")
                return True
        except Exception as e:
            logger.warning(f"无法连接到数据库IP: {self.db_ip}:{self.db_port}, 原因: {e}")
            return False

    def _choose_provider(self) -> AlgorithmBoxProvider:
        """
        根据网络连通情况与配置选择使用数据库Provider或本地Provider
        """
        if self.db_uri and self.algorithm_box_table and self._can_ping_db():
            try:
                logger.info("尝试使用数据库Provider获取 算法框几何")
                provider = DatabaseAlgorithmBoxProvider(db_uri=self.db_uri, algorithm_box_table=self.algorithm_box_table)
                # 可根据需要测试某个默认 id 来确保 Provider 可用
                _ = provider.get_algo_box_by_id(id_='test')  # 可能需要调整此处的测试逻辑
                return provider
            except Exception as e:
                logger.error(f"数据库Provider初始化或获取算法框几何失败: {e}，将回退使用本地algoProvider", exc_info=True)
        logger.info("使用本地Provider获取算法框几何")
        return LocalAlgorithmBoxProvider(range_dir=self.local_algo_box_dir)

    def get_algo_box_by_id(self, id_: Union[str, int]) -> Union[Polygon, MultiPolygon]:
        logger.debug(f"通过Provider获取算法框几何，id={id_}")
        return self.provider.get_algo_box_by_id(id_)

# if __name__ == '__main__':
#     # 配置项：
#     db_ip = "1.2.3.4"
#     db_port = 5432
#     db_uri = "postgresql://user:password@1.2.3.4:5432/mydb"
#     algo_box_table = "algo_box_data"
#     local_range_dir = "/data/batch-195"
#
#     # 初始化 Manager
#     algo_manager = AlgorithmBoxManager(
#         local_range_dir=Path(local_range_dir),
#     )
#
#     # 获取指定 ID 的 ROI
#     algorithm_id = 123  # 示例的算法框 ID
#     try:
#         algo_box = algo_manager.get_algo_box_by_id(algorithm_id)
#         logger.info(f"成功获取算法框，范围：{algo_box.bounds}")
#     except Exception as e:
#         logger.error(f"获取算法框失败: {e}", exc_info=True)