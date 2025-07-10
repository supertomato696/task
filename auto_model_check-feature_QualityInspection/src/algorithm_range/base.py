from abc import ABC, abstractmethod
from shapely.geometry import Polygon, MultiPolygon
from typing import Union

class AlgorithmBoxProvider(ABC):
    @abstractmethod
    def get_algo_box_by_id(self, id_: Union[str, int]) -> Union[Polygon, MultiPolygon]:
        """
        返回指定算法框ID对应的的几何对象。
        :param id_: 算法框的ID，可以是字符串或数字
        :return: shapely.geometry.Polygon 或 MultiPolygon 对象
        """
        pass