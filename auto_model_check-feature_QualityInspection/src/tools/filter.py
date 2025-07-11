import numpy as np
import pandas as pd
import geopandas as gpd
from typing import Dict, List, Union, Any

from geopandas import GeoDataFrame
from src.logger import logger
from pydantic import BaseModel, ValidationError, RootModel

from src.tools.common import transform_to_utm, ensure_single_geometries


class FilterCondition(BaseModel):
    """
    单个验证条件，用于定义特定字段的过滤规则。

    :param value: 要比较的值，可以是单个值或一个值的列表。
    :param operator: 过滤操作符，如 'eq'（等于）, 'neq'（不等于）等。
    """
    value: Union[str, int, float, List[Union[str, int, float]]]
    operator: str


class FilterConditions(RootModel):
    """
    复合验证条件，用于包含多个字段的过滤规则。

    :param root: 包含多个字段名及其对应的FilterCondition对象的字典。
    """

    root: Dict[str, FilterCondition]

    @staticmethod
    def translate_to_natural_language(
            filter_conditions: Dict[str, Dict[str, Union[str, int, float, List[Union[str, int, float]]]]]) -> str:
        """
        将过滤条件翻译为自然语言描述，便于用户理解。

        :param filter_conditions: 过滤条件字典。
        :return: 自然语言描述的字符串。
        """
        conditions = []
        for key, condition in filter_conditions.items():
            operator = condition["operator"]
            value = condition["value"]

            # 根据不同的操作符生成相应的自然语言描述
            if operator == "eq":
                conditions.append(f"{key} 等于 {value}")
            elif operator == "neq":
                conditions.append(f"{key} 不等于 {value}")
            elif operator == "gt":
                conditions.append(f"{key} 大于 {value}")
            elif operator == "lt":
                conditions.append(f"{key} 小于 {value}")
            elif operator == "ge":
                conditions.append(f"{key} 大于或等于 {value}")
            elif operator == "le":
                conditions.append(f"{key} 小于或等于 {value}")
            elif operator == "in":
                conditions.append(f"{key} 在 {value} 中")
            elif operator == "not in":
                conditions.append(f"{key} 不在 {value} 中")
            else:
                raise ValueError(f"不支持的操作符: {operator}")

        return " 并且 ".join(conditions)  # 返回组合后的自然语言描述


class GeoDataFrameFilter:
    """
    用于过滤 GeoDataFrame 的类，通过指定的过滤条件对 GeoDataFrame 进行筛选。

    :param gdf: 输入的 GeoDataFrame 对象。
    """

    def __init__(self, gdf: gpd.GeoDataFrame):
        """
        初始化 GeoDataFrameFilter 类。

        :param gdf: 输入的 GeoDataFrame。
        """
        self.gdf = gdf  # 存储传入的GeoDataFrame

    def get_column_with_warning(self, key: str, default_value: Any) -> pd.Series:
        """
        获取指定列，如果列不存在则使用默认值并打印警告信息。

        :param key: 列名。
        :param default_value: 默认值。
        :return: 返回指定列的 Series，如果列不存在，则返回填充默认值的 Series。
        """
        if key in self.gdf.columns:
            return self.gdf[key]  # 返回指定列
        else:
            logger.warning(f"列'{key}'在 GeoDataFrame 中未找到。使用默认值。")  # 打印警告日志
            return pd.Series([default_value] * len(self.gdf), index=self.gdf.index)  # 返回填充默认值的Series

    @staticmethod
    def __to_numeric(series: pd.Series) -> pd.Series:
        """
        尝试将系列中的元素转为数字，不能转换的设为 NaN。

        :param series: 输入的 Series。
        :return: 转换后的 Series，无法转换的值会变为 NaN。
        """
        return pd.to_numeric(series, errors='coerce')  # 强制转换为数值类型，出错设置为NaN

    def apply_operator(self, series: pd.Series, operator: str,
                       value: Union[str, int, float, List[Union[str, int, float]]]) -> pd.Series:
        """
        应用指定的操作符进行比较。

        :param series: 输入的 Series。
        :param operator: 操作符（如 "eq", "neq", "gt", "lt", "ge", "le", "in", "not in"）。
        :param value: 比较的值。
        :return: 布尔值的 Series, 指示每个元素是否满足条件。
        """
        # 如果操作符要求数字比较，先尝试转换
        if operator in ["gt", "lt", "ge", "le"]:
            series = self.__to_numeric(series)  # 转换为数值类型
            value = float(value)  # 将比较值转换为浮点数

        # 根据操作符进行比较并返回结果
        if operator == "eq":
            return series == value  # 等于
        elif operator == "neq":
            return series != value  # 不等于
        elif operator == "gt":
            return series > value  # 大于
        elif operator == "lt":
            return series < value  # 小于
        elif operator == "ge":
            return series >= value  # 大于或等于
        elif operator == "le":
            return series <= value  # 小于或等于
        elif operator == "in":
            if not isinstance(value, list):
                raise ValueError("'in'条件需要输入列表")  # 检查in操作符的值是否为列表
            return series.isin(value)  # 判断是否在给定列表中
        elif operator == "not in":
            if not isinstance(value, list):
                raise ValueError("'not in'条件需要输入列表")  # 检查not in操作符的值是否为列表
            return ~series.isin(value)  # 取反，判断是否不在给定列表中
        else:
            raise ValueError(f"不支持的操作符: {operator}")  # 提示不支持的操作符

    def generate_mask(self, filter_conditions: FilterConditions) -> pd.Series:
        """
        根据过滤条件生成掩码。

        :param filter_conditions: 过滤条件对象。
        :return: 返回布尔值的 Series 作为掩码，指示哪些行满足所有条件。
        """
        masks = []  # 存储各列条件的布尔值
        for key, condition in filter_conditions.root.items():
            series = self.get_column_with_warning(key, False)  # 获取当前列的数据
            masks.append(self.apply_operator(series, condition.operator, condition.value))  # 计算该条件的掩码

        # 返回最终掩码，满足所有条件的行
        return np.logical_and.reduce(masks) if masks else pd.Series([True] * len(self.gdf), index=self.gdf.index)

    def filter_gdf(self, filter_conditions: Dict[
        str, Dict[str, Union[str, int, float, List[Union[str, int, float]]]]]) -> gpd.GeoDataFrame:
        """
        根据过滤条件过滤 GeoDataFrame。

        :param filter_conditions: 过滤条件字典。
        :return: 返回过滤后的 GeoDataFrame。
        """
        validated_conditions = self.validate_filter_conditions(filter_conditions)  # 验证过滤条件
        mask = self.generate_mask(validated_conditions)  # 生成掩码
        return self.gdf[mask]  # 根据掩码返回过滤后的DataFrame

    @staticmethod
    def validate_filter_conditions(filter_conditions: Dict[
        str, Dict[str, Union[str, int, float, List[Union[str, int, float]]]]]) -> FilterConditions:
        """
        验证过滤条件是否合法。

        :param filter_conditions: 过滤条件字典。
        :return: 验证后的 FilterConditions 对象。
        """
        try:
            validated_conditions = FilterConditions.model_validate(filter_conditions)  # 使用Pydantic验证条件格式
            #logger.info(
            #    f"过滤条件验证成功:{FilterConditions.translate_to_natural_language(filter_conditions)}")  # 记录验证成功信息
            return validated_conditions
        except ValidationError as e:
            logger.error("过滤条件验证失败:", e)  # 记录验证失败信息
            raise e  # 抛出异常


def get_filtered_geometries(gdf: GeoDataFrame, filter_conditions: Dict=None, convert_to_utm=False):
    """
    获取过滤后的几何要素，支持坐标系转换。

    :param gdf: 输入的 GeoDataFrame。
    :param convert_to_utm: 是否将坐标系转换至UTM（默认为False）。
    :param filter_conditions: 过滤条件字典（可选）。
    :return: 过滤后的 GeoDataFrame。
    """
    logger.debug(f"传入的筛选条件为{filter_conditions}")
    gdf_filter = GeoDataFrameFilter(gdf)  # 创建 GeoDataFrameFilter 实例

    if filter_conditions:
        # 过滤 GeoDataFrame
        gdf = gdf_filter.filter_gdf(filter_conditions)

    if convert_to_utm:
        # 转换坐标系至UTM
        gdf = transform_to_utm(gdf)

    # return ensure_single_geometries(gdf)  # 确保只有单一几何体
    return gdf
