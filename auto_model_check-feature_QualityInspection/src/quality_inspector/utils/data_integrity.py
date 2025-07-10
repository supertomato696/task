from pathlib import Path
from typing import List, Optional

import geopandas as gpd
import pandera.pandas as pa
from pandera import errors

from src.app_path import APP_ROOT
from src.logger import logger
from src.quality_inspector.error import BadBevIntegrityError
from src.quality_inspector.models.integrity import (
    Integrity,
    IntegrityElement,
    ColumnInfo,
)
from src.tools.common import yaml_safe_load


class DataIntegrityChecker:
    def __init__(self, data_root: Path, config_path: Optional[Path] = None):
        # 在方法内部检查config_path是否为None
        if config_path is None:
            # 如果是None，使用pathlib的斜杠操作符安全地构建默认路径
            config_path = Path(
                APP_ROOT, "src", "quality_inspector", "integrity_rule.yaml"
            )

        intersection_cfg = yaml_safe_load(str(config_path.resolve())).get(
            "intersection", {}
        )
        self.integrity = Integrity(**intersection_cfg)
        self.data_root = data_root

    def check_integrity(self):
        results = {}
        file_suffixs = [".shp", ".dbf", ".prj", ".shx"]

        for name, elem in self.integrity.get_element_fields():
            name: str
            elem: IntegrityElement

            # 检查文件完整性
            logger.info(f"开始检查 {name} 的数据完整性")
            prefix = elem.prefix
            # 获取所有预期文件
            expected_files = [f"{prefix}{ext}" for ext in file_suffixs]
            missing_files = [
                f
                for f in expected_files
                if not (
                    Path(self.data_root, f).exists()
                    and Path(self.data_root, f).is_file()
                )
            ]

            cur_result = []
            if missing_files:
                logger.warning(f"{name} 缺失文件: {missing_files}")
                cur_result.append({"type": "缺失文件", "missing_files": missing_files})
                results[name] = cur_result
                # 文件缺失不需要继续进行了
                continue

            # 读取shp文件
            shp_path = Path(self.data_root, f"{prefix}.shp")
            try:
                gdf = gpd.read_file(shp_path)
            except Exception as e:
                cur_result.append(
                    {"type": f"读取 {prefix}.shp 文件错误", "error": str(e)}
                )
                results[name] = cur_result
                continue

            # 检查列字段
            cols_info = elem.columns
            missing_cols = []
            for col_info in cols_info:
                col_name = col_info.name
                # 本质上从配置文件中移除就好
                if col_name == "GEOM":
                    continue

                if col_name not in gdf.columns:
                    missing_cols.append(col_name)

            if missing_cols:
                logger.warning(f"{name} 缺失列: {missing_cols}")
                cur_result.append({"type": "缺失列", "missing_columns": missing_cols})
                results[name] = cur_result
                continue

            # 检查数据类型
            schema = self.__build_pandera_schema(cols_info)
            try:
                schema.validate(gdf, lazy=True)
            except errors.SchemaErrors as err:
                for failure_case in err.schema_errors:
                    logger.error(
                        f"name: {name}, type: {failure_case.reason_code.name}, msg: {failure_case.args}"
                    )
                    cur_result.append(
                        {
                            "type": failure_case.reason_code.name,
                            "column": failure_case.column_name,
                            "message": ";".join(list(failure_case.args)),
                            "row": failure_case.check_index,
                        }
                    )
            except Exception as err:
                cur_result.append({"type": "字段类型其他错误", "error": str(err)})

            # TODO: 检测geometry类型
            if cur_result:
                results[name] = cur_result

        if results:
            raise BadBevIntegrityError("数据完整性校验失败", results)

        logger.info(f"数据完整性校验通过")

    def __build_pandera_schema(self, columns_info: List[ColumnInfo]):
        cols = {}
        dtype_mapping = {
            "string": pa.String,
            "int": pa.Int32,
            "float": pa.Float64,
            "double": pa.Float64,
        }
        for col in columns_info:
            # 几何类型不在这里检查
            if col.type == "geometry":
                continue

            dtype = dtype_mapping.get(col.type, pa.String)
            nullable = col.nullable
            checks = []
            if col.range:
                checks.append(pa.Check.isin(col.range))
            cols[col.name] = pa.Column(dtype, nullable=nullable, checks=checks)
        return pa.DataFrameSchema(cols)
