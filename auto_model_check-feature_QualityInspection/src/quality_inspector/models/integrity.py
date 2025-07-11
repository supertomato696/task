from typing import List, Optional, Generator, Tuple
from pydantic import BaseModel, field_validator, ConfigDict, Field

class ColumnInfo(BaseModel):
    name: str
    type: str
    nullable: Optional[bool] = True
    range: Optional[List] = []

class IntegrityElement(BaseModel):
    prefix: str
    type: str
    columns: List[ColumnInfo]

class Integrity(BaseModel):
    lane_lines: Optional[IntegrityElement] = None
    road_boundaries: Optional[IntegrityElement] = None
    lane_center: Optional[IntegrityElement] = None
    crosswalks: Optional[IntegrityElement] = None
    stop_lines: Optional[IntegrityElement] = None
    arrows: Optional[IntegrityElement] = None
    traffic_lights: Optional[IntegrityElement] = None
    lane_boundary_vectorize: Optional[IntegrityElement] = None
    road_boundary_vectorize: Optional[IntegrityElement] = None
    lanes: Optional[IntegrityElement] = None
    roads: Optional[IntegrityElement] = None

    model_config = ConfigDict(arbitrary_types_allowed=True)

    def get_element_fields(self) -> Generator[Tuple[str, IntegrityElement], None, None]:
        """
        一个生成器函数，用于优雅地获取模型中所有非空的EvaluateElementProfile字段。

        Yields:
            一个元组 (字段名, 字段值)，其中字段值是 EvaluateElementProfile 的实例。
        """
        # 1. 遍历模型的字段定义 (model_fields 是一个字典 {field_name: FieldInfo})
        for field_name, field_info in self.model_fields.items():
            # 2. 检查字段的类型注解是否为我们想要的类型.为了简化，直接获取实例中的值并判断其类型
            field_value = getattr(self, field_name, None)

            # 3. 如果字段的值是 EvaluateElementProfile 的实例，则处理它
            if isinstance(field_value, IntegrityElement):
                yield field_name, field_value