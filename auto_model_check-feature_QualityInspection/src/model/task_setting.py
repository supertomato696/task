# from typing import List, Optional
#
# from pydantic import BaseModel, field_validator
# from shapely import Polygon
#
#
# class TaskSetting(BaseModel):
#     """
#     任务设置模型，包含可视化选项和评测参数。
#
#     :param is_visualize: 是否启用可视化，默认为True。
#     :param is_save_svg: 是否保存SVG文件，默认为True.
#     :param params: 评测参数配置。
#     :param evaluate_areas: 包含评测区域的列表。
#     """
#
#     is_visualize: bool = True
#     is_save_svg: bool = True
#     evaluate_rect_setting: Optional[EvaluateRectSetting] = None
#     params: EvaluatorParams
#     evaluate_areas: List[EvaluateArea]
#
#
# if __name__ == '__main__':
#     d = {'is_visualize': True, 'is_save_svg': False, 'evaluate_rect_setting': {'is_limit_by_rects': False, 'rects': [{'left_upper': {'lon': 114.32086539, 'lat': 22.70433302}, 'right_bottom': {'lon': 114.32174208, 'lat': 22.70301799}}]}, 'params': {'line_buffer_length_meter': 20, 'polygon_match_distance': 10, 'landmarking_match_distance': 1.5}, 'evaluate_areas': [{'name': '【铍】', 'evaluate_laneline': False, 'evaluate_boundary': False, 'evaluate_crosswalks': True, 'evaluate_stoplines': True, 'evaluate_landmarks': True}]}
#     ts = TaskSetting(**d)