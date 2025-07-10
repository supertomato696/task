# -*- coding: utf-8 -*-
# @Time    : 2025/6/5 21:42
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : error
from src.quality_inspector.error_code import ErrorCode


class InvalidGeometryTypeError(Exception):
    pass


class BadPolygonError(Exception):
    pass


class BadAlgoBoxGeom(Exception):
    pass


class BadBevIntegrityError(Exception):

    def __init__(self, message, error_info):
        self.code = ErrorCode.BAD_INTEGRITY
        self.message = message
        self.error_info = error_info
