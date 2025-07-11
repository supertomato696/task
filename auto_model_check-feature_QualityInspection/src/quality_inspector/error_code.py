# -*- coding: utf-8 -*-
# @Time    : 2025/6/12 15:11
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : error_coode.py
from enum import IntEnum, Enum


class STATUS(Enum):
    PASS: 0
    FAIL: 1


class Status(IntEnum):
    PASS: 0
    FAIL: 1


class ErrorCode(IntEnum):
    SUCCESS = 0
    INIT = 400
    INTERNAL_ERROR = 500
    INVALID_PARAMETER = 600
    TIMEOUT = 700

    BAD_INTEGRITY = 10001
    PARTIAL_FAIL = 20001
