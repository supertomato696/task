# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 13:31
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : errors.py

# 基础异常类
class GeoEvaError(Exception):
    """应用程序基础异常"""
    pass


# 子类异常
class AlgoboxNotfound(GeoEvaError):
    """没有找到算法框"""
    pass


class FileNotFoundError(GeoEvaError):
    """文件未找到异常"""
    pass


class DataEmpty(GeoEvaError):
    """评测数据缺失"""
    pass


class InputDataIllegal(GeoEvaError):
    """输入文件不对劲"""
    pass


class BrokeFile(GeoEvaError):
    """输入文件不对劲"""
    pass
