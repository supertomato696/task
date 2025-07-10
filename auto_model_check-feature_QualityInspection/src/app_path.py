# -*- coding: utf-8 -*-
# @Time    : 2025/3/11 15:17
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : app_path.py
import os
import sys
from pathlib import Path


def get_app_path():
    """
    获取应用程序的根目录 (exe 所在的目录, 或开发环境下的 src 目录)。
    """
    if getattr(sys, 'frozen', False):  # 是否是 PyInstaller --onefile 模式
        application_path = os.path.dirname(sys.executable)
    else:
        # 开发环境: 假设 main.py 在 src/my_package 下, 项目根目录是上一级
        application_path = Path(__file__).resolve().parent.parent
        # __file__ 在这里是 app_paths.py 的路径
    return str(application_path)  # 转为字符串，方便使用


# 项目根目录 (在模块加载时就计算好)
APP_ROOT = get_app_path()

# 改变当前工作目录 (如果需要, 且确定所有相对路径都相对于 APP_ROOT)
os.chdir(APP_ROOT)


def resource_path(relative_path):
    """
    获取资源的绝对路径 (相对于项目根目录)。
    """
    return os.path.join(APP_ROOT, relative_path)