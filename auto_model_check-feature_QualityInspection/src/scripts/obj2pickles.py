# -*- coding: utf-8 -*-
# @Time    : 2025/3/6 18:05
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : obj2pickles.py
import pickle
from typing import Any


def save_here(d: Any, filename):
    """
    将 obj保存为 pickle 文件。
    """
    with open(filename, 'wb') as f:
        pickle.dump(d, f)

def load_this(filename) -> Any:
    with open(filename, 'rb') as f:
        d = pickle.load(f)
    return d
