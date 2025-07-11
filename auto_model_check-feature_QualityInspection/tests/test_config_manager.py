# -*- coding: utf-8 -*-
# @Time    : 2025/3/11 10:53
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : test_config_manager
from src.tools.config_manager import ConfigManager


def test_load_config():
    c = ConfigManager()
    cfg = c.load_config("../v2config.yaml")

    print(cfg.evaluation_actions)
