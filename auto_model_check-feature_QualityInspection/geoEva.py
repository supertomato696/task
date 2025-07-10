# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 14:20
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : demo.py
import argparse
import os
import sys

from src import __version__
from src.intersection_evaluator.manager import EvaluationManager
from src.scripts.cut_by_algo_box import perform_clipping

"""
71951   氦
71952   锂
72377   铍
72001   硼
71699   氖
"""


def get_app_path():
    """
  获取应用程序的根目录。
  """
    if getattr(sys, 'frozen', False):  # 是否是 PyInstaller --onefile 模式
        # --onefile 模式下，application_path 是 exe 文件所在的目录
        application_path = os.path.dirname(sys.executable)
    else:
        # 开发环境下, application_path 是当前脚本所在的目录
        application_path = os.path.dirname(os.path.abspath(__file__))
    return application_path


# 1. 获取应用程序根目录
app_root = get_app_path()

# 2. 改变当前工作目录到应用程序根目录
os.chdir(app_root)


def resource_path(relative_path):
    """
    获取资源的绝对路径 (相对于应用程序根目录)。
    """
    return os.path.join(app_root, relative_path)

def main():
    em = EvaluationManager("config.yaml")
    em.start_evaluation()


def start_cli():
    parser = argparse.ArgumentParser(
        description="Process a YAML file for evaluation, with optional clipping functionality."
    )

    # 原有的 YAML 配置文件参数
    parser.add_argument("-f", "--file", help="Path to the YAML configuration file.")

    # 新增的裁剪相关参数
    parser.add_argument("--clip", action="store_true", help="Enable clipping functionality.")
    parser.add_argument("--algo_id", help="用来剪裁的算法框ID")
    parser.add_argument("--input_dir", help="剪裁的输入")
    parser.add_argument("--output_dir", help="根据算法框剪裁之后输出到这里")

    args = parser.parse_args()

    # 如果没有使用--clip那么认为是正经评测，若缺少-f参数指定配置文件那么报错
    if not args.clip and args.file is None:
        parser.error("the following arguments are required when not using --clip: -f/--file")

    # 根据是否启用裁剪，执行不同的逻辑
    if args.clip:
        if not args.algo_id or not args.input_dir:
            parser.error("--clip requires --algo_id and --input_dir.")

        # 如果未指定输出目录，则使用当前工作目录作为基础
        output_base = args.output_dir if args.output_dir else os.getcwd()
        output_dir = os.path.join(output_base, args.algo_id)  # 输出到 {output_dir}/{algo_id}/

        # 直接调用独立的裁剪函数
        perform_clipping(args.algo_id, args.input_dir, output_dir)
    else:
        # 原有的评测逻辑
        em = EvaluationManager(args.file)
        em.start_evaluation()


if __name__ == '__main__':
    print(f"GeoEva version= {__version__}")
    main()
