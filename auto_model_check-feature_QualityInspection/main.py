# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 14:20
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : main.py
import argparse
from pathlib import Path
import sys


from src.intersection_evaluator.manager import EvaluationManager
from src.logger import logger
from src.quality_inspector.geo_eva_union_run import feat_BevCoverCal
from src.quality_inspector.main_inspector import run_inspect_task

VERSION = "v2.0.2"
QAVERSION = "v1.3.0"


def main():
    em = EvaluationManager("data/input/config.yaml")
    em.start_evaluation()


def start_cli():
    """命令行接口(CLI)启动函数"""
    parser = argparse.ArgumentParser(
        description="GeoEva满血版：精度评测+几何质检。",
        formatter_class=argparse.RawTextHelpFormatter,  # 使用 RawTextHelpFormatter 可以保留 help 字符串中的换行
    )

    # 创建一个互斥组required=True确保了这个组中至少有一个参数必须被提供
    group = parser.add_mutually_exclusive_group(required=True)

    group.add_argument(
        "-t",
        "--task",
        dest="task",  # 明确指定存储的变量名
        help="GeoEva x BevCoverCal。\n 复用BevCoverCal的运行路径。",
    )

    group.add_argument(
        "-p",
        "--path",
        dest="path",  # 明确指定存储的变量名
        help="指定质检任务目录的路径。\n该目录下应包含 tasks.json, sd_link.geojson 等文件。",
    )
    group.add_argument(
        "-f",
        "--file",
        dest="file",
        help="指定精度评测YAML配置文件路径。用于启动精度评价流程。",
    )

    args = parser.parse_args()

    # 根据提供的参数执行相应的逻辑
    if args.file:
        logger.info("===== GeoEva精度评测模式 =====")
        try:
            config_file = Path(args.file)
            if not config_file.is_file():
                # 使用 logger 或 print 输出错误
                logger.error(
                    f"[GeoEva精度评测] 配置文件{config_file}不存在或不是一个合法文件，请检查~"
                )
                sys.exit(1)

            em = EvaluationManager(config_file)
            em.start_evaluation()

        except Exception as e:
            # 捕获 EvaluationManager 初始化或执行中的所有可能错误
            logger.exception(f"精度评测过程中发生错误: {e}")
            sys.exit(1)

    elif args.path:
        logger.info("===== GeoEva质检模式 =====")
        try:
            input_dir = Path(args.path)
            if not input_dir.is_dir():
                logger.error(
                    f"[GeoEva质检] 提供的路径'{input_dir}'不是一个有效的目录。"
                )
                sys.exit(1)

            _ = run_inspect_task(input_dir)

        except Exception as e:
            # 捕获所有其他未知错误
            logger.exception(f"处理目录时发生未知错误: {e}")
            sys.exit(1)

    elif args.task:
        logger.info('[GeoEva质检 x BevCoverCal 版本]: V24.3.2')
        inspect_task_dir = Path(args.task)
        feat_BevCoverCal(inspect_task_dir)


# def debug():
#     inspect_task_dir = Path("/mnt/tasks/model/label_cover/")
#     feat_BevCoverCal(inspect_task_dir)

if __name__ == "__main__":
    # debug()
    start_cli()
