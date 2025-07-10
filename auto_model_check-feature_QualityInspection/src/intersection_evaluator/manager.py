# -*- coding: utf-8 -*-
# @Time    : 2025/3/3 13:12
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : manager
import os
import json
from src.algorithm_range.manager import AlgorithmBoxManager
from src.intersection_evaluator.evaluator import IntersectionEvaluator
from src.intersection_evaluator.init_intersection import TruthIntersectionDataGetter, InputIntersectionDataGetter, Intersection, Intersection
from src.logger import logger
from src.model import Config
from src.tools.config_manager import ConfigManager
from src.tools.json2csv import json2csv
from datetime import datetime

class EvaluationManager:
    def __init__(self, config_path=None):
        """
        初始化评测管理类，加载配置与任务设置。

        :param config_path: 配置文件路径。
        :param config_path: 任务设置文件路径。
        """
        if config_path is None:
            logger.error("请输入配置文件的路径")

        # # config_name也就是配置文件名作为任务名称
        # self.config_name = extract_filename(config_path)

        self.config_manager = ConfigManager()
        self.cfg: Config = self.config_manager.load_config(config_path)
        logger.info("配置加载完毕")

        self.tool_config = self.cfg.config
        self.tasks = self.cfg.tasks

        # 检查配置有效性
        if not self.cfg.versions:
            raise Exception("没有传入有效的版本资料")
        if not self.cfg.config.file_paths:
            raise Exception(f"请检查数据路径：{self.cfg.config.file_paths}")

        # 真值主路径
        self.truth_file_dir = self.build_file_path(self.cfg.config.file_paths.data_path,
                                                   self.cfg.config.file_paths.truth_data_path)
        # 输入值值主路径
        self.input_data_path = self.build_file_path(self.cfg.config.file_paths.data_path,
                                                    self.cfg.config.file_paths.input_data_path)

        # 输出主路径
        self.output_data_path = self.build_file_path(self.cfg.config.file_paths.data_path,
                                                     self.cfg.config.file_paths.output_data_path)

        # 算法框主路径
        self.algo_box_data_path = self.build_file_path(self.cfg.config.file_paths.data_path,
                                                       self.cfg.config.file_paths.algorithm_box_data_path)
        # 实例化组件们
        self.algo_box_manager = AlgorithmBoxManager(self.algo_box_data_path)
        self.tig = TruthIntersectionDataGetter(self.tool_config)

    def init_intersection(self):
        ...

    @classmethod
    def build_file_path(cls, base_dir: str, *paths: str) -> str:
        """
        构建文件路径

        :param base_dir: 基础目录
        :param paths: 相对路径部分
        :return: 完整的文件路径
        """
        return os.path.join(base_dir, *paths)

    def start_evaluation(self):
        """
        执行评测任务。
        """

        logger.info(f"即将执行的评测项目为{self.cfg.evaluation_actions}")

        # 对于每一个版本
        for input_version_data in self.cfg.versions:
            # 版本号
            version_name = input_version_data.version
            # 小标签
            nickname = input_version_data.nickname

            # 版本数据保存的路径
            version_data_path = os.path.join(self.input_data_path, version_name)

            # 带评测路口数据获取器
            iig = InputIntersectionDataGetter(tool_config=self.tool_config, version_config=input_version_data)

            if not self.cfg.tasks:
                # 如果不传那么应该所有的算法框都要评。后面再实现
                raise NotImplemented("如果不传那么应该所有的算法框都要评。后面实现")

            task_result = {
                "version": version_name,
                "nickname": nickname,
                "elapsed_time": 0,
                "result": {}
            }

            elapsed_time = 0

            for task in self.cfg.tasks:
                # FIXME debug完毕记得加个tryCatch
                algo_box_id = task.name
                # 先看看算法框在不在
                algo_box_geom = self.algo_box_manager.get_algo_box_by_id(algo_box_id)

                # 拿真值路口
                truth_intersection_raw = self.tig.get_local_truth_data_by_algorithm_range(algo_box_id=algo_box_id)
                # 根据算法框剪裁真值路口
                # truth_intersection: Intersection = truth_intersection_raw.prune_by(exact_area=algo_box_geom)
                truth_intersection: Intersection = truth_intersection_raw.prune_by(exact_area=algo_box_geom)
                # 附上算法框几何
                truth_intersection.set_algo_box_geom(algo_box_geom)

                # 拿待评测路口
                input_intersection_raw = iig.get_local_input_data_in_algorithm_range(algo_box_id=algo_box_id,
                                                                                     nickname=nickname)
                # 根据算法框剪裁待评测路口
                input_intersection = input_intersection_raw.prune_by(exact_area=algo_box_geom)
                # 附上算法框几何
                input_intersection.set_algo_box_geom(algo_box_geom)

                # 0411临时增加：当算法框内的真值不充足，使用经过算法框剪裁后，框内真值的凸包作为新的评测区域
                logger.debug("防止当算法框内的真值不充足，使用经过算法框剪裁后，框内真值的凸包作为新的评测区域...正在剪裁真值")
                algo_box_geom_ = truth_intersection.convex_hull
                truth_intersection: Intersection = truth_intersection_raw.prune_by(exact_area=algo_box_geom_,
                                                                                   polygon_crs=truth_intersection.lane_lines.crs)
                truth_intersection.set_algo_box_geom(algo_box_geom)

                logger.debug("防止当算法框内的真值不充足，使用经过算法框剪裁后，框内真值的凸包作为新的评测区域...正在剪裁待评测值")
                input_intersection = input_intersection_raw.prune_by(exact_area=algo_box_geom_,
                                                                     polygon_crs=truth_intersection.lane_lines.crs)
                input_intersection.set_algo_box_geom(algo_box_geom)

                # 实例化路口评测工具本体。所有评测都由这个evaluator发起。
                evaluator = IntersectionEvaluator(truth_intersection=truth_intersection,
                                                  input_intersection=input_intersection,
                                                  result_output_root_dir=self.output_data_path,
                                                  algo_box_id=algo_box_id,
                                                  tool_param=self.tool_config.params)

                elapsed_time += evaluator.linear_run(self.cfg.evaluation_actions, task_result)
            task_result['elapsed_time'] = f"{elapsed_time:.2f}s"

            task_json_file = os.path.join(self.output_data_path,
                                          task_result['version'] + '_' + datetime.now().strftime("%Y-%m-%d_%H%M%S") + '.json')

            # TODO 根据评测结果是否有效决定是否创建文件目录？
            with open(task_json_file, 'w', encoding='utf-8') as f:
                json.dump(task_result, f, ensure_ascii=False, indent=4)
            json2csv(self.output_data_path, task_json_file)
            # # 禁忌之力，非必要不要打开
            # evaluator.parallel_run(self.cfg.evaluation_actions)
            logger.info("任务跑完，可以收工")
