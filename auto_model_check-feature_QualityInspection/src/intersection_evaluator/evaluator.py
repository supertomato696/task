import os
from typing import Union, Optional
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from geopandas import GeoDataFrame

from src.intersection_evaluator.arrow import ArrowEvaluator, ArrowEvaluateSummary
from src.intersection_evaluator.boundary import RoadBoundaryEvaluator, VectorizeOnlyRoadBoundaryEvaluator
from src.intersection_evaluator.crosswalk import CrosswalkEvaluator, PolygonEvaluateSummary
from src.intersection_evaluator.init_intersection import Intersection
from src.intersection_evaluator.lane_line import LaneLineEvaluator, LineMatchResult, VectorizeOnlyLaneLineEvaluator
from src.intersection_evaluator.relative_width import LaneWidthEvaluator
from src.intersection_evaluator.stopline import StopLineEvaluator, StopLineEvaluateResult
from src.intersection_evaluator.trafficlight import TrafficlightEvaluator, TrafficLightEvaluateResult
from src.logger import logger
from src.model.config import EvaluatorParams


# 元类 + 装饰器标记
class EvaluationMeta(type):
    def __new__(cls, name, bases, attrs):
        # 创建新类时收集所有带标记的方法
        task_registry = {}
        for attr_name, attr_value in attrs.items():
            if hasattr(attr_value, '_eval_task'):
                task_name = getattr(attr_value, '_eval_task')
                task_registry[task_name] = attr_name

        # 注入任务注册表
        attrs['_task_registry'] = task_registry
        return super().__new__(cls, name, bases, attrs)


def task(name: str = None):
    """安全的任务装饰器"""

    def decorator(func):
        func._eval_task = name or func.__name__
        logger.debug(f"已注册评测方法：{name}")
        return func

    return decorator


class IntersectionEvaluator(metaclass=EvaluationMeta):

    def __init__(self,
                 truth_intersection: Intersection,
                 input_intersection: Intersection,
                 result_output_root_dir: str,
                 algo_box_id: Union[str, int],
                 tool_param: EvaluatorParams,
                 is_ignore_z: bool = True,
                 ):
        """

        """
        self.evaluation_tasks = {
            task_name: getattr(self, method_name)
            for task_name, method_name in self._task_registry.items()
        }

        # 算法框ID
        self.algo_box_id = algo_box_id
        # 工具的配置
        self.tool_param = tool_param

        # 真值和输入值
        self.truth_intersection = truth_intersection
        self.input_intersection = input_intersection

        self.result_output_root_dir = result_output_root_dir
        # 由于输入路口自带版本、任务框、nickname信息，所以生成路径的函数目前放在input_intersection中
        self.result_output_sub_path = self.input_intersection.get_result_save_sub_path()
        # 保存评测结果的完整子路径
        self.result_save_full_path = os.path.join(self.result_output_root_dir, self.result_output_sub_path)

        # 预创建结果保存路径
        os.makedirs(self.result_save_full_path, exist_ok=True)

        # 防止混入高程
        if self.truth_intersection.is_ignored_z != is_ignore_z or self.input_intersection.is_ignored_z != is_ignore_z:
            raise ValueError(f'设置是否忽略高程为{is_ignore_z}。请检查输入真值高程和待评测路口是否包含高程')

    def __get_valid_tasks(self, evaluation_items: list) -> dict:
        """筛选有效任务"""
        return {
            item: self.evaluation_tasks[item]
            for item in evaluation_items
            if item in self.evaluation_tasks
        }

    def linear_run(self, evaluation_items: list, task_result: dict):
        """串行执行评测项"""
        overall_start = time.perf_counter()

        # 筛选有效任务
        valid_tasks = self.__get_valid_tasks(evaluation_items)

        # 执行验证
        if missing := set(evaluation_items) - set(valid_tasks):
            logger.warning(f"未注册的评估项,将会被跳过: {', '.join(missing)}")
        # 执行任务
        task_result['result'][self.algo_box_id] = {}
        for task_name, task_func in valid_tasks.items():
            try:
                logger.info(f"开始执行任务: {task_name}")
                evaluation_results = task_func()
                if evaluation_results is None:
                    logger.warning(f"任务: {task_name}未能执行，跳过")
                    continue
                if task_name != "relative_width_eval" and task_name != "lane_line_offset_statistic":
                    task_result['result'][self.algo_box_id].update({task_name: evaluation_results.get_dict()})
            except Exception as e:
                logger.exception(f"任务 {task_name} 执行失败：{e}", exc_info=True)
        elapsed_time = time.perf_counter() - overall_start
        logger.info(f"任务执行完毕，总耗时 {elapsed_time:.2f}s")
        return elapsed_time

    def parallel_run(self, evaluation_items: list):
        logger.warning("并行执行时产出的svg可能出现异常。")
        evaluation_tasks_ = self.__get_valid_tasks(evaluation_items)

        def run_task_with_timing(task_, task_name):
            """
            运行任务并记录耗时。如果任务成功，返回 (result, elapsed)；
            如果抛出异常，也记录时间后抛出异常。
            """
            start = time.perf_counter()
            try:
                res_ = task_()
            except Exception as e:
                elapsed = time.perf_counter() - start
                raise Exception(
                    f"Task '{task_name}' failed after {elapsed:.2f} seconds. Exception: {e}") from e
            elapsed = time.perf_counter() - start
            return res_, elapsed

        results = {}  # 存储任务返回的结果（以任务名为 key，这样无论返回值是什么类型都不会影响）
        task_durations = {}  # 存储每个任务的耗时

        overall_start = time.perf_counter()

        with ThreadPoolExecutor(max_workers=len(evaluation_tasks_)) as executor:
            # 将任务包装后提交到执行器中，这里保证传入的 key 是字符串
            future_to_task = {
                executor.submit(run_task_with_timing, func, task_name): task_name
                for task_name, func in evaluation_tasks_.items()
            }

            for future in as_completed(future_to_task):
                # 这里保证task_name是之前定义的字符串
                task_name = future_to_task[future]
                try:
                    result, duration = future.result()
                except Exception as exc:
                    logger.exception(f"任务 {task_name} 执行失败：{exc}")
                else:
                    results[task_name] = result
                    task_durations[task_name] = duration

        overall_elapsed = time.perf_counter() - overall_start

        logger.info("\n所有任务执行完毕。")
        logger.info("各任务耗时：")
        for task_name, duration in task_durations.items():
            logger.info(f"  {task_name}: {duration:.2f} 秒")
        logger.info(f"总耗时: {overall_elapsed:.2f} 秒")

    @task("lane_line_eval")
    def evaluate_lane_line(self, is_save_gpkg=True, is_save_svg=True) -> Optional[LineMatchResult]:
        """车道线评测"""
        logger.debug(f"任务:{self.input_intersection.result_save_title_prefix}的车道线评测开始....")
        if self.input_intersection.lane_lines.empty or self.truth_intersection.lane_lines.empty:
            if self.input_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道线评测。真值车道线为空。")
            if self.truth_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道线评测。待评测车道线为空。")
            return None

        # # FIXME 做实验中.....
        # save_here(self.truth_intersection,"./test_truth_intersection.pickle")
        # save_here(self.input_intersection,"./test_input_intersection.pickle")
        # save_here(self.tool_param,"./setting.pickle")

        le = LaneLineEvaluator(truth_intersection=self.truth_intersection,
                               input_intersection=self.input_intersection,
                               evaluator_setting=self.tool_param)
        res: LineMatchResult = le.compute_coverage()

        # le.visualize(result = res)
        if is_save_gpkg:
            # gpkg路径在这里
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_车道线评测(缓冲{self.tool_param.line_buffer_length_meter}米).gpkg")

            logger.debug(f"保存车道线评测结果gpkg路径为{gpkg_path}")
            le.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_车道线评测(缓冲{self.tool_param.line_buffer_length_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_车道线评测(缓冲{self.tool_param.line_buffer_length_meter}米).svg")
            le.plot(title=title, results=res, vector_file_path=svg_save_path)
        return res

    @task("lane_line_offset_statistic")
    def lane_line_offset_statistics(self, is_save_gpkg=True, is_save_svg=False) -> Optional[LineMatchResult]:
        """车道线偏差汇总。主要为了输出gpkg和csv，不输出svg"""
        logger.debug(f"任务:{self.input_intersection.result_save_title_prefix}的车道线偏差汇总统计任务开始....")
        if self.input_intersection.lane_lines.empty or self.truth_intersection.lane_lines.empty:
            if self.input_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道线偏差评测。真值车道线为空。")
            if self.truth_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道线偏差评测。待评测车道线为空。")
            return None

        le = LaneLineEvaluator(truth_intersection=self.truth_intersection,
                               input_intersection=self.input_intersection,
                               evaluator_setting=self.tool_param)
        le.set_truth_lines_buffer_dist(1.5)
        res: LineMatchResult = le.compute_coverage()

        # le.visualize(result = res)
        if is_save_gpkg:
            # gpkg路径在这里
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_车道线偏差汇总统计(1.5米).gpkg")

            logger.debug(f"保存车道线偏差汇总统计gpkg路径为{gpkg_path}，同时已输出csv")
            le.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_车道线偏差汇总统计(1.5米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_车道线偏差汇总统计(1.5米).svg")
            le.plot(title=title, results=res, vector_file_path=svg_save_path)
        return res

    @task("vectorize_only_lane_line_eval")
    def evaluate_vectorize_only_lane_line(self, is_save_gpkg=True, is_save_svg=True) -> Optional[LineMatchResult]:
        """车道线评测"""
        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的仅矢量化车道线评测开始")
        if self.input_intersection.lane_boundary_vectorize.empty or self.truth_intersection.lane_boundary_vectorize.empty:
            if self.input_intersection.lane_boundary_vectorize.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的仅矢量化车道线评测。待评测车道线为空。")
            if self.truth_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的仅矢量化车道线评测。真值车道线为空。")
            return None

        le = VectorizeOnlyLaneLineEvaluator(truth_intersection=self.truth_intersection,
                                            input_intersection=self.input_intersection,
                                            evaluator_setting=self.tool_param)
        res: LineMatchResult = le.compute_coverage()
        # le.visualize(result = res)
        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_仅矢量化车道线评测(缓冲{self.tool_param.line_buffer_length_meter}米).gpkg")
            logger.debug(f"保存仅矢量化车道线评测结果gpkg路径为{gpkg_path}")
            le.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_仅矢量化车道线评测(缓冲{self.tool_param.line_buffer_length_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_仅矢量化车道线评测(缓冲{self.tool_param.line_buffer_length_meter}米).svg")
            le.plot(title=title, results=res, vector_file_path=svg_save_path)

        return res

    @task("crosswalk_eval")
    def evaluate_crosswalk(self, is_save_gpkg=True, is_save_svg=True) -> Optional[PolygonEvaluateSummary]:
        """人行横道评测"""

        title = f"{self.input_intersection.result_save_title_prefix}_人行横道评测(匹配搜索范围{self.tool_param.polygon_match_distance_meter}米)"

        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的人行横道评测开始")
        if self.input_intersection.crosswalks.empty or self.truth_intersection.crosswalks.empty:
            if self.input_intersection.crosswalks.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的人行横道评测。真值人行横道为空。")
            if self.truth_intersection.crosswalks.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的人行横道评测。待评测人行横道为空。")
            return None

        ce = CrosswalkEvaluator(truth_intersection=self.truth_intersection, input_intersection=self.input_intersection,
                                evaluator_setting=self.tool_param)
        res = ce.evaluate()
        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{title}.gpkg")
            logger.debug(f"保存人行横道评测结果gpkg路径为{gpkg_path}")
            ce.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_人行横道评测(缓冲{self.tool_param.line_buffer_length_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_人行横道评测(匹配搜索范围{self.tool_param.polygon_match_distance_meter}米).svg")
            ce.save_svg(results=res, is_visualize=False, vector_file_path=svg_save_path, title=title)

        return res

    @task("arrow_eval")
    def evaluate_arrow(self, is_save_gpkg=True, is_save_svg=True) -> Optional[ArrowEvaluateSummary]:
        """箭头评测"""
        title = f"{self.input_intersection.result_save_title_prefix}_地面箭头评测(匹配搜索范围{self.tool_param.land_marking_match_distance_meter}米)"

        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的地面箭头评测开始")
        if self.input_intersection.arrows.empty or self.truth_intersection.arrows.empty:
            if self.input_intersection.arrows.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的地面箭头评测。真值地面箭头为空。")
            if self.truth_intersection.arrows.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的地面箭头评测。待评测地面箭头为空。")
            return None

        ae = ArrowEvaluator(truth_intersection=self.truth_intersection, input_intersection=self.input_intersection,
                            evaluator_setting=self.tool_param)
        res = ae.evaluate()
        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{title}.gpkg")
            logger.debug(f"保存地面箭头评测结果gpkg路径为{gpkg_path}")
            ae.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_地面箭头评测(缓冲{self.tool_param.land_marking_match_distance_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_地面箭头评测(匹配搜索范围{self.tool_param.land_marking_match_distance_meter}米).svg")
            ae.save_svg(results=res, is_visualize=False, vector_file_path=svg_save_path, title=title)

        return res

    @task("stopline_eval")
    def evaluate_stop_line(self, is_save_gpkg=True, is_save_svg=True) -> Optional[StopLineEvaluateResult]:
        """停止线评测"""
        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的停止线评测开始")

        if self.input_intersection.stop_lines.empty or self.truth_intersection.stop_lines.empty:
            if self.input_intersection.stop_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的停止线评测。真值停止线为空。")
            if self.truth_intersection.stop_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的停止线评测。待评测停止线为空。")
            return None

        se = StopLineEvaluator(truth_intersection=self.truth_intersection, input_intersection=self.input_intersection,
                               evaluator_setting=self.tool_param)
        res: StopLineEvaluateResult = se.evaluate()
        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_停止线评测(匹配搜索范围{self.tool_param.stopline_match_distance_meter}米).gpkg")
            logger.debug(f"保存停止线评测结果gpkg路径为{gpkg_path}")
            se.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_停止线评测(匹配搜索范围{self.tool_param.stopline_match_distance_meter}米).svg")
            logger.debug(f"保存停止线评测结果svg路径为{svg_save_path}")
            if svg_save_path:
                # 确保目标目录存在
                output_dir = os.path.dirname(svg_save_path)
                os.makedirs(output_dir, exist_ok=True)

            se.save_svg(evaluate_result=res, vector_file_path=svg_save_path)

        return res

    @task("traffic_light_eval")
    def evaluate_traffic_light(self, is_save_gpkg=True, is_save_svg=True) -> Optional[TrafficLightEvaluateResult]:
        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的信号灯评测开始")

        if self.input_intersection.traffic_lights.empty or self.truth_intersection.traffic_lights.empty:
            if self.input_intersection.traffic_lights.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的信号灯评测。真值信号灯为空。")
            if self.truth_intersection.traffic_lights.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的信号灯评测。待评测信号灯为空。")
            return None

        te = TrafficlightEvaluator(truth_intersection=self.truth_intersection,
                                   input_intersection=self.input_intersection,
                                   evaluator_setting=self.tool_param)

        res: TrafficLightEvaluateResult = te.evaluate(
            buffer_distance=self.tool_param.traffic_light_match_distance_meter)

        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_信号灯评测(检索范围{self.tool_param.traffic_light_match_distance_meter}米).gpkg")
            logger.debug(f"保存道路信号灯评测结果gpkg路径为{gpkg_path}")
            te.save_as_gkpg(results=res, gpkg_path=gpkg_path)
            # res.to_gdf().to_file(gpkg_path, layer="配对关系与记录测量结果", driver="GPKG")

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_信号灯评测(检索范围{self.tool_param.traffic_light_match_distance_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_信号灯评测(检索范围{self.tool_param.traffic_light_match_distance_meter}米).svg")
            te.plot(title=title, result=res, vector_file_path=svg_save_path)

        return res

    @task("road_boundary_eval")
    def evaluate_road_boundary(self, is_save_gpkg=True, is_save_svg=True) -> Optional[LineMatchResult]:
        """道路边界评测"""
        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的道路边界评测开始")

        if self.input_intersection.road_boundaries.empty or self.truth_intersection.road_boundaries.empty:
            if self.input_intersection.road_boundaries.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的道路边界评测。待评测道路边界为空。")
            if self.truth_intersection.road_boundaries.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的道路边界评测。真值道路边界为空。")
            return None

        le = RoadBoundaryEvaluator(truth_intersection=self.truth_intersection,
                                   input_intersection=self.input_intersection,
                                   evaluator_setting=self.tool_param)
        res: LineMatchResult = le.compute_coverage()
        # le.visualize(result = res)
        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_道路边界评测(缓冲{self.tool_param.line_buffer_length_meter}米).gpkg")
            logger.debug(f"保存道路边界评测结果gpkg路径为{gpkg_path}")
            le.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_道路边界评测(缓冲{self.tool_param.line_buffer_length_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_道路边界评测(缓冲{self.tool_param.line_buffer_length_meter}米).svg")
            le.plot(title=title, results=res, vector_file_path=svg_save_path)

        return res

    @task("vectorize_only_road_boundary_eval")
    def evaluate_vectorize_only_road_boundary(self, is_save_gpkg=True, is_save_svg=True) -> Optional[LineMatchResult]:
        """仅矢量化道路边界评测"""
        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的仅矢量化道路边界评测开始")

        if self.input_intersection.road_boundary_vectorize.empty or self.truth_intersection.road_boundaries.empty:
            if self.input_intersection.road_boundary_vectorize.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的仅矢量化道路边界评测。待评测道路边界为空。")
            if self.truth_intersection.road_boundaries.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的仅矢量化道路边界评测。真值仅矢量化道路边界为空。")
            return None

        le = VectorizeOnlyRoadBoundaryEvaluator(truth_intersection=self.truth_intersection,
                                                input_intersection=self.input_intersection,
                                                evaluator_setting=self.tool_param)
        res: LineMatchResult = le.compute_coverage()
        # le.visualize(result = res)
        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_仅矢量化道路边界评测(缓冲{self.tool_param.line_buffer_length_meter}米).gpkg")
            logger.debug(f"保存仅矢量化道路边界评测结果gpkg路径为{gpkg_path}")
            le.save_as_gkpg(results=res, gpkg_path=gpkg_path)

        if is_save_svg:
            title = f"{self.input_intersection.result_save_title_prefix}_仅矢量化道路边界评测(缓冲{self.tool_param.line_buffer_length_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_仅矢量化道路边界评测(缓冲{self.tool_param.line_buffer_length_meter}米).svg")
            le.plot(title=title, results=res, vector_file_path=svg_save_path)

        return res
    
    @task("relative_width_eval")
    def evaluate_relative_width(self, is_save_gpkg=True, is_save_svg=True):
        logger.info(f"任务:{self.input_intersection.result_save_title_prefix}的车道相对宽度评测开始")

        if self.input_intersection.lane_lines.empty or self.input_intersection.lane_centerlines.empty or self.truth_intersection.lane_lines.empty:
            if self.input_intersection.lane_centerlines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道相对宽度评测。真值车道中心线为空。")
            if self.input_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道相对宽度评测。真值车道线为空。")
            if self.truth_intersection.lane_lines.empty:
                logger.warning(f"无法进行{self.algo_box_id}路口的车道相对宽度评测。待评测车道线为空。")
            return None

        lwe = LaneWidthEvaluator(truth_intersection=self.truth_intersection,
                                 input_intersection=self.input_intersection,
                                 evaluator_setting=self.tool_param)
        res: GeoDataFrame = lwe.evaluate()
        # lwe.visualize(res)

        # FIXME 这里得改一下，visualize拿到plt再save fig这个方式不是很好
        if is_save_svg:
            # title = f"{self.input_intersection.result_save_title_prefix}_人行横道评测(缓冲{self.tool_param.line_buffer_length_meter}米)"
            svg_save_path = os.path.join(self.result_save_full_path,
                                         f"{self.input_intersection.result_save_title_prefix}_车道相对宽度误差统计.svg")

            # 确保目标目录存在
            output_dir = os.path.dirname(svg_save_path)
            os.makedirs(output_dir, exist_ok=True)

            logger.debug(f"保存相对宽度差评测结果gpkg路径为{svg_save_path}")

            lwe.visualize(res).savefig(svg_save_path, format='svg', bbox_inches='tight')

        if is_save_gpkg:
            gpkg_path = os.path.join(self.result_save_full_path,
                                     f"{self.input_intersection.result_save_title_prefix}_车道相对宽度误差统计.gpkg")

            # 确保目标目录存在
            output_dir = os.path.dirname(gpkg_path)
            os.makedirs(output_dir, exist_ok=True)

            logger.debug(f"保存相对宽度差评测结果gpkg路径为{gpkg_path}")

            res = res.to_crs(epsg=4326)
            res.to_file(filename=gpkg_path, driver="GPKG")
        # res.to_crs(epsg=4326)
        # # 保存为 GeoPackage 格式，几何信息会被保留
        # res.to_file("relative_width.gpkg", driver="GPKG")

        # # 保存为 CSV 格式
        # # 这里可以将几何信息转换成 WKT 格式，以便在 CSV 中存储
        # result_csv = result.copy()
        # result_csv["geometry"] = result_csv["geometry"].apply(lambda geom: geom.wkt)
        # result_csv.to_csv("output_result.csv", index=False)
        #
        # # 保存为 JSON 格式，GeoPandas 会按照 GeoJSON 格式输出
        # result.to_file("output_result.json", driver="GeoJSON")

        # raise NotImplementedError("该功能正在测试中")

    def evaluate_intersection_field(self, is_save_gpkg=True):
        raise NotImplementedError("该功能暂未实现")
