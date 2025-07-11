import multiprocessing
import os
from pathlib import Path
from typing import Dict, Optional

import numpy as np
from geopandas import GeoDataFrame, clip
from pandas import DataFrame
from pydantic import BaseModel, Field, computed_field

from src.data_process.file_read import DataReader
from src.intersection_evaluator.init_intersection import Intersection
from src.logger import logger
from src.model.config import EvaluateElementProfile
from src.quality_inspector.error import BadBevIntegrityError
from src.quality_inspector.error_code import ErrorCode
from src.quality_inspector.inspectors.base import BaseInspector
from src.quality_inspector.models.task import QualitiInspectTask
from src.quality_inspector.utils.data_integrity import DataIntegrityChecker
from src.tools.common import json_dump
from src.tools.filter import GeoDataFrameFilter


class InspectResult(BaseModel):
    """
    评测结果
    """

    error_code: int = ErrorCode.INIT  # 错误码
    build_task_id: Optional[str] = 'UNKNOWN'  # 建图任务ID
    algo_box_id: str = Field(default=None, alias="algorithm_box_id")  # 算法框ID
    algo_version: Optional[str] = Field(
        default=None, alias="algorithm_version"
    )  # 算法框ID
    expect_score: float = 0  # 期望输入分数
    actual_score: float = 0  # 实际计算分数
    message: str = None  # 质检信息

    bev_score: float = 0  # BEV覆盖度输入分数

    @computed_field
    @property
    def status(self) -> int:
        return 0 if self.error_code != ErrorCode.SUCCESS else 1


class InspectorManager:
    def __init__(
        self,
        task_paras: QualitiInspectTask,
        material_path: Path,
        upload_path: Path = None,
    ):
        """
        注意upload_path是为了适配建图平台上传资料特调的，一般不需要传
        """
        self.task_paras: QualitiInspectTask = task_paras

        self.algo_result_path = material_path

        if material_path.parent.name == "input":
            self.output_dir = material_path.parent.parent.joinpath("output")
        else:
            self.output_dir = material_path.parent.joinpath("output")

        if upload_path:
            self.upload_path = upload_path
            os.makedirs(self.upload_path, exist_ok=True)
        else:
            self.upload_path = None

        logger.info(f"[初始化] 输出目录为{self.output_dir}")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # 构建待评测路口对象
        self.intersection: Intersection = self.__build_intersection()
        # 读取SD link，并统计其总长
        self.sd_link: GeoDataFrame = self.task_paras.sd_link_gdf
        # 获取评测项列表
        self.inspectors = self.__build_inspectors()

        # 保存目录路径
        self.summary = []

        # 初始化评测结果
        self.inspect_result: InspectResult = self.__init_result()

        # 记录跑失败的子评测任务
        self.failed_tasks = []

    def __init_result(self):
        """初始化评测报告结构体"""
        res = InspectResult()
        res.algo_box_id = self.task_paras.algo_box_id
        res.algo_version = self.task_paras.algo_version
        res.build_task_id = self.task_paras.build_task_id
        res.expect_score = self.task_paras.expected_score

        return res

    def get_raw_failure_report(
        self, message: str, bev_score: float = None, code: ErrorCode = None
    ) -> InspectResult:
        """
        获取一个失败的output.json模板
        支持设置bev覆盖度计算分数、错误信息、自定义错误码
        """
        res: InspectResult = self.__init_result()
        res.message = message

        res.error_code = ErrorCode.INTERNAL_ERROR if not code else code
        res.bev_score = bev_score if not bev_score else bev_score

        return res

    def run_single_inspect_task(self, inspector: BaseInspector):
        """
        对单个质检项执行完整的检查、获取和保存结果的流程。
        这个方法将作为并行任务的目标函数。
        """
        logger.info(f"Inspector {inspector.__class__.__name__} 出撃する!")
        inspector.inspect()
        # 获取质检结果
        try:
            statistic, result = inspector.get_result()
            logger.debug(f"stat: {statistic}")
            # 保存单个质检项结果
            self.__save_single_inspect_result(statistic, result)
            # 这里返回状态或结果，方便后续处理
            return True
        except Exception as e:
            # msg = f"评测项{inspector.__class__.__name__}失败: {e}"
            # logger.error(msg)
            return False

    @staticmethod
    def run_inspector_task(inspector: "BaseInspector"):
        """
        一个独立的、可被序列化的函数，用于在子进程中执行单个质检项。

        Args:
            inspector (BaseInspector): 要执行的质检器实例。

        Returns:
            A tuple: (is_success, result_data)
            - 如果成功: (True, (statistic, result))
            - 如果失败: (False, (error_code, error_message))
        """
        inspector_name = inspector.__class__.__name__
        try:
            logger.info(f"开始执行评测项: {inspector_name}...")
            inspector.inspect()
            statistic, result = inspector.get_result()
            logger.info(f"评测项: {inspector_name} 执行成功。")
            # 返回成功标志和数据
            return True, (statistic, result)
        except Exception as e:
            # 捕获所有可能的异常
            msg = f"评测项 {inspector_name} 执行失败: {e}"
            logger.error(msg, exc_info=True)  # exc_info=True会记录完整的堆栈跟踪
            # 返回失败标志和错误信息
            return False, (ErrorCode.INTERNAL_ERROR, msg)  # 使用字符串作为错误码示例

    def run_inspections(self):
        if self.task_paras.check_integrity:
            logger.info("[完整性检查] 已配置完整性检查")
            try:
                dic = DataIntegrityChecker(Path(self.algo_result_path))
                dic.check_integrity()
            except BadBevIntegrityError as err:
                logger.error(
                    f"code: {err.code}\nmessge:{err.message}\nerror_info: {err.error_info}"
                )
                self.inspect_result.error_code = err.code
                self.inspect_result.message = f"{err.message}: {err.error_info}"

                output_file = f"{self.task_paras.build_task_id}_{self.task_paras.algo_box_id}_output.json"

                if self.upload_path:
                    json_dump(
                        self.inspect_result.model_dump(),
                        str(self.upload_path.joinpath(output_file).resolve()),
                    )
                else:
                    json_dump(
                        self.inspect_result.model_dump(),
                        str(self.output_dir.joinpath(output_file).resolve()),
                    )

                logger.info(f"[质检结果] {self.inspect_result.model_dump()}")
                return self.inspect_result
        else:
            logger.info("[完整性检查] 未配置完整性检查")

        if len(self.inspectors) <= 5:
            logger.debug(f"评测项共计{len(self.inspectors)}项")
            return self.run_inspection_linear()
        else:
            logger.debug(f"评测项共计{len(self.inspectors)}项")
            return self.run_inspection_in_parallel(num_processes=5, task_timeout=300)

    def run_inspection_linear(self) -> InspectResult:
        logger.info(f"准备进行共{len(self.inspectors)}项质检。")

        for inspector in self.inspectors:
            cur: BaseInspector = inspector
            cur.inspect()
            # 获取质检结果
            statistic, result = cur.get_result()  # 统计信息，质检详细结果
            logger.debug(f"stat: {statistic}")

            # 保存单个质检项结果过程如保存文件,保存到总表。已实现。
            self.__save_single_inspect_result(statistic, result)

        # 汇总结果，将总表GeoDataframe的内容输出为csv，已实现。
        res = self.__summary_and_save()
        return res

    def run_inspection_in_parallel(
        self, num_processes: int = 5, task_timeout: int = 300
    ) -> InspectResult:
        """
        使用多进程并行执行所有评测任务，并带有超时控制。

        Args:
            num_processes (int, optional): 工作进程数。默认为CPU核心数-1。
            task_timeout (int, optional): 单个评测任务的超时时间（秒）。默认为300秒（5分钟）。
        """
        from multiprocessing import TimeoutError as MPTimeoutError  # 别名以防混淆

        if not self.inspectors:
            logger.warning("没有需要执行的评测项。")
            self.inspect_result.message = "没有需要执行的评测项。"
            return self.inspect_result

        if num_processes is None:
            num_processes = max(1, multiprocessing.cpu_count() - 1)

        logger.info(
            f"启动并行评测，使用{num_processes}个进程，单个任务超时时间:{task_timeout}秒。"
        )

        with multiprocessing.Pool(processes=num_processes) as pool:
            # 异步提交所有任务，并保存 AsyncResult 对象和 inspector 的对应关系
            results_map = {
                inspector.__class__.__name__: pool.apply_async(
                    self.run_inspector_task, args=(inspector,)
                )
                for inspector in self.inspectors
            }

            for inspector_name, async_result in results_map.items():
                try:
                    # 等待结果，并设置超时
                    success, data = async_result.get(timeout=task_timeout)

                    if success:
                        statistic, result = data
                        self.__save_single_inspect_result(statistic, result)
                    else:
                        error_code, msg = data
                        self.__save_fail(inspector_name, error_code, msg)

                except MPTimeoutError:
                    # 捕获超时异常
                    msg = f"评测项{inspector_name}执行超过{task_timeout}秒，判定为超时终止。"
                    logger.error(msg)
                    self.__save_fail(inspector_name, ErrorCode.TIMEOUT, msg)

                except Exception as e:
                    msg = f"评测项{inspector_name}发生意外错误: {e}"
                    logger.error(msg, exc_info=True)
                    self.__save_fail(inspector_name, ErrorCode.INTERNAL_ERROR, msg)

        logger.info("所有并行评测任务已完成处理，开始进行最终汇总。")
        res = self.__summary_and_save()

        # 可以在这里打印最终的失败报告
        if self.failed_tasks:
            logger.warning(
                f"[InspectorManager] 评测完成，但存在失败项。失败的评测项列表: {self.failed_tasks}"
            )
            return res
        else:
            logger.info("[InspectorManager] Tasks Done. Returning test result")
            return res

    @staticmethod
    def __get_filtered_geometries(gdf: GeoDataFrame, filter_conditions: Dict = None):
        """
        获取过滤后的几何要素，支持坐标系转换。

        :param gdf: 输入的 GeoDataFrame。
        :param filter_conditions: 过滤条件字典（可选）。
        :return: 过滤后的 GeoDataFrame。
        """
        logger.debug(f"传入的筛选条件为{filter_conditions}")
        gdf_filter = GeoDataFrameFilter(gdf)  # 创建 GeoDataFrameFilter 实例

        if filter_conditions:
            # 过滤 GeoDataFrame
            gdf = gdf_filter.filter_gdf(filter_conditions)

        # return ensure_single_geometries(gdf)  # 确保只有单一几何体
        return gdf

    def __build_intersection(self):
        # 获取传入的路口配置
        intersection_cfg = self.task_paras.intersection_config

        intersection_dict = self.task_paras.intersection_config.model_dump()
        # 按要素读取
        for name, item in intersection_cfg.get_element_fields():
            name: str
            item: EvaluateElementProfile

            # 获取要素路径
            feature_file_path = self.algo_result_path.joinpath(
                f"{item.prefix}.{item.suffix}"
            )
            logger.debug(f"读取要素：{name}, 目标文件：{str(feature_file_path)}")

            if not feature_file_path.exists():
                logger.warning(
                    f"未包含要素 {name} 的数据，目标文件：{str(feature_file_path)}"
                )
                intersection_dict[name] = GeoDataFrame()
                continue

            # 读取要素数据(默认转投影坐标系 + 移除z坐标)
            object_gdf = DataReader().get_this_file_ready(
                file_path=str(feature_file_path.resolve()),
                is_to_utm=True,
                is_ignore_z=True,
            )

            if not object_gdf.empty:
                object_gdf = self.__get_filtered_geometries(
                    gdf=object_gdf, filter_conditions=item.filter_conditions
                )
            logger.info(
                f"[数据准备] 在{feature_file_path}路径中找到{name}要素文件，要素数量为{object_gdf.shape[0]}"
            )
            intersection_dict[name] = object_gdf

        # 构建路口
        intersection = Intersection(**intersection_dict)
        return intersection

    def __build_sd_link(self, sd_link_path: Path):
        # 读取原始的sd link
        sd_link_gdf = DataReader().get_this_file_ready(
            file_path=str(sd_link_path.resolve()), is_to_utm=True, is_ignore_z=True
        )
        sd_link_crs = sd_link_gdf.crs
        sd_link_gdf["length"] = sd_link_gdf.geometry.length
        logger.debug(f"sd origin length: {sd_link_gdf['length'].sum()}")

        # 获取算法框
        algorithm_polygon = self.task_paras.algo_box_geom

        # # 一定要给几何。建任务的时候就必填了。这段暂时注释
        # algorithm_box_id = self.task_paras.algo_box_id
        # algorithm_polygon_raw = self.task_paras.algorithm_polygon
        # # 不能两个都为空
        # if not algorithm_polygon_raw and not algorithm_box_id:
        #     raise ValueError(f'Missing required parameters: algorithm_box_id, algorithm_polygon')
        #
        # algorithm_polygon = None
        # if algorithm_polygon_raw:
        #     # 转换多边形字符串
        #     algorithm_polygon = wkt.loads(algorithm_polygon_raw)
        #
        # else:
        #     # 根据算法框ID在本地找
        #     algo_manager = AlgorithmBoxManager(str(Path('./data/algorithm_boxes')))
        #     try:
        #         algorithm_polygon = algo_manager.get_algo_box_by_id(algorithm_box_id)
        #     except Exception as e:
        #         logger.error(f"获取算法框（id={algorithm_box_id}）几何失败: {e}")
        # 读取算法框成功，尝试对sd link裁剪
        if algorithm_polygon:
            algo_poly_gdf = GeoDataFrame(
                {"geometry": [algorithm_polygon]}, crs="EPSG:4326"
            )
            algo_poly_gdf = algo_poly_gdf.to_crs(sd_link_crs)

            clipped_sd_link_gdf = clip(sd_link_gdf, algo_poly_gdf)
            clipped_sd_link_gdf["length"] = clipped_sd_link_gdf.geometry.length
            logger.debug(f"sd clipped length: {clipped_sd_link_gdf['length'].sum()}")

            return clipped_sd_link_gdf

        # 否则直接返回原始sd link
        return sd_link_gdf

    def __build_inspectors(self):
        instances = []
        inspector_params = {
            "sd_link": self.sd_link,
            "intersection": self.intersection,
            "data_dir": self.algo_result_path,
        }

        inspect_items = self.task_paras.inspect_items

        # 如果质检项为空，返回空列表
        if not inspect_items:
            return []

        # 特殊值"All"
        if "All" in inspect_items or "ALL" in inspect_items or "all" in inspect_items:
            logger.info("[初始化] 选择了所有质检项")
            inspector_instances = [
                InspectorClass(**inspector_params)
                for InspectorClass in BaseInspector.registry.values()
            ]
            return inspector_instances

        for alias in inspect_items:
            cls = BaseInspector.registry.get(alias, None)
            if not cls:
                logger.error(f"The inspector called {alias} was not found")
            else:
                instances.append(cls(**inspector_params))

        return instances

    def __save_single_inspect_result(self, statistic, result):
        task_id = self.task_paras.build_task_id
        algo_box_id = self.task_paras.algo_box_id
        feat_type = statistic.get("feature_type")
        name = statistic.get("inspector_name")

        if self.upload_path:
            output_file = self.upload_path.joinpath(
                f"{task_id}_{algo_box_id}_log_{feat_type}_{name}.gpkg"
            )
        else:
            output_file = self.output_dir.joinpath(
                f"{task_id}_{algo_box_id}_log_{feat_type}_{name}.gpkg"
            )
        # 确保文件不在
        if output_file.is_file():
            output_file.unlink()
        # 保存文件
        self.__write_to_gpkg(str(output_file), result)
        # 保存到总表
        self.summary.append(statistic)

    def __save_success(self, code: ErrorCode, message=None):
        # TODO __summary_and_save计划使用这个函数
        ...

    def __save_fail(self, inspector_name: str, code: ErrorCode, message):
        # TODO __summary_and_save计划使用这个函数
        pass

        logger.debug("保存了一只错误的东西")
        # 将失败的任务名加入列表
        if inspector_name not in self.failed_tasks:
            self.failed_tasks.append(inspector_name)

    def __summary_and_save(self) -> InspectResult:
        filename = f"{self.task_paras.build_task_id}_{self.task_paras.algo_box_id}"

        weight_sum = self.__calc_weight_sum()

        if weight_sum <= 0:
            self.inspect_result.message = (
                f"质检项配置数量为{self.inspectors},权重和为零。请检查"
            )
            logger.warning(self.inspect_result.message)
        else:
            if self.sd_link.empty:
                logger.warning(
                    "sd_link未输入！这里暂时置为0，需要考虑兼容这个分支。比如不输出按里程计数的部分"
                )
                sd_link_len = 0
            else:
                sd_link_len = self.sd_link["length"].sum()

            logger.info(
                f"[结果计算] 权重总和:{weight_sum}, SD_link总里程:{sd_link_len}米。开始汇总结果.."
            )

            summary_df = DataFrame(self.summary)
            mask_p00 = summary_df["inspector_level"] == "P00"
            mask_total_zero = summary_df["total_count"] == 0
            summary_df["score"] = np.where(
                mask_p00 | mask_total_zero,
                0,
                1 - summary_df["log_count"].astype(float) / summary_df["total_count"],
            )

            # 计算合计分
            if mask_p00.any():
                self.inspect_result.actual_score = 0
                self.inspect_result.error_code = ErrorCode.BAD_INTEGRITY
                self.inspect_result.message = "存在P00质检项问题，得分置为0"
            else:
                numerator = (summary_df["score"] * summary_df["level_weight"]).sum()
                final_score = numerator / weight_sum
                self.inspect_result.actual_score = final_score.round(3)

                # # 建图平台不建议在没有错误的时候写message
                # if final_score >= self.task_paras.expected_score:
                #     self.inspect_result.message = ""
                # else:
                #     self.inspect_result.message = "Lower than expect."

                # 取三位小数
                summary_df["score"] = summary_df["score"].round(3)

                # 保存汇总表
                col_zh = ["质检项名称", "等级", "权重", "类型", "总数", "log数", "分数"]
                summary_file = f"{filename}_log_count.csv"
                # 用utf_8_sig编码更方便用户直接打开文件

                if self.upload_path:
                    summary_df.to_csv(
                        str(self.upload_path.joinpath(summary_file).resolve()),
                        index=True,
                        index_label="序号",
                        header=col_zh,
                        encoding="utf_8_sig",
                    )
                else:
                    summary_df.to_csv(
                        str(self.output_dir.joinpath(summary_file).resolve()),
                        index=True,
                        index_label="序号",
                        header=col_zh,
                        encoding="utf_8_sig",
                    )

        output_file = f"{filename}_output.json"
        # output_file = 'output.json'

        if self.failed_tasks:
            self.inspect_result.error_code = ErrorCode.PARTIAL_FAIL
            self.inspect_result.message = f"[GeoEva] PartialFail: {self.failed_tasks}"
        else:
            # 设置错误码
            self.inspect_result.error_code = ErrorCode.SUCCESS

        if self.upload_path:
            json_dump(
                self.inspect_result.model_dump(),
                str(self.upload_path.joinpath(output_file).resolve()),
            )
        else:
            json_dump(
                self.inspect_result.model_dump(),
                str(self.output_dir.joinpath(output_file).resolve()),
            )

        logger.info(f"[质检结果] {self.inspect_result.model_dump()}")
        logger.info("[InspectorManager] Summary Done.")

        return self.inspect_result

    def __calc_weight_sum(self):
        weight_sum = 0
        for i in self.inspectors:
            weight_sum += i.level.value
        return weight_sum

    @staticmethod
    def __write_to_gpkg(gpkg_path: str, result: Dict):
        # 保存文件
        if not result:
            logger.warning("Result is empty")
            return

        first = True
        for layer_name, layer_data in result.items():
            mode = "w" if first else "a"
            try:
                layer_data.to_file(
                    gpkg_path, layer=layer_name, driver="GPKG", mode=mode
                )
                first = False
            except Exception as e:
                logger.error(f"保存 {layer_name} 失败: {e}\nGPKG文件: {gpkg_path}")
        logger.info(f"保存 {layer_name} 成功, GPKG文件: {gpkg_path}")
