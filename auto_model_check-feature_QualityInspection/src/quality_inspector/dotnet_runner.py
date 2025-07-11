# -*- coding: utf-8 -*-
# @Time    : 2025/6/18 20:14
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : dotnet_runner.py
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time
from typing import Any, Optional, IO, Union

from pydantic import BaseModel, field_validator

from src.errors import InputDataIllegal
from src.logger import logger
from src.quality_inspector.error_code import ErrorCode

# --- 配置参数 ---
DLL_FILE = "DNT.BevCoverage.dll"
APP_DIR = "/app"  # dotnet 程序所在的目录
OUTPUT_FILE_PATH = "/mnt/tasks/model/label_cover/output/output.json"
TASK_FILE_PATH = "/mnt/tasks/model/label_cover/input/tasks.json"


class BevCoverCalcTask(BaseModel):
    """建图平台给建图后覆盖度计算任务下发的tasks.json结构"""

    shp: str  # shp压缩包存储路径
    sd_link: str  # sd_link文件存储路径
    frame_id: str  # 算法框ID
    polygon: str  # 算法框WKT。这里暂时不转换，沿用纯wkt
    output: str

    @field_validator("frame_id", mode="before")
    @classmethod
    def convert_frame_id_to_str(cls, v: Any) -> str:
        """
        在验证前，将任何输入都转换为字符串。
        """
        return str(v)


class BevCoverCalcResult(BaseModel):
    # {"status": 1, "message": null, "score": 0.8845}
    status: int = 1  # 状态码，1通过0不通过
    score: float  # BEV建图覆盖度计算分数
    message: Optional[str] = None


class ModifiedBevCoverCalcTask(BevCoverCalcTask):
    """为GeoEva特制的部分"""

    algorithm_version: str  # 算法框版本
    build_task_id: Union[str, int, None] = "UNKNOWN"  # 任务ID，默认为UNKNOWN
    inspect_items: list = []  # 具体质检项


class DotnetTaskRunner:
    """
    一个用于启动、监控和获取.NET DLL后台任务输出的类。

    这个类封装了以下流程：
        1. 在指定目录下启动BevCoverCal进程。
        2. 轮询监控该进程是否执行完毕。
        3. 进程结束后，读取指定的output.json输出文件。
        4. 管理超时和错误情况。
    """

    def __init__(
        self,
        dll_file: str = DLL_FILE,
        app_dir: str = APP_DIR,
        output_path: str = OUTPUT_FILE_PATH,
        task_file_path: str = TASK_FILE_PATH,
        timeout_seconds: int = 20,
    ):
        """
        初始化任务运行器。

        Args:
            dll_file (str): 默认"DNT.BevCoverage.dll"。
            app_dir (str): `BevCoverCal目录，默认"/app"。
            output_path (str): 任务完成后生成的output.json文件的完整路径。
            timeout_seconds (int): 等待任务完成的超时时间，默认20秒。
        """
        self.dll_file = dll_file
        self.app_dir = app_dir
        self.output_path = output_path
        self.task_file_path = task_file_path
        self.timeout_seconds = timeout_seconds

        self.task_info: BevCoverCalcTask = self.get_task_info()
        self.algo_version: Optional[str] = self.__find_version_number(
            self.task_info.shp
        )
        self.build_task_id: Optional[str] = self.__find_build_task_id(
            self.task_info.shp
        )

        # 状态属性
        self.process: Optional[subprocess.Popen] = None
        self.result: Optional[BevCoverCalcResult] = None
        self.status_code = ErrorCode.INIT
        self.status_message: str = "任务尚未开始"

    def get_task_info_for_geoeva(self, inspect_items: list[str] = []) -> ModifiedBevCoverCalcTask:
        """为GeoEva组装一个task概要"""
        r = self.task_info.model_dump()
        r.update(
            {
                "algorithm_version": self.algo_version,
                "build_task_id": self.build_task_id,
                "inspect_items": inspect_items,
            }
        )
        return ModifiedBevCoverCalcTask(**r)

    def get_task_info(self) -> BevCoverCalcTask:
        """获取输入的task.json。如果输入异常，这里会直接提早退出"""
        try:
            with open(self.task_file_path, "r", encoding="utf-8") as f:
                return BevCoverCalcTask(**json.load(f))
        except Exception as e:
            raise InputDataIllegal(
                f"[GeoEva x BevCoverCal] bev覆盖度计算输入协议疑似有变: {e}"
            )

    def write_json_file(self, data: dict):
        # 确保目标目录存在
        _output_storage_dir = Path(self.output_path).parent
        _output_storage_dir.mkdir(exist_ok=True)

        with open(self.output_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=4)

    def refresh_input_json(self, data: ModifiedBevCoverCalcTask):
        with open(self.task_file_path, "w", encoding="utf-8") as f:
            logger.info(
                f"[GeoEva x BevCoverCal] 新增新版本号={self.algo_version}、建图任务号={self.build_task_id} 已更新到tasks.json"
            )
            json.dump(data.model_dump(), f, ensure_ascii=False, indent=4)
            logger.info("[GeoEva x BevCoverCal] 准备完毕! 塔塔开,一字莫塔塔开!")

    @property
    def get_output_json_data(self) -> dict:
        with open(self.output_path, "r", encoding="utf-8") as f:
            return json.load(f)

    @staticmethod
    def __find_version_number(path_string: str) -> Optional[str]:
        """获取版本号"""
        pattern = r"(v\d+\.\d+\.\d+)"
        match = re.search(pattern, path_string)
        if match:
            # match.group(0) 会返回整个匹配到的字符串，即 "v3.05.02_"
            # match.group(1) 只会返回第一个捕获组内的内容，即 "3.05.02"
            return match.group(1)
        return None

    @staticmethod
    def __find_build_task_id(path_string: str) -> Optional[str]:
        """获取中业任务ID号"""
        pattern = r"ID(\d+)\.zip$"
        match = re.search(pattern, path_string)
        if match:
            return match.group(1)
        return 'UNKNOWN'

    @staticmethod
    def bev_cover_calc_log(stream: IO[str], log_level: str):
        name = threading.current_thread().name
        try:
            for line in iter(stream.readline, ""):
                logger.log(log_level, line.strip())
        except Exception as e:
            logger.error(f'[GeoEva][{name}] 获取BevCoverCal输出时发生错误: {e}')
        finally:
            logger.debug(f"[GeoEva][{name}] 获取BevCoverCal输出完毕. 驾驶员自我感觉良好!")

    def run_and_wait(self) -> ErrorCode:
        """
        它会按顺序执行启动和等待，并返回执行结果，用Code表示是否成功。
        """
        # 1. 尝试启动进程
        start_success = self._start_process()
        if not start_success:
            return self.status_code

        # 1.1 创建并启动日志线程
        # 必须在 process 启动后才能访问 process.stdout/stderr
        stdout_thread = threading.Thread(
            target=self.bev_cover_calc_log,
            args=(self.process.stdout, "INFO"),  # 标准输出记为 INFO
            name='GUNDAM_STDOUT'
        )
        stderr_thread = threading.Thread(
            target=self.bev_cover_calc_log,
            args=(self.process.stderr, "ERROR"),  # 标准错误记为 ERROR
            name='GUNDAM_STDERR'
        )
        stdout_thread.start()
        stderr_thread.start()

        # 2. 如果启动成功，则开始等待进程完成
        # self._wait_for_completion()
        try:
            logger.info("[GeoEva] 等待BevCoverCal任务完成...")
            self.process.wait(timeout=self.timeout_seconds)
        except subprocess.TimeoutExpired as te:
            # 杀死进场并防止僵尸进程产生
            self.process.kill()
            self.process.wait()
            # 超时复制
            self.status_message = f"[GeoEva] BevCoverCal任务超时 (超过{self.timeout_seconds}秒)，已击毙。"
            self.status_code = ErrorCode.TIMEOUT

        # 等待进程都打印完
        stdout_thread.join()
        stderr_thread.join()
        # 处理结果
        self._wait_for_completion()

        return self.status_code

    def _start_process(self) -> bool:
        """[内部方法] 启动 dotnet 子进程。"""
        command = ["dotnet", f"{APP_DIR}/{self.dll_file}"]

        # 预检
        if not os.path.isdir(self.app_dir):
            self.status_message = f"[GeoEva] 目录不存在->{self.app_dir}"
            self.status_code = ErrorCode.INTERNAL_ERROR
            return False

        dll_path = os.path.join(self.app_dir, self.dll_file)
        if not os.path.isfile(dll_path):
            self.status_message = f"[GeoEva] 文件不存在->{dll_path}"
            self.status_code = ErrorCode.INTERNAL_ERROR
            return False

        logger.info(
            f"[GeoEva] BevCoverCal启动: {' '.join(command)}！--ガンダムで出撃する！--"
        )
        try:
            logger.debug(f"cmd={command}")
            self.process = subprocess.Popen(
                command,
                cwd=self.app_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.status_message = f"[GeoEva] BevCoverCal进程 PID: {self.process.pid}"
            logger.info(self.status_message)
            return True
        except FileNotFoundError:
            self.status_message = f"[GeoEva] 未找到BevCoverCal脚本"
            self.status_code = ErrorCode.INTERNAL_ERROR
            return False
        except Exception as e:
            self.status_message = (
                f"[GeoEva x BevCoverCal] BevCoverCal启动任务时发生未知错误: {e}"
            )
            self.status_code = ErrorCode.INTERNAL_ERROR
            return False

    def _wait_for_completion(self) -> None:
        """[内部方法] 轮询等待进程结束并处理结果。"""
        return_code = self.process.returncode
        # if return_code is not None:
        logger.info("[GeoEva] BevCoverCal任务已结束")
        if return_code == 0:
            # 进程成功退出，检查并读取文件
            if os.path.exists(self.output_path):
                try:
                    from time import sleep

                    sleep(1)
                    with open(self.output_path, "r", encoding="utf-8") as f:
                        d = json.load(f)
                        logger.debug(d)
                        self.result = BevCoverCalcResult(**d)
                    self.status_message = f"[GeoEva] BevCoverCal任务成功完成并已读取输出文件。{self.result.model_dump()}"
                    self.status_code = ErrorCode.SUCCESS
                except Exception as e:
                    logger.exception(e)
                    self.status_message = f"[GeoEva] BevCoverCal任务output.json读取或解析失败: {e}"
                    self.status_code = ErrorCode.INTERNAL_ERROR

            else:
                self.status_message = f"[GeoEva] BevCoverCal进程已结束，但未找到输出文件。"
                self.status_code = ErrorCode.INTERNAL_ERROR
        else:
            # 避免因超时杀死覆盖了对应的错误码和message
            if self.status_code != ErrorCode.TIMEOUT:
                # 进程异常退出
                self.status_message = f"[GeoEva] BevCoverCal任务进程异常退出，退出码: {return_code}。"
                self.status_code = ErrorCode.INTERNAL_ERROR

