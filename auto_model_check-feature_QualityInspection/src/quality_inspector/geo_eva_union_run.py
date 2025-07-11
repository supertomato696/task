# -*- coding: utf-8 -*-
# @Time    : 2025/6/20 18:01
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : geo_eva_union_run.py
import sys
from pathlib import Path

from src.logger import logger
from src.quality_inspector import DEFAULT_INSPECT_ITEMS
from src.quality_inspector.dotnet_runner import ModifiedBevCoverCalcTask
from src.quality_inspector.error_code import ErrorCode
from src.quality_inspector.inspector_manager import InspectResult
from src.quality_inspector.main_inspector import get_inspector_manager
from src.quality_inspector.utils.utils import archive_inspect_output_files

from src.quality_inspector.dotnet_runner import DotnetTaskRunner
from src.quality_inspector.inspector_manager import InspectorManager


def run_bev_calc(dotnet_task_runner: DotnetTaskRunner) -> bool:
    """运行bev覆盖度任务，返回是否成功"""
    # bev覆盖度计算服务是否成功的标志
    is_bev_calc_success = False
    try:
        # 尝试启动BevCoverCal：
        code: ErrorCode = dotnet_task_runner.run_and_wait()
        if code != ErrorCode.SUCCESS:
            logger.error(dotnet_task_runner.status_message)
            is_bev_calc_success = False
        else:
            # 执行成功
            bev_calc_code = dotnet_task_runner.result.status
            if bev_calc_code == 1:  # 无事发生，继续
                is_bev_calc_success = True
            else:  # 如果Bev覆盖计算任务失败了
                logger.error(f"[GeoEva x BevCoverCal] BevCoverCal任务状态异常: {code}")
                dotnet_task_runner.result.message = (
                    f"[GeoEva x BevCoverCal] BevCoverCal任务状态异常: {code}"
                )
                is_bev_calc_success = False
    except Exception as e:
        logger.exception(
            f"[GeoEva x BevCoverCal] BevCoverCal启动任务时发生未知错误: {e}"
        )
        dotnet_task_runner.result.message = f"[GeoEva x BevCoverCal] BevCoverCal任务状态异常: {e}."
        is_bev_calc_success = False

    return is_bev_calc_success


def feat_BevCoverCal(inspect_task_dir: Path):
    logger.info("===== GeoEva质检 x BevCoverCal 模式 =====")

    if not inspect_task_dir.is_dir():
        logger.error(f"[GeoEva质检] 提供的路径'{inspect_task_dir}'不是一个有效的目录。")
        sys.exit(1)
    # 调起dotnet taskRunner
    dtr = DotnetTaskRunner()

    is_bev_calc_success = run_bev_calc(dtr)
    if is_bev_calc_success:
        # 保存一手数据
        bev_cover_cale_score = dtr.result.score

        # 为了防止GeoEva错误导致整体流水线阻塞。这里使用trycatch捕捉错误。
        # 但是仍然透传bev_cover_cal的分数
        try:
            # 构造GeoEva专用tasks.json
            # 目前平台应该不会传任何inspect_items。
            # 先默认是空的inspect_items
            geoeva_task_input: ModifiedBevCoverCalcTask = dtr.get_task_info_for_geoeva()
            # GeoEva质检结果指定保存目录
            upload_path = Path(Path(dtr.output_path).parent, "upload")
            

            if not geoeva_task_input.inspect_items:
                geoeva_task_input.inspect_items = DEFAULT_INSPECT_ITEMS
                logger.warning("[GeoEva] 检测到质检项为空，已默认补充默认项")

            # 使用符合规格的，并且补充了质检项的geoeva_task_input。更新tasks.json
            dtr.refresh_input_json(geoeva_task_input)

            # 初始化GeoEva
            im: InspectorManager = get_inspector_manager(
                task_dir=inspect_task_dir, upload_dir=upload_path
            )

            geo_eva_inspect_res: InspectResult = im.run_inspections()
            # 填充bev分数
            geo_eva_inspect_res.bev_score = bev_cover_cale_score
            dtr.write_json_file(geo_eva_inspect_res.model_dump())

            # 后处理，保证除了output.json之外其他文件都丢到upload里面了
            archive_inspect_output_files(Path(dtr.output_path).parent)

        except Exception as e:
            im: InspectorManager = get_inspector_manager(inspect_task_dir)
            output_json: InspectResult = im.get_raw_failure_report(
                f"[GeoEva] Oooops...{e}"
            )
            # output_json.update({
            #     "bev_score": bev_cover_cale_score,
            #     "status": -1
            # })
            logger.exception(e)
            dtr.write_json_file(output_json.model_dump())

    else:
        logger.warning(
            "[GeoEva x BevCoverCal] BevCoverCal任务状态异常退出，输出output.json"
        )
        broke_result = InspectResult(
            **{
                "error_code": ErrorCode.INTERNAL_ERROR,
                "build_task_id": dtr.build_task_id,
                "algorithm_box_id": dtr.task_info.frame_id,
                "message": dtr.status_message,
            }
        )

        dtr.write_json_file(broke_result.model_dump())

    logger.info("===== GeoEva质检 x BevCoverCal Finished =====")
