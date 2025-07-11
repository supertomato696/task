import re
import sys
import zipfile
from pathlib import Path
from typing import List

from src.errors import InputDataIllegal, BrokeFile
from src.logger import logger
from src.quality_inspector.inspector_manager import InspectResult


def unzip_file(zip_path: Path, destination_dir: Path):
    """
    将zip文件解压到指定目录。

    :param zip_path: .zip 文件的路径。
    :param destination_dir: 解压目标目录。
    """
    logger.debug(f"检测到建图成果压缩包: {zip_path},准备解压到: {destination_dir}")
    try:
        with zipfile.ZipFile(zip_path, 'r') as zf:
            zf.extractall(path=destination_dir)
        logger.debug("解压完成。")
    except zipfile.BadZipFile:
        raise BrokeFile(f"{zip_path}不是一个有效的zip文件或已损坏。")

    except Exception as e:
        logger.error(f"解压过程中发生未知错误: {e}", file=sys.stderr)
        raise


def find_algorithm_result(base_dir: Path, pattern: str = None) -> Path:
    """
    在基础目录下查找算法结果。
    优先使用正则表达式查找.zip压缩包，如果找不到，则按查找同名文件夹。

    Raise:
        InputDataIllegal
        FileNotFoundError
    """

    core_pattern_str = r'^(?:\d+_)?map_shp_v\d+(?:\.\d+){2}(?:_ID\d+)?'

    if pattern:
        # 指定了名字
        # 使用 def 定义清晰的内部函数
        def zip_checker(name):
            return name == f"{pattern}.zip"

        def dir_checker(name):
            return name == pattern
    else:
        # 未指定名字，默认按照平台的pattern
        zip_regex = re.compile(rf"^{core_pattern_str}\.zip$")
        dir_regex = re.compile(f"^{core_pattern_str}$")

        # 使用def定义清晰的内部函数
        def zip_checker(name):
            return bool(zip_regex.match(name))

        def dir_checker(name):
            return bool(dir_regex.match(name))

    # 遍历目录
    found_zips: List[Path] = []
    found_dirs: List[Path] = []
    for item in base_dir.iterdir():
        if item.is_file() and zip_checker(item.name):
            found_zips.append(item)
        elif item.is_dir() and dir_checker(item.name):
            found_dirs.append(item)

    if found_zips:
        if len(found_zips) > 1:
            raise InputDataIllegal(f"Multiple Zip files: {found_zips}")

        zip_to_process = found_zips[0]
        # 目标解压目录名 = zip文件名
        target_dir = zip_to_process.with_suffix('')

        if not target_dir.is_dir():
            logger.info(f"解压{zip_to_process}中..")
            unzip_file(zip_to_process, target_dir)
        else:
            logger.debug(f"目录{target_dir.name}已存在，跳过解压")

        return target_dir

    if found_dirs:
        if len(found_dirs) > 1:
            raise InputDataIllegal(f"Multiple dirs: {found_zips}")

        return found_dirs[0]

    raise FileNotFoundError("Task input not found.")


def find_required_path(base_dir: Path, filename: str) -> Path:
    """在基础目录下查找必需的文件，找不到则抛出异常。"""
    path = base_dir / filename
    if not path.exists():
        raise FileNotFoundError(f"{filename}在目录{base_dir}中未找到。")
    return path


def get_inspector_manager(task_dir: Path, upload_dir: Path = None):
    import json

    from src.app_path import APP_ROOT
    from src.data_process.file_read import DataReader

    from src.intersection_evaluator.init_intersection import IntersectionType
    from src.quality_inspector.inspector_manager import InspectorManager
    from src.quality_inspector.models.task import QualitiInspectTask, ElementDesc
    from src.tools.common import yaml_safe_load

    """主逻辑函数"""
    logger.info(f"[初始化] 开始处理任务，输入目录: {task_dir}")

    potential_input_dir = task_dir / "input"

    if potential_input_dir.is_dir():
        input_dir = potential_input_dir
        logger.debug(f"[初始化] 发现'input'子目录，使用路径: {input_dir}")
    else:
        input_dir = task_dir
        logger.info(f"[初始化] 未发现'input'子目录，使用任务根目录: {input_dir}")

    # 必须保证task文件和sd_link文件齐全。tasks.json跟带评测的建图成果处于同一级目录
    task_file_path = find_required_path(input_dir, 'tasks.json')

    with open(task_file_path, encoding='utf-8') as f:
        task_params = json.load(f)

    material_path: Path = find_algorithm_result(base_dir=input_dir,
                                                pattern=task_params.get("algorithm_result") if task_params.get(
                                                    "algorithm_result") else None)

    tsk = QualitiInspectTask(
        **task_params
    )

    if not tsk.intersection_config:
        config_path = Path(APP_ROOT) / 'src' / 'quality_inspector' / 'instersection.yaml'
        d = yaml_safe_load(config_path)['intersection']
        d.update({
            "type": IntersectionType.input,
            "name": tsk.algo_box_id,
            "version": tsk.algo_version,
            "nickname": tsk.build_task_id,
            "is_ignored_z": True
        })
        tsk.intersection_config = ElementDesc(**d)

    try:
        sd_link_path = find_required_path(input_dir, 'sd_link.geojson')
        tsk.sd_link_gdf = DataReader().get_this_file_ready(str(sd_link_path))
        logger.info("[初始化] 读取Link文件.")
    except FileNotFoundError:
        logger.warning("[初始化] 未找到有效Link文件.")
        pass

    im = InspectorManager(tsk, material_path, upload_dir)
    return im


def run_inspect_task(task_dir: Path, upload_dir: Path = None) -> InspectResult:
    im = get_inspector_manager(task_dir=task_dir, upload_dir=upload_dir)
    result = im.run_inspections()
    return result


if __name__ == '__main__':
    run_inspect_task(Path(f"/data/geoEva"))
    ...
