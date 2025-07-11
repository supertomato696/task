import os
import sys
from datetime import datetime
import loguru

# 项目根路径
PROJCECT_ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# 设置日志目录路径
log_path = os.path.join(PROJCECT_ROOT_PATH, 'log')

# 确保日志目录存在
os.makedirs(log_path, exist_ok=True)


class __ApiAutoLog:
    """
    利用loguru封装接口自动化项目日志记录器
    """

    def __new__(cls, *args, **kwargs):
        encoding = "utf-8"  # 写入日志文件时编码格式为utf-8

        log_name = datetime.now().strftime("%Y-%m-%d")  # 以时间命名日志文件，格式为"年-月-日"
        # sink = "../log/{}.log".format(log_name)  # 日志记录文件路径
        sink = f"{PROJCECT_ROOT_PATH}/log/{log_name}.log"  # 日志记录文件路径

        level = "DEBUG"  # 记录的最低日志级别为DEBUG
        # level = "INFO"  # 记录的最低日志级别为DEBUG

        rotation = "500MB"  # 日志文件最大为500MB，超过则新建文件记录日志
        retention = "1 week"  # 日志保留时长为一星期，超时则清除

        enqueue = True  # 多线程多进程时保证线程安全

        format = "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>"

        # loguru.logger.configure(extra={"task_id": "main"})
        # format = (
        #     "<green>{time:HH:mm:ss}</green>|"
        #     "<level>{level: <7}</level> |"
        #     "<cyan>{extra[task_id]: <10}</cyan> | "
        #     "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - "
        #     "<level>{message}</level>"
        # )

        # 移除默认处理器
        loguru.logger.remove()

        loguru.logger.add(
            encoding=encoding,
            sink=sink,
            level=level,
            rotation=rotation,
            retention=retention,
            enqueue=enqueue,

            backtrace=True,
            diagnose=True,

            format=format

            # serialize=True,
        )

        loguru.logger.add(
            sys.stdout, level=level, format=format
        )
        return loguru.logger


logger = __ApiAutoLog()

if __name__ == '__main__':
    logger.debug("yahaha")
