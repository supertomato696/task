
import time
import math
import numpy as np
import logging
import os
from enum import Enum


class FSDMapLogger:
    def __init__(self) -> None:
        self.logger = logging.getLogger('my_logger')
        self.logger.setLevel(logging.DEBUG)
        self.formatter = logging.Formatter('%(message)s')
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        console_handler.setFormatter(self.formatter)
        self.logger.addHandler(console_handler)

        np.set_printoptions(suppress=True)  # 不使用科学计数法
        np.set_printoptions(precision=8)  # 精度为 8 位

    class titleColor(Enum):
        RED = "0;41m"
        GREEN = "0;42m"
        YELLOW = "0;43m"
        BLUE = "0;44m"

    @staticmethod
    def time_str():
        return time.strftime('%d %H:%M:%S',  time.localtime())

    def set_log_file_path(self, path: str):
        folder = os.path.dirname(path)
        if not os.path.exists(folder):
            os.makedirs(folder)
        file_handler = logging.FileHandler(path)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(self.formatter)
        self.logger.addHandler(file_handler)

    def info(self, *args):
        self.logger.info(self.time_str()+" "+"".join(str(item) for item in args))

    def debug(self, *args):
        self.logger.info("\033[92m" + self.time_str()+" "+"".join(str(item) for item in args) + "\033[00m")

    def warning(self, *args):
        self.logger.info("\033[93m" + self.time_str()+" "+"".join(str(item) for item in args) + "\033[00m")

    def error(self, *args):
        self.logger.info("\033[91m" + self.time_str()+" "+"".join(str(item) for item in args) + "\033[00m")

    def mid_title(self, title: str, color: titleColor = titleColor.RED, title_len: int = 30):
        space = 0
        if len(title) < title_len:
            space = math.floor((title_len - len(title))/2)
        log_str = " "*space + title + " "*space
        self.logger.info("\n\033[1;" + color.value + log_str + "\033[00m")

    def title(self, title: str, color: titleColor = titleColor.RED, space: int = 6):
        log_str = " "*space + title + " "*space
        self.logger.info("\033[1;" + color.value + "\n"+log_str + "\033[0m\n")


log = FSDMapLogger()
