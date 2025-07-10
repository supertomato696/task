import yaml

from src.logger import logger
from src.model import Config


class ConfigManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigManager, cls).__new__(cls)
            cls._instance.config = None  # 配置字典，用于存储不同类型的配置
        return cls._instance

    def load_config(self, file_path: str) -> Config:
        """ 加载工具配置 """
        with open(file_path, 'r', encoding='utf8') as file:
            config_data = yaml.safe_load(file)
            logger.debug(f"读取{file_path}完毕：{config_data}")
            self.config = Config(**config_data)
            logger.debug(f"数据要放在这个路径下哦：{self.config.config.file_paths.data_path}")
            return self.config
