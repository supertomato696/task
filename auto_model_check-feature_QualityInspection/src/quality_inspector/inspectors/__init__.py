import pkgutil
import importlib
from src.quality_inspector.inspectors import arrow
from src.logger import logger

def _import_inspector_modules(package_name):
    package = importlib.import_module(package_name)
    for finder, name, ispkg in pkgutil.walk_packages(package.__path__, package_name + '.'):
        if ispkg:
            continue
        
        split_names = name.rsplit('.', 1)
        if split_names[-1].startswith('bev_'):
            importlib.import_module(name)
            logger.debug(f'[初始化]发现质检项: {split_names[-1]}')

package_name = __name__
_import_inspector_modules(package_name)

