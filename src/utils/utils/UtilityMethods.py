import os
from ament_index_python.packages import get_package_share_path

class UtilityMethods:
    @staticmethod
    def getPackageConfig(pkg_name: str = "control") -> str:
        pkg_share = get_package_share_path(pkg_name).parents[1]
        pkg_root = os.path.dirname(os.path.dirname(pkg_share))
        pkg_config = os.path.join(pkg_root, f"src/{pkg_name}/config")
        if not os.path.exists(pkg_config):
            raise FileNotFoundError("Could not find project root with /config directory")
        return pkg_config
    
    @staticmethod
    def getPackageModels(pkg_name: str = "control") -> str:
        pkg_share = get_package_share_path(pkg_name).parents[1]
        pkg_root = os.path.dirname(os.path.dirname(pkg_share))
        pkg_models = os.path.join(pkg_root, f"src/{pkg_name}/models")
        if not os.path.exists(pkg_models):
            raise FileNotFoundError("Could not find project root with /models directory")
        return pkg_models