import rospkg
import os


class AssetFinder:
    def __init__(self):
        r = rospkg.RosPack()
        self.__path = r.get_path('yeetbot_gui')
        self.__path = os.path.join(self.__path, 'assets')
        self.__asset_map = {}
        
        self.find_asset('test', 'test.gif')

    def find_asset(self, name, file_name):
        if name in self.__asset_map.keys():
            raise ValueError("Asset '" + name + "' has already been found!")

        asset_path = os.path.join(self.__path, file_name)
        if not os.path.exists(asset_path):
            raise ValueError("Path '" + asset_path + "' does not exist!")

        self.__asset_map[name] = asset_path

    def get_asset(self, name):
        if name not in self.__asset_map.keys():
            raise ValueError("Asset '" + name + "' has not been loaded!")

        return self.__asset_map[name]
