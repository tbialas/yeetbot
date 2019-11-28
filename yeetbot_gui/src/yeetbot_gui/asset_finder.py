import rospkg
import os


class AssetFinder:
    def __init__(self):
        r = rospkg.RosPack()
        self.__path = r.get_path('yeetbot_gui')
        self.__path = os.path.join(self.__path, 'assets')
        self.__asset_map = {}
        
        self.find_asset('angry'       , 'toolbot_angry.gif'       )
        self.find_asset('dab'         , 'toolbot_dab.gif'         )
        self.find_asset('idle'        , 'toolbot_idle.gif'        )
        self.find_asset('idle_to_wave', 'toolbot_idle_to_wave.gif')
        self.find_asset('wave'        , 'toolbot_wave.gif'        )
        self.find_asset('jump'        , 'toolbot_happy_jump.gif'  )
        self.find_asset('kawaii'      , 'toolbot_kawaii.gif'      )
        self.find_asset('background'  , 'background.png'          )
        self.find_asset('speech'      , 'speech_bubble.png'       )

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
