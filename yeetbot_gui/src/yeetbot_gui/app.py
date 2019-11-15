from Tkinter import *

from yeetbot_gui.asset_finder import AssetFinder
from yeetbot_gui.image_label import ImageLabel


class App(Canvas):
    def create_widgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "YEET"
        self.QUIT["fg"] = "red"
        self.QUIT["command"] = self.quit
        self.QUIT.pack({"side": "left"})

        size = 0.2

        self.angry = ImageLabel(self)
        self.angry.pack()
        self.angry.load(self.__assets.get_asset('angry'), size)

        self.idle = ImageLabel(self)
        self.idle.pack()
        self.idle.load(self.__assets.get_asset('idle'), size)

        self.dab = ImageLabel(self)
        self.dab.pack()
        self.dab.load(self.__assets.get_asset('dab'), size)

        self.idle_to_wave = ImageLabel(self)
        self.idle_to_wave.pack()
        self.idle_to_wave.load(self.__assets.get_asset('idle_to_wave'), size)

        self.wave = ImageLabel(self)
        self.wave.pack()
        self.wave.load(self.__assets.get_asset('wave'), size)

        self.jump = ImageLabel(self)
        self.jump.pack()
        self.jump.load(self.__assets.get_asset('jump'), size)

        self.kawaii = ImageLabel(self)
        self.kawaii.pack()
        self.kawaii.load(self.__assets.get_asset('kawaii'), size)

    def __init__(self, master=None):
        Canvas.__init__(self, master, width=1920, height=1080, bd=0)
        self.pack()
        self.__assets = AssetFinder()
        self.create_widgets()
