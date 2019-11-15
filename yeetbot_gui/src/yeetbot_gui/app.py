from Tkinter import *

from yeetbot_gui.asset_finder import AssetFinder
from yeetbot_gui.image_label import ImageLabel


class App(Frame):
    def create_widgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "YEET"
        self.QUIT["fg"] = "red"
        self.QUIT["command"] = self.quit
        self.QUIT.pack({"side": "left"})

        self.image = ImageLabel(self)
        self.image.pack()
        self.image.load(self.__assets.get_asset('kawaii'))

    def __init__(self, master=None):
        Frame.__init__(self, master, width=1920, height=1080, bd=0)
        self.pack()
        self.__assets = AssetFinder()
        self.create_widgets()
