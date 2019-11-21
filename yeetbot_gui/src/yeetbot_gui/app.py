from Tkinter import *

from yeetbot_gui.asset_finder import AssetFinder
from yeetbot_gui.image_label import ImageLabel


class App(Canvas):
    def write_yeetbot_speech(self, text):
        x_pos = 300
        y_pos = 1000
        self.speech_label_text.set(text)
        self.speech_label.place(relx=0.5, rely=0.8, anchor=CENTER)

    def clear_screen(self):
        self.angry.place_forget()
        self.kawaii.place_forget()
        self.idle.place_forget()
        self.idle_to_wave.place_forget()
        self.wave.place_forget()
        self.dab.place_forget()
        self.jump.place_forget()
        self.speech_label.place_forget()

    def create_idle_screen(self):
        self.idle.place(x=1500, y=700)

    def create_receiving_request_screen(self):
        self.wave.place(x=1500, y=700)

    def create_receiving_tool_screen(
            self, early=False, on_time=False, late=False):
        if early:
            self.kawaii.place(x=1500, y=700)
        elif on_time:
            self.jump.place(x=1500, y=700)
        elif late:
            self.angry.place(x=1500, y=700)
        else:
            raise ValueError("Neither early, on_time or late!!!")

    def create_travelling_screen(self):
        self.dab.place(x=1500, y=700)

    def process_new_state(state_msg):
        self.clear_screen()
        if state_msg.current_state == state_msg.IDLE:
            self.create_idle_screen()
        elif state_msg.current_state == state_msg.RECEIVING_REQUEST:
            self.create_receiving_request_screen()
        elif state_msg.current_state == state_msg.RECEIVING_TOOL_EARLY:
            self.create_receiving_tool_screen(early=True)
        elif state_msg.current_state == state_msg.RECEIVING_TOOL_ON_TIME:
            self.create_receiving_tool_screen(on_time=True)
        elif state_msg.current_state == state_msg.RECEIVING_TOOL_LATE:
            self.create_receiving_tool_screen(late=True)
        elif state_msg.current_state == state_msg.TRAVELLING:
            self.create_travelling_screen()
        else:
            raise ValueError("Unknown state received: {}".format(state_msg.current_state))

        self.last_state = state_msg

    def create_widgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "YEET"
        self.QUIT["fg"] = "red"
        self.QUIT["command"] = self.quit
        self.QUIT.place(relx=0.5, rely=0.5, anchor=CENTER)

        size = 0.2

        self.angry = ImageLabel(self)
        self.angry.place()
        self.angry.load(self.__assets.get_asset('angry'), size)

        self.idle = ImageLabel(self)
        self.idle.place()
        self.idle.load(self.__assets.get_asset('idle'), size)

        self.dab = ImageLabel(self)
        self.dab.place()
        self.dab.load(self.__assets.get_asset('dab'), size)

        self.idle_to_wave = ImageLabel(self)
        self.idle_to_wave.place()
        self.idle_to_wave.load(self.__assets.get_asset('idle_to_wave'), size)

        self.wave = ImageLabel(self)
        self.wave.place()
        self.wave.load(self.__assets.get_asset('wave'), size)

        self.jump = ImageLabel(self)
        self.jump.place()
        self.jump.load(self.__assets.get_asset('jump'), size)

        self.kawaii = ImageLabel(self)
        self.kawaii.place()
        self.kawaii.load(self.__assets.get_asset('kawaii'), size)

        self.speech_label_text = StringVar()
        self.speech_label = Label(
            self, textvariable=self.speech_label_text, wraplength=1200)
        self.speech_label.place()

    def __init__(self, master=None):
        Canvas.__init__(self, master, width=1600, height=2560, bd=0)
        self.pack()
        self.last_state = None
        self.__assets = AssetFinder()
        self.create_widgets()
        self.clear_screen()
        self.create_idle_screen()
        self.write_yeetbot_speech("Hello world")
