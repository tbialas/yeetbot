from PyQt4 import QtGui, QtCore
import queue

from yeetbot_gui.asset_finder import AssetFinder
from yeetbot_gui.button_option import ButtonOption


class App(QtGui.QMainWindow):
    def set_response_cb(self, response_cb):
        self.response_cb = response_cb

    def write_new_choices(self, choices_msg):
        try:
            self.choices_queue.put_nowait(choices_msg)
        except queue.Full:
            self.choices_queue.get_nowait()
            self.choices_queue.put_nowait(choices_msg)

    def process_choices_queue(self):
        try:
            choice = self.choices_queue.get_nowait()
        except queue.Empty:
            return
        for button in self.option_buttons:
            button.remove()
        self.option_buttons = []
        num_options = len(choice.user_options)
        h = 1000 / num_options - 20
        num = 0
        for option in choice.user_options:
            self.option_buttons.append(ButtonOption(
                self, option_number=num, text=option, 
                callback=self.response_cb, height=h))
            self.option_buttons[-1].move(17, num * (h + 20) + 1425)
            self.option_buttons[-1].show()
            num += 1
        self.show()

    def write_yeetbot_speech(self, text):
        try:
            self.speech_queue.put_nowait(text)
        except queue.Full:
            self.speech_queue.get_nowait()
            self.speech_queue.put_nowait(text)

    def process_speech_queue(self):
        try:
            text = self.speech_queue.get_nowait()
        except queue.Empty:
            return
        self.speech_bubble.move(86, 629)
        self.speech_bubble.show()
        self.speech_label.setText(text)
        self.speech_label.show()

    def clear_screen(self):
        self.angry.hide()
        self.kawaii.hide()
        self.idle.hide()
        self.wave.hide()
        self.dab.hide()
        self.jump.hide()
        self.speech_label.hide()
        self.option_buttons = []

    def create_idle_screen(self):
        self.idle.move(844, 1579)
        self.idlemovie.jumpToFrame(0)
        self.idle.show()

    def create_receiving_request_screen(self):
        self.wave.move(844, 1466)
        self.wavemovie.jumpToFrame(0)
        self.wave.show()

    def create_receiving_tool_screen(
            self, early=False, on_time=False, late=False):
        if early:
            self.kawaii.move(916, 1566)
            self.kawaiimovie.jumpToFrame(0)
            self.kawaii.show()
        elif on_time:
            self.jump.move(640, 1201)
            self.jumpmovie.jumpToFrame(0)
            self.jump.show()
        elif late:
            self.angry.move(893, 1468)
            self.angrymovie.jumpToFrame(0)
            self.angry.show()
        else:
            raise ValueError("Neither early, on_time or late!!!")

    def create_travelling_screen(self):
        self.dab.move(671, 1546)
        self.dabmovie.jumpToFrame(0)
        self.dab.show()

    def process_new_state(self, state_msg):
        try:
            self.state_queue.put_nowait(state_msg)
        except queue.Full:
            self.state_queue.get_nowait()
            self.state_queue.put_nowait(state_msg)

    def process_state_queue(self):
        try:
            state_msg = self.state_queue.get_nowait()
        except queue.Empty:
            return
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
        self.bg = QtGui.QLabel(self) 
        pixmap = QtGui.QPixmap(self.__assets.get_asset('background'))
        self.bg.setPixmap(pixmap)
        self.bg.resize(pixmap.width(), pixmap.height())
        self.bg.move(0, 0)

        self.QUIT = QtGui.QPushButton("YEET", self)
        self.QUIT.clicked.connect(QtCore.QCoreApplication.instance().quit)
        self.QUIT.resize(self.QUIT.minimumSizeHint())#40, 20)
        self.QUIT.move(1500, 10)

        self.speech_bubble = QtGui.QLabel(self)
        pixmap = QtGui.QPixmap(self.__assets.get_asset('speech'))
        self.speech_bubble.setPixmap(pixmap)
        self.speech_bubble.resize(pixmap.width(), pixmap.height())

        self.speech_label = QtGui.QLabel(self.speech_bubble)
        self.speech_label.move(173, 154)
        self.speech_label.setAlignment(QtCore.Qt.AlignCenter)
        self.speech_label.resize(1030, 400)
        self.speech_label.setWordWrap(True)
        self.speech_label.setFont(QtGui.QFont("Monospace", 30))

        self.angry = QtGui.QLabel(self)
        self.angrymovie = QtGui.QMovie(self.__assets.get_asset('angry'))
        self.angry.setMovie(self.angrymovie)
        self.angrymovie.setSpeed(240)
        self.angrymovie.start()
        self.angry.resize(self.angrymovie.frameRect().size())

        self.idle = QtGui.QLabel(self)
        self.idlemovie = QtGui.QMovie(self.__assets.get_asset('idle'))
        self.idle.setMovie(self.idlemovie)
        self.idlemovie.setSpeed(240)
        self.idlemovie.start()
        self.idle.resize(self.idlemovie.frameRect().size())

        self.dab = QtGui.QLabel(self)
        self.dabmovie = QtGui.QMovie(self.__assets.get_asset('dab'))
        self.dab.setMovie(self.dabmovie)
        self.dabmovie.setSpeed(240)
        self.dabmovie.start()
        self.dab.resize(self.dabmovie.frameRect().size())

        self.wave = QtGui.QLabel(self)
        self.wavemovie = QtGui.QMovie(self.__assets.get_asset('wave'))
        self.wave.setMovie(self.wavemovie)
        self.wavemovie.setSpeed(240)
        self.wavemovie.start()
        self.wave.resize(self.wavemovie.frameRect().size())

        self.jump = QtGui.QLabel(self)
        self.jumpmovie = QtGui.QMovie(self.__assets.get_asset('jump'))
        self.jump.setMovie(self.jumpmovie)
        self.jumpmovie.setSpeed(240)
        self.jumpmovie.start()
        self.jump.resize(self.jumpmovie.frameRect().size())

        self.kawaii = QtGui.QLabel(self)
        self.kawaiimovie = QtGui.QMovie(self.__assets.get_asset('kawaii'))
        self.kawaii.setMovie(self.kawaiimovie)
        self.kawaiimovie.setSpeed(240)
        self.kawaiimovie.start()
        self.kawaii.resize(self.kawaiimovie.frameRect().size())


    def __init__(self, master=None):
        super(App, self).__init__()
        self.setGeometry(0, 0, 1600, 2560)

        self.speech_queue = queue.Queue(maxsize=2)
        self.state_queue = queue.Queue(maxsize=2)
        self.choices_queue = queue.Queue(maxsize=2)

        # QTimers to process ROS message inputs on the main thread
        self.speech_timer = QtCore.QTimer()
        self.speech_timer.timeout.connect(self.process_speech_queue)
        self.speech_timer.start(500) # 2 Hz

        self.state_timer = QtCore.QTimer()
        self.speech_timer.timeout.connect(self.process_state_queue)
        self.state_timer.start(500) # 2 Hz

        self.choices_timer = QtCore.QTimer()
        self.choices_timer.timeout.connect(self.process_choices_queue)
        self.choices_timer.start(500) # 2 Hz

        self.choice_buttons = []

        self.last_state = None
        self.__assets = AssetFinder()
        self.create_widgets()
        self.clear_screen()
        self.create_idle_screen()
        self.write_yeetbot_speech("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Cras pellentesque ante purus, ac porta orci maximus ac. Nunc at maximus purus. Cras sit amet lacus interdum, elementum lectus a, interdum sem. Nullam venenatis porta viverra. Nullam posuere lacus vel tellus pharetra, ut pellentesque turpis sagittis. Nulla non ") 

        self.show()
