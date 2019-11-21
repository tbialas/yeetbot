from PyQt4 import QtGui, QtCore

from yeetbot_gui.asset_finder import AssetFinder


class App(QtGui.QMainWindow):
    def write_yeetbot_speech(self, text):
        self.speech_bubble.move(86, 629)
        self.speech_bubble.show()
        #self.speech_label_text.set(text)
        #self.speech_label.place(relx=0.5, rely=0.41, anchor=CENTER)

    def clear_screen(self):
        self.angry.hide()
        self.kawaii.hide()
        self.idle.hide()
        self.wave.hide()
        self.dab.hide()
        self.jump.hide()
        #self.speech_label.hide()

    def create_idle_screen(self):
        self.idle.move(844, 1579)
        self.idle.show()

    def create_receiving_request_screen(self):
        self.wave.move(844, 1466)
        self.wave.show()

    def create_receiving_tool_screen(
            self, early=False, on_time=False, late=False):
        if early:
            self.kawaii.move(916, 1566)
            self.kawaii.show()
        elif on_time:
            self.jump.move(640, 1201)
            self.jump.show()
        elif late:
            self.angry.move(893, 1468)
            self.angry.show()
        else:
            raise ValueError("Neither early, on_time or late!!!")

    def create_travelling_screen(self):
        self.dab.move(671, 1546)
        self.dab.show()

    def process_new_state(self, state_msg):
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
        
        """
        self.speech_label_text = StringVar()
        self.speech_label = Label(
            self.speech_bubble, textvariable=self.speech_label_text, 
            wraplength=1050, font=("monospace", 30))
        """

        self.angry = QtGui.QLabel(self)
        self.angrymovie = QtGui.QMovie(self.__assets.get_asset('angry'))
        self.angry.setMovie(self.angrymovie)
        self.angrymovie.start()
        self.angry.resize(self.angrymovie.frameRect().size())

        self.idle = QtGui.QLabel(self)
        self.idlemovie = QtGui.QMovie(self.__assets.get_asset('idle'))
        self.idle.setMovie(self.idlemovie)
        self.idlemovie.start()
        self.idle.resize(self.idlemovie.frameRect().size())

        self.dab = QtGui.QLabel(self)
        self.dabmovie = QtGui.QMovie(self.__assets.get_asset('dab'))
        self.dab.setMovie(self.dabmovie)
        self.dabmovie.start()
        self.dab.resize(self.dabmovie.frameRect().size())

        self.wave = QtGui.QLabel(self)
        self.wavemovie = QtGui.QMovie(self.__assets.get_asset('wave'))
        self.wave.setMovie(self.wavemovie)
        self.wavemovie.start()
        self.wave.resize(self.wavemovie.frameRect().size())

        self.jump = QtGui.QLabel(self)
        self.jumpmovie = QtGui.QMovie(self.__assets.get_asset('jump'))
        self.jump.setMovie(self.jumpmovie)
        self.jumpmovie.start()
        self.jump.resize(self.jumpmovie.frameRect().size())

        self.kawaii = QtGui.QLabel(self)
        self.kawaiimovie = QtGui.QMovie(self.__assets.get_asset('kawaii'))
        self.kawaii.setMovie(self.kawaiimovie)
        self.kawaiimovie.start()
        self.kawaii.resize(self.kawaiimovie.frameRect().size())


    def __init__(self, master=None):
        super(App, self).__init__()
        self.setGeometry(0, 0, 1600, 2560)

        #self.pack()
        self.last_state = None
        self.__assets = AssetFinder()
        self.create_widgets()
        self.clear_screen()
        self.create_idle_screen()
        self.write_yeetbot_speech("Hello world")
        self.show()
