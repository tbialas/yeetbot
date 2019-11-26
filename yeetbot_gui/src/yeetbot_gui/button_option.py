from PyQt4 import QtGui, QtCore


class ButtonOption(QtGui.QLabel):
    def __init__(self, master, option_number=0, text="Example", 
                 callback=None, height=200, width=800, choice_id=0):
        super(QtGui.QLabel, self).__init__(master)
        self.button = QtGui.QPushButton(self)
        self.button.clicked.connect(
            lambda :callback(option_number, choice_id))
        self.button.resize(100, 100)
        self.button.move(0, 0)
        self.button.show()

        self.text = QtGui.QLabel(self)
        self.text.setAlignment(QtCore.Qt.AlignLeft)
        self.text.setWordWrap(True)
        self.text.resize(width-100, height)
        self.text.move(100, 0)
        self.text.setFont(QtGui.QFont("Monospace", 26))
        self.text.setText(text)
        self.text.show()

        self.resize(width, height)
    
    def remove(self):
        self.setParent(None)
        self.text.deleteLater()
        self.button.deleteLater()
        self.deleteLater()
