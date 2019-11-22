#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from PyQt4 import QtGui
import sys

from yeetbot_gui.app import App
from yeetbot_msgs.msg import YEETBotState, YEETBotUserResponse, YEETBotUserChoices

def main():
    rospy.init_node("GUI")

    root = QtGui.QApplication(sys.argv)

    app = App()

    def speech_text_cb(text_msg):
        app.write_yeetbot_speech(text_msg.data)

    rospy.Subscriber("/text_msg", String, speech_text_cb)

    def state_swap_cb(state_msg):
        app.process_new_state(state_msg)

    rospy.Subscriber("/yeetbot_state", YEETBotState, state_swap_cb)

    root.exec_()

if __name__ == '__main__':
    main()
