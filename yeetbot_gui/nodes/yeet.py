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

    global state
    state = 0
    def state_swap_cb(timer):
        global state
        state += 1
        state %= 6
        state_msg = YEETBotState()
        state_msg.current_state = state
        app.process_new_state(state_msg)

    timer = rospy.Timer(rospy.Duration(3), state_swap_cb)

    root.exec_()

if __name__ == '__main__':
    main()
