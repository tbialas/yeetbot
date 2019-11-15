#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import Tkinter

from yeetbot_gui.app import App
from yeetbot_msgs.msg import YEETBotState, YEETBotUserResponse, YEETBotUserChoices

def main():
    rospy.init_node("GUI")

    root = Tkinter.Tk()
    app = App(master=root)

    def speech_text_cb(text_msg):
        app.write_yeetbot_speech(text_msg.data)

    rospy.Subscriber("/text_msg", String, speech_text_cb)

    try:
        app.mainloop()
    except rospy.exceptions.ROSException:
        pass

    try:
        root.destroy()
        root.update()
    except Tkinter._tkinter.TclError:
        pass

if __name__ == '__main__':
    main()
