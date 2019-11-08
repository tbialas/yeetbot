#!/usr/bin/env python
import rospy

import Tkinter

from yeetbot_gui.app import App

def main():
    rospy.init_node("GUI")

    root = Tkinter.Tk()
    app = App(master=root)

    try:
        app.mainloop()
    except rospy.exceptions.ROSException:
        pass

    try:
        root.destroy()
    except Tkinter._tkinter.TclError:
        pass

if __name__ == '__main__':
    main()
