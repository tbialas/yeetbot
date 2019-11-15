#!/usr/bin/env python
import rospy

import Tkinter

from yeetbot_gui.app import App

def main():
    root = Tkinter.Tk()
    app = App(master=root)

    rospy.init_node("GUI")

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
