#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import String
from yeetbot_msgs.msg import YEETBotUserResponse, YEETBotUserChoices, YEETBotState

def init():
    global pub
    global state
    global inventory
    global pi

    #serial setup and handshake
    pi = serial.Serial(
        port = '/dev/ttyACM0',
        baudrate=115200,
        timeout=0.5)
    done = 0

    pi.write(">yeet<")

    #while not done:
    #    cmd = pi.read_until("<")
    #    if cmd == ">ack<":
    #        done = 1
    #    else:
    #        print("Unexpected: '",cmd,"' from speech pi")
    
    state = 0
    inventory = []
    rospy.init_node('yeet_speech')
    pub_response = rospy.Publisher("/user_response", YEETBotUserResponse, queue_size=10)
    pub_doa = rospy.Publisher("/yeetbot_voice_direction", Float64, queue_size=10)
    pub_state = rospy.Publisher("/yeetbot_state", YEETBotState, queue_size=10)
    rospy.Subscriber("/user_choices", YEETBotUserChoices, choiceCallback)
    rospy.Subscriber("/yeetbot_state", YEETBotState, stateCallback) 
    rospy.Subscriber("/text_msg", String, speechCallback)

def choiceCallback(user_choices):
    global inventory
    inventory = user_choices.user_options
    serial_inventory = ">i/" + ','.join(inventory) + "<"
    pi.write(serial_inventory)
    
def stateCallback(yeetbot_state):
    global state
    state = yeetbot_state.current_state
    serial_state = ">s/" + str(state) + "<"
    pi.write(serial_state)

def speechCallback(speech_string):
    yeet_speak = speech_string.text
    serial_speech = ">m/" + yeet_speak + "<"
    pi.write(serial_speech)

def main():
    global pi
    print "Setting up..."
    init()
    print "Setup completed"
    while not rospy.is_shutdown():
        cmd = pi.read_until("<")
        if cmd:
            topic, serial_info = cmd[:3], cmd[3:-1]
            #user_choice
            if topic == ">u/":
                msg = YEETBotUserResponse()
                serial_info = list(serial_info.split(","))
                msg.choice = int(serial_info[0])
                msg.invalid_choice = True if serial_info[1] == "t" else False
                pub_response.publish(msg)
                rospy.loginfo(msg)
                
            #doa
            elif topic == ">a/":
                msg = float(serial_info)
                pub_doa.publish(msg)
                rospy.loginfo(msg)
                
            #state
            elif topic == ">s/":
                msg = YEETBotState()
                msg.current_state = int(msg)
                pub_state.publish(msg)
                rospy.loginfo(msg)

    pi.write("quit")

if __name__ == '__main__':
    main()
