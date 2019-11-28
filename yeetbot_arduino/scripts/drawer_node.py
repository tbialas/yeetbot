#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import serial
import time

from std_msgs.msg import String
from yeetbot_msgs.msg import YEETBotDrawerStates, YEETBotItemStates

arduino = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate=115200)
STATES = [[0,0,0,0],[0,0,0,0]]    
OLD_DRAWER_STATES = [0,0,0,0]
OLD_ITEM_STATES = [1,1,1,1]

def callback(drawer_states_msg):
    if drawer_states_msg.plier_drawer:
        command = ">1o<"
    else:
        command = ">1c<"
    arduino.write(command)
    #print (command)
    #arduino.read_until("<") #Don't do anything with this info for now
    
    if drawer_states_msg.screw_driver_drawer:
        command = ">2o<"
    else:
        command = ">2c<"
    arduino.write(command)
    #print (command)
    #arduino.read_until("<") #Don't do anything with this info for now
    
    if drawer_states_msg.wire_stripper_drawer:
        command = ">3o<"
    else:
        command = ">3c<"
    arduino.write(command)
    #print (command)
    #arduino.read_until("<") #Don't do anything with this info for now
      
    if drawer_states_msg.vernier_caliper_drawer:
        command = ">4o<"
    else:
        command = ">4c<"
    arduino.write(command)
    #print (command)
    #arduino.read_until("<") #Don't do anything with this info for now
    
        

def talker():
    #arduino.open()
    
    done = 0
    print ("Waiting for Arduino")
    while not done:
        cmd = arduino.read_until("<")
        if cmd == ">yeet<":
            done = 1
        else:
            print("Unexpected: '",cmd,"' from arduino")     
    
    
    rospy.init_node('drawers', anonymous=True)                  ###DRAWERS###
    
    drawer_status_pub = rospy.Publisher('/drawer_state_changed', YEETBotDrawerStates, queue_size=2)     ###CHANGE CHATTER###
    item_status_pub = rospy.Publisher('/item_state_changed', YEETBotItemStates, queue_size=2)
    
    rospy.Subscriber('/drawer_state_request', YEETBotDrawerStates, callback)
    
    drawer_msg = YEETBotDrawerStates()
    item_msg = YEETBotItemStates()
    
    STATES = getStates()
    OLD_DRAWER_STATES = STATES[0]
    OLD_ITEM_STATES = STATES[1]
    drawer_msg.plier_drawer = OLD_DRAWER_STATES[0]
    drawer_msg.screw_driver_drawer = OLD_DRAWER_STATES[1]
    drawer_msg.wire_stripper_drawer = OLD_DRAWER_STATES[2]
    drawer_msg.vernier_caliper_drawer = OLD_DRAWER_STATES[3]
    drawer_status_pub.publish(drawer_msg)
    
    item_msg.pliers =  [OLD_ITEM_STATES[0]]
    item_msg.screw_drivers = [OLD_ITEM_STATES[1]]
    item_msg.wire_strippers = [OLD_ITEM_STATES[2]]
    item_msg.vernier_calipers = [OLD_ITEM_STATES[3]]
    item_status_pub.publish(item_msg)
    
    
    print("Initial message sents. About to enter loop")
    
    rate = rospy.Rate(1) # 1 Hz
    
    while not rospy.is_shutdown():
        
        
        # drawer_msg.x = y
        # drawer_status_pub.publish(drawer_msg)
        
        
        STATES = getStates()
        DRAWER_STATES = STATES[0]
        ITEM_STATES = STATES[1]
        
        if OLD_DRAWER_STATES != DRAWER_STATES:
            OLD_DRAWER_STATES = DRAWER_STATES
            drawer_msg.plier_drawer = DRAWER_STATES[0]
            drawer_msg.screw_driver_drawer = DRAWER_STATES[1]
            drawer_msg.wire_stripper_drawer = DRAWER_STATES[2]
            drawer_msg.vernier_caliper_drawer = DRAWER_STATES[3]
            drawer_status_pub.publish(drawer_msg)
            
        if OLD_ITEM_STATES != ITEM_STATES:
            OLD_ITEM_STATES = ITEM_STATES
            item_msg.pliers =  [ITEM_STATES[0]]
            item_msg.screw_drivers = [ITEM_STATES[1]]
            item_msg.wire_strippers = [ITEM_STATES[2]]
            item_msg.vernier_calipers = [ITEM_STATES[3]]
            item_status_pub.publish(item_msg)
        
        # Maintain update frequency
        rate.sleep()


def getStates():
    temp_states = [[0,0,0,0],[0,0,0,0]]
    arduino.write(">1i<")
    #cmd = arduino.read_until("<")
    cmd = arduino.read_until("<").strip()
    #print (cmd)
    temp_states[0][0] = int(cmd[2]) #set drawer status of 1st drawer to the 2nd char of the reply
    temp_states[1][0] = int(cmd[3]) #set item status of 1st drawer to the 3rd char of the reply
    
    arduino.write(">2i<")
    #cmd = arduino.read_until("<")
    cmd = arduino.read_until("<").strip()
    temp_states[0][1] = int(cmd[2]) #set drawer status of 2nd drawer to the 2nd char of the reply
    temp_states[1][1] = int(cmd[3]) #set item status of 2nd drawer to the 3rd char of the reply
    
    arduino.write(">3i<")
    #cmd = arduino.read_until("<")
    cmd = arduino.read_until("<").strip()
    temp_states[0][2] = int(cmd[2]) #set drawer status of 3rd drawer to the 2nd char of the reply
    temp_states[1][2] = int(cmd[3]) #set item status of 3rd drawer to the 3rd char of the reply
    
    arduino.write(">4i<")
    #cmd = arduino.read_until("<")
    cmd = arduino.read_until("<").strip()
    temp_states[0][3] = int(cmd[2]) #set drawer status of 4th drawer to the 2nd char of the reply
    temp_states[1][3] = int(cmd[3]) #set item status of 4th drawer to the 3rd char of the reply
    
    return temp_states
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass