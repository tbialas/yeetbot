#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState
import std_msgs
from math import pi
import math

def callback(rcv):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    dat_num = rcv.data
    rad_data = (float(dat_num)/180) * pi
    msg.position = [rad_data]
    msg.name.append("base_camera_joint")
    
    pub.publish(msg)
    
def joint_pub():
    pub = rospy.Publisher('kinect_state', JointState, queue_size=5)
    sub = rospy.Subscriber('set_tilt_degree', std_msgs.msg.Int16, callback)
    rospy.spin()    
        

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('kinect_state', JointState, queue_size=5)
        rospy.init_node('kinect_state_nodelet', anonymous=False)
        joint_pub()
    except rospy.ROSInterruptException:
        pass
