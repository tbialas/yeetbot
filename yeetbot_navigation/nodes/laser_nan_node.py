#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
import std_msgs
import math

def callback(rcv):

    range_list = list(rcv.ranges)

    for (i,range_val) in enumerate(range_list):
        if math.isnan(float(range_val)):
            range_list[i] = rcv.range_max

    range_tuple = tuple(range_list)
    rcv.ranges = range_tuple
    pub.publish(rcv)

def joint_pub():
    sub = rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('no_nan_scan', LaserScan, queue_size=1)
        rospy.init_node('scan_nan_node', anonymous=False)
        joint_pub()
    except rospy.ROSInterruptException:
        pass
