#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from yeetbot_master_controller.state_machine import StateMachine


def main():
    rospy.init_node('master_controller')
    rate = rospy.Rate(0.2)

    # Init the state machine
    machine = StateMachine()

    input_array = { 'yeet_request':1,
                    'tool_timeout':0,
                    'request':"lend",
                    'request_verified':1,
                    'tool_removed':1,
                    'tool_replaced':0,
                    'target_set':0,
                    'target_reached':1}

    while not rospy.is_shutdown():
        machine.run_next(input_array)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
