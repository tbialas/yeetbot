#!/usr/bin/env python

import rospy

rospy.init_node('master_controller')

from yeetbot_master_controller.item_database import item_database
from yeetbot_master_controller.state_machine import StateMachine
from yeetbot_master_controller.exceptions import NoToolsTimedOutError
from yeetbot_master_controller.user_interface import user_interface


def main():
    rospy.sleep(1)
    rate = rospy.Rate(2)

    # Init the state machine
    machine = StateMachine()

    #item_database.wait_until_ready()

    print "Item Database ready"

    input_array = { 'yeet_request':0,
                    'tool_timeout':0,
                    'request':'',
                    'request_verified':0,
                    'tool_removed':0,
                    'tool_replaced':0,
                    'target_set':0,
                    'target_reached':0}

    while not rospy.is_shutdown():
        # Update the state machine
        machine.run_next(input_array)

        # Check if any tools have timed out
        try:
            item_database.get_timed_out_tool()
            input_array['tool_timeout'] = 1
        except NoToolsTimedOutError:
            input_array['tool_timeout'] = 0

        # Check if the user has requested a state change
        if user_interface.user_requested_borrow:
            input_array['request'] = 'lend'
            user_interface.reset()
        elif user_interface.user_requested_return:
            input_array['request'] = 'return'
            user_interface.reset()
        else:
            input_array['request'] = ''

        if user_interface.tool_requested != '':
            input_array['request_verified'] = 1
            user_interface.reset()
        else:
            input_array['request_verified'] = 0

        # Sleep
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
