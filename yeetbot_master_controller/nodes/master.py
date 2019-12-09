#!/usr/bin/env python

import rospy

rospy.init_node('master_controller')

from yeetbot_master_controller.item_database import item_database
from yeetbot_master_controller.state_machine import StateMachine
from yeetbot_master_controller.exceptions import NoToolsTimedOutError
from yeetbot_master_controller.user_interface import user_interface
from yeetbot_master_controller.interfaces import battery_voltage
from yeetbot_master_controller.human_tracker_interface import tracker_interface
from yeetbot_master_controller.navigation_interface import nav_interface


BATTERY_LOW_VOLTAGE = 11.3
YEET_REQUEST_TIMEOUT = 3.5


def main():
    rospy.sleep(1)
    rate = rospy.Rate(2)

    # Init the state machine
    machine = StateMachine()

    item_database.wait_until_ready()

    nav_interface.wait_until_ready()

    print "Item Database ready"

    input_array = { 'yeet_request':0, # Done
                    'tool_timeout':0, # Done
                    'request':'', # Done
                    'request_verified':0, # Done
                    'tool_removed':0, # Done
                    'tool_replaced':0, # Done
                    'low_voltage':0 } # Done

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
        elif user_interface.user_requested_return:
            input_array['request'] = 'return'
        else:
            input_array['request'] = ''

        if user_interface.tool_requested != '':
            input_array['request_verified'] = 1
        else:
            input_array['request_verified'] = 0

        # Check if any tools have been taken or removed
        if item_database.tool_taken:
            input_array['tool_removed'] = 1
        else:
            input_array['tool_removed'] = 0
        if item_database.tool_returned:
            input_array['tool_replaced'] = 1
        else:
            input_array['tool_replaced'] = 0

        # Check for low voltage from batteries
        if battery_voltage > 0 and battery_voltage < BATTERY_LOW_VOLTAGE:
            input_array['low_voltage'] = 1
        else:
            input_array['low_voltage'] = 0

        # Check for yeet requests
        dur = rospy.Time.now() - tracker_interface.voice_update_time
        rospy.loginfo(dur)
        if dur.secs + dur.nsecs / 1e9 < YEET_REQUEST_TIMEOUT:
            input_array['yeet_request'] = 1
        else:
            input_array['yeet_request'] = 0

        # Sleep
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
