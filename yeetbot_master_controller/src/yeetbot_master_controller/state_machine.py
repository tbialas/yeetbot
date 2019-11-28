import rospy
from yeetbot_master_controller.states import *


class StateMachine:
    def __init__(self):
        self.current_state = Idle()
        self.current_state.run()

    def __del__(self):
        # Ensure that we finish in the idle state
        self.current_state = Idle()
        self.current_state.run()
    
    def run_next(self, input_array):
        self.current_state = self.current_state.next(input_array)
        rospy.logdebug(self.current_state.run())

    def run_all(self, input_array):
        for i in input_array:
            print i
            self.run_next(i)
