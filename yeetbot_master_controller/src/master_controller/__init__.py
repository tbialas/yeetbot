#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class State:
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"

class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    def run_next(self, inputs):
        self.currentState = self.currentState.next(inputs)
        
    def runAll(self, inputs):
        for i in inputs:
            print(i)
            self.currentState = self.currentState.next(i)
            self.currentState.run()

#states and transitions
class Idle(State):
    def run(self):
        #to be replaced with something useful   
        return "Idle state"
    def next(self, input_array):
        if input_array['yeet_request'] == 1:
            return MasterController.setlocation
        elif input_array['tool_timeout'] == 1:
            return MasterController.forcereturn
        else:
            return MasterController.idle


class Setlocation(State):
    #could be merged with goto location
    def run(self):
        return "Finding Human Target"
    def next(self, input_array):
        if input_array['target_set'] == 1:
            return MasterController.gotolocation
        else:
            return MasterController.setlocation

class Gotolocation(State):
    def run(self):
        return "Going to Target"
    def next(self, input_array):
        if input_array['target_reached'] == 1:
            return MasterController.verifyrequest
        else:
            return MasterController.gotolocation

class Verifyrequest(State):
    #check tool database
    def run(self):
        return "Check Request"
    def next(self, input_array):
        if input_array['request'] == "lend" and input_array['request_verified'] == 1:
            return MasterController.lendtool
        elif input_array['request'] == "return" and input_array['request_verified'] == 1:
            return MasterController.returntool
        else:
            return MasterController.verifyrequest

class Lendtool(State):
    def run(self):
        #start tool timer
        #amend tool database
        return "Tool Lent"
    def next(self, input_array):
        if input_array['tool_removed'] == 1:
            return MasterController.idle
        else:
            return MasterController.lendtool

class Returntool(State):
    #tool has been returned
    def run(self):
        #update database
        return "Tool Returned"
    def next(self, input_array):
        if input_array['tool_replaced'] == 1:
            return MasterController.idle
        else:
            return MasterController.returntool


class Forcereturn(State):
    #you know a tool is out and needs to be returned
    #op1 - shout that you want it back
    #op2 - go to the person who has it (human tracking dependent)
    def run(self):
        return "give me the tool back"
    def next(self, input_array):
        if input_array['request'] == "return" and input_array['request_verified'] == 1:
            return MasterController.returntool
        else:
            return MasterController.forcereturn



#statemachine to be called
class MasterController(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, MasterController.idle)

MasterController.idle           = Idle()
MasterController.setlocation    = Setlocation()
MasterController.gotolocation   = Gotolocation()
MasterController.verifyrequest  = Verifyrequest()
MasterController.lendtool       = Lendtool()
MasterController.returntool     = Returntool()
MasterController.forcereturn    = Forcereturn()


#ros function to run the statemachine
def run_controller():
    state_pub = rospy.Publisher('mc_state', String, queue_size=1)
    rospy.init_node('master_controller', anonymous=True)
    rate = rospy.Rate(1)

    #init the state machine
    mc_sm = MasterController()

    input_array = { 'yeet_request':0,
                    'tool_timeout':0,
                    'request':"none",
                    'request_verified':0,
                    'tool_removed':0,
                    'tool_replaced':0,
                    'target_set':0,
                    'target_reached':0}

    while not rospy.is_shutdown():
                
        pub_str = mc_sm.currentState.run() + str(rospy.get_time())
        #pub_str = "hello %s" % rospy.get_time()
        state_pub.publish(pub_str)
        mc_sm.run_next(input_array)
        rate.sleep()


if __name__ == '__main__':
    try:
        run_controller()
    except rospy.ROSInterruptException:
        pass
