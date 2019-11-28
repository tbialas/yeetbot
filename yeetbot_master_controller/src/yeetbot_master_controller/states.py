import rospy

from yeetbot_msgs.msg import YEETBotState
from yeetbot_master_controller.interfaces import publish_state_update, text_msg_pub
from yeetbot_master_controller.user_interface import user_interface
from std_msgs.msg import String


class State:
    def run(self):
        raise NotImplementedError

    def next(self, input_array):
        raise NotImplementedError


class Idle(State):
    def __init__(self):
        publish_state_update(YEETBotState.IDLE)

        speech_msg = String()
        speech_msg.data = "Hello, I am YEETBot3000!\n\nTo get my attention, just wave and say \"YEETBot3000\".\n\nHow can I help you today?"
        text_msg_pub.publish(speech_msg)

        user_interface.idle_screen_choices()

    def run(self):
        return "Idle"

    def next(self, input_array):
        if input_array['yeet_request'] == 1:
            return Travelling()
        elif input_array['tool_timeout'] == 1:
            return ForceReturn()
        elif input_array['request'] == 'lend':
            return VerifyRequest('lend')
        elif input_array['request'] == 'return':
            return VerifyRequest('return')
        else:
            return self


class Travelling(State):
    def __init__(self):
        publish_state_update(YEETBotState.TRAVELLING)

    def run(self):
        # TODO: Interface with the navigation stuff and attempt to move to 
        # the required location
        return "Travelling"

    def next(self, input_array):
        if input_array['target_reached'] == 1:
            return VerifyRequest('')
        else:
            return self


class VerifyRequest(State):
    def __init__(self, request_type):
        publish_state_update(YEETBotState.RECEIVING_REQUEST)

        self.request_type = request_type

        speech_msg = String()
        if request_type == 'lend':
            speech_msg.data = "Which tool would you like to borrow?"
            user_interface.borrow_choices()
        elif request_type == 'return':
            speech_msg.data = "Which tool would you like to return?"
            user_interface.return_choices()
        else:
            speech_msg.data = "How may I help you?"
            user_interface.idle_screen_choices()
        text_msg_pub.publish(speech_msg)

    def run(self):
        return "VerifyRequest"

    def next(self, input_array):
        if input_array['request_verified'] == 1:
            if self.request_type == 'lend':
                # Reset the flag
                user_interface.user_requested_borrow = False
                return LendTool()
            elif self.request_type == 'return':
                # Reset the flag
                user_interface.user_requested_return = False
                return ReturnTool()
            else:
                print "Request is verified but we don't know which type of request we are? {}".format(self.request_type)
                raise NotImplementedError

        if self.request_type != 'lend' and self.request_type != 'return':
            if input_array['request'] == 'lend':
                self.request_type = 'lend'
                speech_msg.data = "Which tool would you like to borrow?"
                user_interface.borrow_choices()
            elif input_array['request'] == 'return':
                self.request_type = 'return'
                speech_msg.data = "Which tool would you like to return?"
                user_interface.return_choices()
        return self


class LendTool(State):
    def __init__(self):
        publish_state_update(YEETBotState.GIVING_TOOL)

        tool = user_interface.tool_requested
        if(tool != 'pliers' and tool != 'screw_driver'
           and tool != 'wire_strippers' and tool != 'vernier_calipers'):
            raise RuntimeError("Unknown tool {}".format(tool))
        
        speech_msg = String()
        speech_msg.data = "Please take the {} from the drawer that I am opening for you.".format(tool.replace('_', ' '))
        text_msg_pub.publish(speech_msg)

        user_interface.tool_requested = ''
        user_interface.user_requested_borrow = False

        self.time_started = rospy.Time.now()

    def run(self):
        return "LendTool"

    def next(self, input_array):
        if input_array['tool_removed'] == 1:
            return Idle()
        dur = rospy.Time.now() - self.time_started
        if dur.secs > 15:
            return Idle()
        return self


class ReturnTool(State):
    def __init__(self):
        # TODO: Check whether the tool is early or on time
        publish_state_update(YEETBotState.RECEIVING_TOOL_ON_TIME)

        tool = user_interface.tool_requested
        if(tool != 'pliers' and tool != 'screw_driver'
           and tool != 'wire_strippers' and tool != 'vernier_calipers'):
            raise RuntimeError("Unknown tool {}".format(tool))
        
        speech_msg = String()
        speech_msg.data = "Please return the {} to the drawer that I am opening for you.".format(tool.replace('_', ' '))
        text_msg_pub.publish(speech_msg)

        user_interface.tool_requested = ''
        user_interface.user_requested_borrow = False

        self.time_started = rospy.Time.now()

    def run(self):
        return "ReturnTool"

    def next(self, input_array):
        if input_array['tool_replaced'] == 1:
            return Idle()
        dur = rospy.Time.now() - self.time_started
        if dur.secs > 15:
            return Idle()
        return self


class ForceReturn(State):
    def __init__(self):
        publish_state_update(YEETBotState.RECEIVING_TOOL_LATE)

    def run(self):
        # You know a tool is out and needs to be returned
        # Op1 - shout that you want it back
        # Op2 - go to the person who has it (human tracking dependent)
        return "ForceReturn"

    def next(self, input_array):
        if input_array['request'] == "return" and input_array['request_verified'] == 1:
            return ReturnTool()
        else:
            return self
