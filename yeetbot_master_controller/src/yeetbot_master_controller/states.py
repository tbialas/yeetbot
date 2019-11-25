from yeetbot_msgs.msg import YEETBotState
from yeetbot_master_controller.interfaces import state_pub, text_msg_pub
from yeetbot_master_controller.user_interface import user_interface
from std_msgs.msg import String

class State:
    def run(self):
        raise NotImplementedError

    def next(self, input_array):
        raise NotImplementedError


class Idle(State):
    def __init__(self):
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.IDLE
        state_pub.publish(state_msg)

        speech_msg = String()
        speech_msg.data = "Hello, I am YEETBot3000!\n\nTo get my attention, just wave and say \"YEETBot3000\".\n\nHow can I help you today?"
        text_msg_pub.publish(speech_msg)

        user_interface.idle_screen_choices()

    def run(self):
        # In the idle state we are just that - idle
        # We don't need to do anything other than wait for inputs to change
        # the situation
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
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.TRAVELLING
        state_pub.publish(state_msg)

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
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.RECEIVING_REQUEST
        state_pub.publish(state_msg)

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
        # TODO: check tool database
        return "VerifyRequest"

    def next(self, input_array):
        if input_array['request'] == 'lend':
            if input_array['request_verified'] == 1:
                return LendTool()
            self.request_type = 'lend'
        elif input_array['request'] == 'return':
            if input_array['request_verified'] == 1:
                return ReturnTool()
            self.request_type = 'return'
        else:
            return self


class LendTool(State):
    def __init__(self):
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.GIVING_TOOL
        state_pub.publish(state_msg)

        if user_interface.tool_requested == 'pliers':
            print "requested pliers"
        elif user_interface.tool_requested == 'screw_driver':
            print "requested screw_driver"
        elif user_interface.tool_requested == 'wire_strippers':
            print "requested wire strippers"
        elif user_interface.tool_requested == 'vernier_calipers':
            print "requested vernier calipers"
        else:
            raise RuntimeError("Unknown tool {}".format(user_interface.tool_requested))
        user_interface.tool_requested = ''
        user_interface.user_requested_borrow = False

    def run(self):
        # Start tool timer
        # Amend tool database
        return "LendTool"

    def next(self, input_array):
        if input_array['tool_removed'] == 1:
            return Idle()
        else:
            return self


class ReturnTool(State):
    def __init__(self):
        state_msg = YEETBotState()
        # TODO: Check whether the tool is early or on time
        state_msg.current_state = YEETBotState.RECEIVING_TOOL_ON_TIME
        state_pub.publish(state_msg)

        if user_interface.tool_requested == 'pliers':
            print "requested pliers"
        elif user_interface.tool_requested == 'screw_driver':
            print "requested screw_driver"
        elif user_interface.tool_requested == 'wire_strippers':
            print "requested wire strippers"
        elif user_interface.tool_requested == 'vernier_calipers':
            print "requested vernier calipers"
        else:
            raise RuntimeError("Unknown tool {}".format(user_interface.tool_requested))
        user_interface.tool_requested = ''
        user_interface.user_requested_return = False

    def run(self):
        # Tool has been returned
        # Update database
        return "ReturnTool"

    def next(self, input_array):
        if input_array['tool_replaced'] == 1:
            return Idle()
        else:
            return self


class ForceReturn(State):
    def __init__(self):
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.RECEIVING_TOOL_LATE
        state_pub.publish(state_msg)

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
