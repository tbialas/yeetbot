from yeetbot_msgs.msg import YEETBotState
from yeetbot_master_controller.interfaces import state_pub, text_msg_pub
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
        speech_msg.data = "YEET"
        #speech_msg.data = "Hello, I am YEETBot3000!\n\nTo get my attention, just wave and say \"YEETBot3000\".\n\nHow can I help you today?"
        text_msg_pub.publish(speech_msg)

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
            return VerifyRequest()
        else:
            return self


class VerifyRequest(State):
    def __init__(self):
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.RECEIVING_REQUEST
        state_pub.publish(state_msg)

    def run(self):
        # TODO: check tool database
        return "VerifyRequest"

    def next(self, input_array):
        if input_array['request'] == "lend" and input_array['request_verified'] == 1:
            return LendTool()
        elif input_array['request'] == "return" and input_array['request_verified'] == 1:
            return ReturnTool()
        else:
            return self


class LendTool(State):
    def __init__(self):
        state_msg = YEETBotState()
        state_msg.current_state = YEETBotState.GIVING_TOOL
        state_pub.publish(state_msg)

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
