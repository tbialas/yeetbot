# This class attempts to run the general speech dialogue
# It has every option that can happen with the GUI

# The speech processing node needs to try and match the given response 
# options when communicating with the user.

import rospy
from std_msgs.msg import String

from yeetbot_msgs.msg import YEETBotUserChoices, YEETBotUserResponse
from yeetbot_master_controller.interfaces import text_msg_pub

BORROW = "I would like to borrow a tool."
RETURN = "I would like to return a tool."

BORROW_PLIERS = "I would like to borrow a pair of pliers."
BORROW_SCREW_DRIVER = "I would like to borrow a screw driver."
BORROW_WIRE_STRIPPERS = "I would like to borrow a pair of wire strippers."
BORROW_VERNIER_CALIPERS = "I would like to borrow a vernier caliper set."

RETURN_PLIERS = "I would like to return a pair of pliers."
RETURN_SCREW_DRIVER = "I would like to return a screw driver."
RETURN_WIRE_STRIPPERS = "I would like to return a pair of wire strippers."
RETURN_VERNIER_CALIPERS = "I would like to return a vernier caliper set."

class UserInterface:
    def __init__(self):
        self.choices_pub = rospy.Publisher(
            "/user_choices", YEETBotUserChoices, queue_size=1)
        self.reset()

        rospy.Subscriber(
            "/user_response", YEETBotUserResponse, self.response_cb)

        self.tool_requested = ''
        self.user_requested_borrow = False
        self.user_requested_return = False

        self.choice_id = 0

    def reset(self):
        self.choices = YEETBotUserChoices()
        self.choices.multi_choice = False
        self.response = None
        self.choices_pub.publish(self.choices)

    def update_choices(self, choice_array):
        self.reset()
        for choice in choice_array:
            self.choices.user_options.append(choice)
        self.choice_id += 1
        self.choices.id = self.choice_id
        self.choices_pub.publish(self.choices)

    def idle_screen_choices(self):
        self.update_choices([BORROW, RETURN])

    def borrow_choices(self):
        self.update_choices(
            [BORROW_PLIERS, BORROW_SCREW_DRIVER, BORROW_WIRE_STRIPPERS, 
             BORROW_VERNIER_CALIPERS])

    def return_choices(self):
        self.update_choices(
            [RETURN_PLIERS, RETURN_SCREW_DRIVER, RETURN_WIRE_STRIPPERS, 
             RETURN_VERNIER_CALIPERS])

    def response_cb(self, response_msg):
        if response_msg.id != self.choice_id:
            print "User response ID did not match expected ID"
            return
        if response_msg.invalid_choice:
            self.reset()
            msg = String()
            msg.data = "I'm sorry, I didn't understand that. Could you try again?"
            text_msg_pub.publish(msg)
            self.choices_pub.publish(self.choices)
            return
        try:
            resp = self.choices.user_options[response_msg.choice]
        except IndexError:
            print "IndexError"
            return

        if resp == BORROW:
            self.user_requested_borrow = True
        elif resp == RETURN:
            self.user_requested_return = True
        elif resp == BORROW_PLIERS:
            self.tool_requested = 'pliers'
        elif resp == BORROW_SCREW_DRIVER:
            self.tool_requested = 'screw_driver'
        elif resp == BORROW_WIRE_STRIPPERS:
            self.tool_requested = 'wire_strippers'
        elif resp == BORROW_VERNIER_CALIPERS:
            self.tool_requested = 'vernier_calipers'
        elif resp == RETURN_PLIERS:
            self.tool_requested = 'pliers'
        elif resp == RETURN_SCREW_DRIVER:
            self.tool_requested = 'screw_driver'
        elif resp == RETURN_WIRE_STRIPPERS:
            self.tool_requested = 'wire_strippers'
        elif resp == RETURN_VERNIER_CALIPERS:
            self.tool_requested = 'vernier_calipers'


user_interface = UserInterface()
