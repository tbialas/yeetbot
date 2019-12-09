from math import sqrt, cos, sin, acos, atan2

import rospy

import tf

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from yeetbot_msgs.msg import YEETBotState
from yeetbot_master_controller.interfaces import publish_state_update, text_msg_pub
from yeetbot_master_controller.user_interface import user_interface
from yeetbot_master_controller.human_tracker_interface import tracker_interface
from yeetbot_master_controller.item_database import item_database
from yeetbot_master_controller.exceptions import HumanDeadError
from yeetbot_master_controller import navigation_interface
from navigation_interface import nav_interface


HOME_X = 3.45
HOME_Y = 4.15
HOME_YAW = -3.14 / 2


tf_listener = tf.TransformListener()


class State(object):
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

        self.start_time = rospy.Time.now()

    def run(self):
        return "Idle"

    def next(self, input_array):
        if input_array['low_voltage'] == 1:
            return LowVoltage()
        elif input_array['yeet_request'] == 1:
            return TravelToRequest()
        elif input_array['request'] == 'lend':
            return VerifyRequest('lend')
        elif input_array['request'] == 'return':
            return VerifyRequest('return')
        elif input_array['tool_timeout'] == 1:
            try:
                return ForceReturn()
            except NoToolsTimedOutError:
                pass
        dur = rospy.Time.now() - self.start_time
        if dur.secs > 15:
            return ReturnHome()
        return self


class Travelling(State):
    def __init__(self, goal=PoseStamped()):
        publish_state_update(YEETBotState.TRAVELLING)

        self.current_goal = goal
        self.new_goal = goal
        nav_interface.goto_pos(self.current_goal)

        self.state = navigation_interface.ACTIVE

    def goal_distance(self, target_pose):
        dx = target_pose.pose.position.x - self.current_goal.pose.position.x
        dy = target_pose.pose.position.y - self.current_goal.pose.position.y
        return sqrt(dx*dx + dy*dy)

    def run(self):
        if self.goal_distance(self.new_goal) >= 2:
            self.current_goal = self.new_goal
            nav_interface.goto_pos(self.current_goal)

        self.state = nav_interface.get_state()

        return "Travelling"

    def next(self, input_array):
        if self.state == navigation_interface.ARRIVED:
            return VerifyRequest('')
        elif self.state == navigation_interface.NOTHING:
            # TODO: Ask for help!
            return Idle()
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
        if item_database.contains_tool(tool):
            speech_msg.data = "Please take the {} from the drawer that I am opening for you.".format(tool.replace('_', ' '))
            item_database.change_drawer_state(tool, True)
            self.false_request = False
        else:
            speech_msg.data = "I'm sorry, I don't have that tool in stock. Perhaps you could return it to me?"
            self.false_request = True

        text_msg_pub.publish(speech_msg)
        
        self.tool = tool

        user_interface.tool_requested = ''
        user_interface.user_requested_borrow = False

        self.time_started = rospy.Time.now()

    def run(self):
        return "LendTool"

    def next(self, input_array):
        if input_array['tool_removed'] == 1:
            item_database.tool_taken = False
            item_database.change_drawer_state(self.tool, False)
            return Idle()
        dur = rospy.Time.now() - self.time_started
        if self.false_request:
            if dur.secs > 5:
                return Idle()
        if dur.secs > 15:
            item_database.change_drawer_state(self.tool, False)
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
        if item_database.is_tool_missing(tool):
            speech_msg.data = "Please return the {} to the drawer that I am opening for you.".format(tool.replace('_', ' '))
            item_database.change_drawer_state(tool, True)
            self.false_request = False
        else:
            speech_msg.data = "I'm not missing any of those!"
            self.false_request = True

        text_msg_pub.publish(speech_msg)

        self.tool = tool

        user_interface.tool_requested = ''
        user_interface.user_requested_borrow = False

        self.time_started = rospy.Time.now()

    def run(self):
        return "ReturnTool"

    def next(self, input_array):
        if input_array['tool_replaced'] == 1:
            item_database.tool_returned = False
            item_database.change_drawer_state(self.tool, False)
            return Idle()
        dur = rospy.Time.now() - self.time_started
        if self.false_request:
            if dur.secs > 5:
                return Idle()
        if dur.secs > 15:
            item_database.change_drawer_state(self.tool, False)
            return Idle()
        return self


class ForceReturn(Travelling):
    def __init__(self):
        pose = PoseStamped()
        pose.pose.position.x = 5.89
        pose.pose.position.y = 2.15
        pose.pose.orientation.w = 1
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        super(ForceReturn, self).__init__(goal=pose)
        publish_state_update(YEETBotState.RECEIVING_TOOL_LATE)
        self.tool = item_database.get_timed_out_tool()

    def run(self):
        super(ForceReturn, self).run()
        if self.state == navigation_interface.ARRIVED:
            speech_msg = String()
            speech_msg.data = "I am looking for my " + self.tool + ". Please return it to me."
            text_msg_pub.publish(speech_msg)

        return "ForceReturn"

    def next(self, input_array):
        if input_array['request'] == "return" and input_array['request_verified'] == 1:
            return ReturnTool()
        else:
            return self


class ReturnHome(Travelling):
    def __init__(self):
        home = PoseStamped()
        home.header.stamp = rospy.Time.now()
        home.header.frame_id = 'map'
        home.pose.position.x = HOME_X
        home.pose.position.y = HOME_Y
        home.pose.orientation.w = cos(HOME_YAW / 2)
        home.pose.orientation.z = sin(HOME_YAW / 2)
        super(ReturnHome, self).__init__(goal=home)
    
    def run(self):
        super(ReturnHome, self).run()
        return "ReturnHome"

    def next(self, input_array):
        if input_array['low_voltage'] == 1:
            return LowVoltage()
        elif self.state == navigation_interface.ARRIVED:
            return Idle()
        elif self.state == navigation_interface.NOTHING:
            return ReturnHome()
        elif input_array['yeet_request'] == 1:
            return Travelling()
        elif input_array['tool_timeout'] == 1:
            try:
                return ForceReturn()
            except NoToolsTimedOutError:
                pass
        return self


class LowVoltage(ReturnHome):
    def __init__(self):
        super(LowVoltage, self).__init__()
        speech_msg = String()
        speech_msg.data = "I am low on battery! Please help me charge myself!"
        text_msg_pub.publish(speech_msg)

    def run(self):
        super(LowVoltage, self).run()
        return "LowVoltage"

    def next(self, input_array):
        if self.state == navigation_interface.ARRIVED:
            return Idle()
        elif self.state == navigation_interface.NOTHING:
            return LowVoltage()
        else:
            return self

class TravelToRequest(Travelling):
    def __init__(self):
        (target, self.id) = self.calculate_target_pose()
        super(TravelToRequest, self).__init__(goal=target)

    def calculate_target_pose(self):
        try:
            (trans, rot) = tf_listener.lookupTransform(
                'base_link', 'map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException):
            rospy.logerr("Failed to get pose of robot in map frame...")
            pose = PoseStamped()
            pose.pose.position.x = HOME_X
            pose.pose.position.y = HOME_Y
            pose.pose.orientation.w = cos(HOME_YAW / 2)
            pose.pose.orientation.z = sin(HOME_YAW / 2)
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            return (pose, -1)

        a = tracker_interface.voice_yaw + 2*acos(rot[3])
        dx = cos(a)
        dy = sin(a)
        sx = trans[0]
        sy = trans[1]
        # We now have a ray in the map coordinate frame pointing at the
        # person who called us (in the form R = s + m*d)

        min_dist = 9e9
        target = None
        # Assume all human poses are in the map frame ( ? )
        for human in tracker_interface.humans:
            hx = human.pose.position.x
            hy = human.pose.position.y
            
            # Shortest distance between human pose and the ray is the length
            # of the ray from the human which is at right angles to the
            # ray from the robot
            # s + m * d = h + n * e
            # d dot e = 0

            (ex, ey) = (0, 0)
            if dx > 1e-9 or dx < -1e-9:
                ey = 1
                ex = dy/dx
                mag = sqrt(1 + ex*ex)
                ex /= mag
                ey /= mag
            else:
                ey = 0
                ex = 1

            # Now both vectors are normalised, calculate the distance from
            # h to the intersection point p
            m = (hx + sy*ex/ey - hy*ex/ey - sx) / (dx*(1 - dy*ex/(ey*dx)))
            dist = abs((sx + m*dx -hx) / ex)

            if dist < min_dist:
                min_dist = dist
                target = human

        if target != None:
            pose = PoseStamped()
            pose.pose.position = target.pose.position
            pose.pose.orientation.w = cos(a / 2)
            pose.pose.orientation.z = sin(a / 2)
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            return (pose, human.id)
        else:
            pose = PoseStamped()
            pose.pose.position.x = HOME_X
            pose.pose.position.y = HOME_Y
            pose.pose.orientation.w = cos(HOME_YAW / 2)
            pose.pose.orientation.z = sin(HOME_YAW / 2)
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            return (pose, -1)
            
    def run(self):
        # Get pose of human with ID that we're driving towards and update
        # self.new_goal
        try:
            human_pose = tracker_interface.get_human_with_id(self.id).pose
        except HumanDeadError:
            # If the human is dead they might have simply been reassigned
            # an ID by the tracker, so we just continue on the current
            # heading
            super(TravelToRequest, self).run()
            return "TravelToRequest"
        try:
            (trans, rot) = tf_listener.lookupTransform(
                'base_link', 'map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException):
            rospy.logerr("Failed to get pose of robot in map frame...")
            super(TravelToRequest, self).run()
            return "TravelToRequest"

        # We want to move towards the human _and face them_
        dx = human_pose.position.x - trans[0]
        dy = human_pose.position.y - trans[1]
        theta = atan2(dy, dx)
        pose = PoseStamped()
        pose.position = human_pose.position
        pose.orientation.w = cos(theta/2)
        pose.orientation.z = sin(theta/2)
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        self.new_goal = pose
        super(TravelToRequest, self).run()
        return "TravelToRequest"

    def next(self, input_array):
        return super(TravelToRequest, self).next(input_array)
