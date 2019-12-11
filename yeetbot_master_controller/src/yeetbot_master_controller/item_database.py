import rospy

from yeetbot_master_controller.exceptions import NoToolsTimedOutError
from yeetbot_msgs.msg import YEETBotDrawerStates, YEETBotItemStates

TOOL_TIMEOUT = 60*15 # 15 mins


class Tool:
    def __init__(self, state):
        self.state = state
        self.time_taken = rospy.Time.now()

    def update(self, state):
        if state != self.state:
            self.state = state
            self.time_taken = rospy.Time.now()
            return True
        return False

    def is_timed_out(self):
        if self.state == False:
            dur = rospy.Time.now() - self.time_taken
            if dur.secs >= TOOL_TIMEOUT:
                return True
        return False


class ItemDatabase:
    def __init__(self):
        self.pliers = []
        self.screw_drivers = []
        self.wire_strippers = []
        self.vernier_calipers = []

        self.tool_returned = False
        self.tool_taken = False

        self.drawer_states = YEETBotDrawerStates()

        rospy.Subscriber("/item_state_changed", YEETBotItemStates,
                         self.database_update_cb)
        rospy.Subscriber("/drawer_state_changed", YEETBotDrawerStates,
                         self.drawer_states_cb)

        self.drawer_state_pub = rospy.Publisher("/drawer_state_request",
            YEETBotDrawerStates, queue_size=1)

    def is_tool_missing(self, tool):
        if tool == 'pliers':
            for grabby in self.pliers:
                if grabby.state == False:
                    return True
            return False
        elif tool == 'screw_driver':
            for twisty in self.screw_drivers:
                if twisty.state == False:
                    return True
            return False
        elif tool == 'wire_strippers':
            for cutty in self.wire_strippers:
                if cutty.state == False:
                    return True
            return False
        elif tool == 'vernier_calipers':
            for measure in self.vernier_calipers:
                if measure.state == False:
                    return True
            return False
        else:
            print "Error! {} is not a known tool!".format(tool)

    def contains_tool(self, tool):
        if tool == 'pliers':
            for grabby in self.pliers:
                if grabby.state:
                    return True
            return False
        elif tool == 'screw_driver':
            for twisty in self.screw_drivers:
                if twisty.state:
                    return True
            return False
        elif tool == 'wire_strippers':
            for cutty in self.wire_strippers:
                if cutty.state:
                    return True
            return False
        elif tool == 'vernier_calipers':
            for measure in self.vernier_calipers:
                if measure.state:
                    return True
            return False
        else:
            print "Error! {} is not a known tool!".format(tool)

    def change_drawer_state(self, drawer, state):
        if not isinstance(state, bool):
            print "State must be a boolean! True is open."
            raise TypeError
        msg = self.drawer_states
        if drawer == 'pliers':
            msg.plier_drawer = state
        elif drawer == 'screw_driver':
            msg.screw_driver_drawer = state
        elif drawer == 'wire_strippers':
            msg.wire_stripper_drawer = state
        elif drawer == 'vernier_calipers':
            msg.vernier_caliper_drawer = state
        else:
            print "{} is an unknown drawer!".format(drawer)
            raise NotImplementedError
        self.drawer_state_pub.publish(msg)

    def wait_until_ready(self):
        while not self.is_ready() and not rospy.is_shutdown():
            rospy.sleep(1)
            print "Waiting for initial item drawer state."

    def is_ready(self):
        if len(self.pliers) == 0 and len(self.screw_drivers) == 0 and len(self.wire_strippers) == 0 and len(self.vernier_calipers) == 0:
            return False
        return True

    def drawer_states_cb(self, drawer_state_msg):
        self.drawer_states = drawer_state_msg

    def database_update_cb(self, item_state_msg):
        if not self.is_ready():
            for grabby in item_state_msg.pliers:
                self.pliers.append(Tool(grabby))
            for twisty in item_state_msg.screw_drivers:
                self.screw_drivers.append(Tool(twisty))
            for cutty in item_state_msg.wire_strippers:
                self.wire_strippers.append(Tool(cutty))
            for measure in item_state_msg.vernier_calipers:
                self.vernier_calipers.append(Tool(measure))
        else:
            for i in range(len(item_state_msg.pliers)):
                if self.pliers[i].update(item_state_msg.pliers[i]):
                    if self.pliers[i].state:
                        self.tool_returned = True
                    else:
                        self.tool_taken = True
            for i in range(len(item_state_msg.screw_drivers)):
                if self.screw_drivers[i].update(
                        item_state_msg.screw_drivers[i]):
                    if self.screw_drivers[i].state:
                        self.tool_returned = True
                    else:
                        self.tool_taken = True
            for i in range(len(item_state_msg.wire_strippers)):
                if self.wire_strippers[i].update(
                        item_state_msg.wire_strippers[i]):
                    if self.wire_strippers[i].state:
                        self.tool_returned = True
                    else:
                        self.tool_taken = True
            for i in range(len(item_state_msg.vernier_calipers)):
                if self.vernier_calipers[i].update(
                        item_state_msg.vernier_calipers[i]):
                    if self.vernier_calipers[i].state:
                        self.tool_returned = True
                    else:
                        self.tool_taken = True

    def get_timed_out_tool(self):
        for grabby in self.pliers:
            if grabby.is_timed_out():
                return grabby
        for twisty in self.screw_drivers:
            if twisty.is_timed_out():
                return twisty
        for cutty in self.wire_strippers:
            if cutty.is_timed_out():
                return cutty
        for measure in self.vernier_calipers:
            if measure.is_timed_out():
                return measure
        raise NoToolsTimedOutError

    def get_timed_out_tool_named(self):
        for grabby in self.pliers:
            if grabby.is_timed_out():
                return "pliers"
        for twisty in self.screw_drivers:
            if twisty.is_timed_out():
                return "screw driver"
        for cutty in self.wire_strippers:
            if cutty.is_timed_out():
                return "wire strippers"
        for measure in self.vernier_calipers:
            if measure.is_timed_out():
                return "vernier calipers"
        raise NoToolsTimedOutError


item_database = ItemDatabase()
