import rospy

from yeetbot_master_controller.exceptions import NoToolsTimedOutError

TOOL_TIMEOUT = 60*15 # 15 mins


class Tool:
    def __init__(self, state):
        self.state = state
        self.time_taken = rospy.Time.now()

    def update(self, state):
        if state != self.state:
            self.state = state
            self.time_taken = rospy.Time.now()

    def is_timed_out(self):
        if self.state = False:
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

    def wait_until_ready(self):
        while not self.is_ready():
            rospy.sleep(1)
            print "Waiting for initial item drawer state."

    def is_ready(self):
        if len(self.pliers) == 0 and len(self.screw_drivers) == 0 and len(self.wire_strippers) == 0 and len(self.vernier_calipers) == 0:
            return False
        return True

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
            for i in range(item_state_msg.pliers):
                self.pliers[i].update(item_state_msg.pliers[i])
            for i in range(item_state_msg.screw_drivers):
                self.screw_drivers[i].update(
                    item_state_msg.screw_drivers[i])
            for i in range(item_state_msg.wire_strippers):
                self.wire_strippers[i].update(
                    item_state_msg.wire_strippers[i])
            for i in range(item_state_msg.vernier_calipers):
                self.vernier_calipers[i].update(
                    item_state_msg.vernier_calipers[i])

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

item_database = ItemDatabase()
