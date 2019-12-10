import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

ACTIVE = 0x00
ARRIVED = 0x01
NOTHING = 0x02


class NavigationInterface:
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        rospy.Subscriber(
            "move_base/status", GoalStatusArray, self.status_cb)
        self.active = False
        self.status = None

    def wait_until_ready(self):
        rospy.loginfo("Connecting to move base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move base.")

    def goto_pos(self, pose):
        if not isinstance(pose, PoseStamped):
            rospy.logerr("pose must be of type PoseStamped, not type {}".format(type(pose)))
            raise TypeError

        # If we're currently moving somewhere, cancel it and begin moving to
        # the new location
        if self.active == True:
            self.client.cancel_goal()

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.active = True
        self.status = None
        self.client.send_goal(goal)
    
    def get_state(self):
        if self.active:
            if self.status == None:
                return ACTIVE
            elif len(self.status.status_list) == 0:
                return NOTHING
            status = self.status.status_list[-1].status
            if status == GoalStatus.ACTIVE:
                return ACTIVE
            elif status == GoalStatus.SUCCEEDED:
                self.active = False
                return ARRIVED
        return NOTHING

    def wait_for_completion(self, timeout_s):
        return self.client.wait_for_result(timeout=rospy.Duration(timout_s))

    def status_cb(self, status):
        self.status = status

nav_interface = NavigationInterface()
