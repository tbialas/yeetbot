import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

ACTIVE = 0x00
ARRIVED = 0x01
NOTHING = 0x02


class NavigationInterface:
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        print "Waiting for move_base..."
        if not self.client.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Timed out waiting for Move Base to start")

    def goto_pos(self, pose):
        if not isinstance(pose, PoseStamped):
            print "pose must be of type PoseStamped, not type {}".format(type(pose))
            raise TypeError

        # If we're currently moving somewhere, cancel it and begin moving to
        # the new location
        if self.client.get_state() == GoalStatus.ACTIVE:
            self.client.cancel_goal()

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.client.send_goal(goal)
    
    def get_state(self):
        state = self.client.get_state()
        if state == GoalStatus.ACTIVE:
            return ACTIVE
        elif state == GoalStatus.SUCCEEDED:
            return ARRIVED
        return NOTHING

    def wait_for_completion(self, timeout_s):
        return self.client.wait_for_result(timeout=rospy.Duration(timout_s))

nav_interface = NavigationInterface()
