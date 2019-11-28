import rospy

from geometry_msgs.msg import PoseArray

class HumanTrackerInterface:
    def __init__(self):
        rospy.Subscriber("/humandetect_poses", PoseArray, self.pose_cb)

    def pose_cb(self, pose_array):
        pass

tracker_interface = HumanTrackerInterface()
