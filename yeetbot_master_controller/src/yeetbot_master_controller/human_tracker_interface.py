import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from yeetbot_msgs.msg import YEETBotHumanPoseArray
from yeetbot_master_controller.exceptions import HumanDeadError


# Length of a human year in seconds
HUMAN_YEAR = 2


class Human:
    def __init__(self, unique_id, pose=Pose()):
        self.id = unique_id
        self.pose = pose
        self.birthday = rospy.Time.now()

    def celebrate_birthday(self, pose):
        self.pose = pose
        self.birthday = rospy.Time.now()


class HumanTrackerInterface:
    def __init__(self):
        rospy.Subscriber(
            "/humandetect_poses", YEETBotHumanPoseArray, self.pose_cb)
        rospy.Subscriber(
            "/yeetbot_voice_direction", Float64, self.voice_dir_cb)
        self.humans = []
        self.voice_yaw = 0
        self.voice_update_time = rospy.Time(0)

    def get_human_with_id(self, id_):
        try:
            return next(human for human in self.humans if human.id == id_)
        except StopIteration:
            raise HumanDeadError

    def bringout_your_dead(self):
        for human in self.humans():
            print human.id
            dur = rospy.Time.now() - human.birthday
            if dur > HUMAN_YEAR:
                self.humans.remove(human)

    def pose_cb(self, pose_array):
        if pose_array.header.frame_id != 'map':
            rospy.logerr('Pose array message is not in the map frame!')
        for it in range(len(pose_array.ids)):
            matching_human = None
            for human in self.humans:
                if human.id == pose_array.ids[it]:
                    matching_human = human
                    break
            if matching_human != None:
                matching_human.celebrate_birthday(
                    pose_array.human_poses[it])
            else:
                self.humans.append(
                    Human(pose_array.ids[it], 
                          pose=pose_array.human_poses[it]))

        # Kill and remove humans who have not had a birthday for a long time
        self.bringout_your_dead()

    def voice_dir_cb(self, voice_dir):
        self.voice_yaw = voice_dir.data
        self.voice_update_time = rospy.Time.now()

tracker_interface = HumanTrackerInterface()
