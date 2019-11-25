import rospy
from std_msgs.msg import String

from yeetbot_msgs.msg import YEETBotState

state_pub = rospy.Publisher("/yeetbot_state", YEETBotState, queue_size=1)
text_msg_pub = rospy.Publisher("/text_msg", String, queue_size=50)
