import rospy
from std_msgs.msg import String

from yeetbot_msgs.msg import YEETBotState, YEETBotItemStates

from yeetbot_master_controller.item_database import item_database

# Publisher interfaces
state_pub = rospy.Publisher("/yeetbot_state", YEETBotState, queue_size=1)
text_msg_pub = rospy.Publisher("/text_msg", String, queue_size=1)

# Subscriber interfaces

rospy.Subscriber("/item_state_changed", YEETBotItemStates, 
                 item_database.database_update_cb)
