import rospy
from std_msgs.msg import String

from yeetbot_msgs.msg import *
from p2os_msgs.msg import BatteryState

# Publisher interfaces
state_pub = rospy.Publisher("/yeetbot_state", YEETBotState, queue_size=1)
text_msg_pub = rospy.Publisher("/text_msg", String, queue_size=1)

def publish_state_update(state):
    state_msg = YEETBotState()
    state_msg.current_state = state
    state_pub.publish(state_msg)
    # Sleep to ensure this is published before any other messages are 
    # published, and thus the other nodes are in the correct state
    # when information arrives
    rospy.sleep(0.1)

battery_voltage = -1

def bat_volt_cb(voltage):
    global battery_voltage
    battery_voltage = voltage.voltage

rospy.Subscriber("/battery_state", BatteryState, bat_volt_cb)
