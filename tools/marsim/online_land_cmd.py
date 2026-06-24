#!/usr/bin/env python3
"""发送 AUTO.LAND 指令"""
import rospy
from mavros_msgs.srv import SetMode
rospy.init_node("land_cmd", anonymous=True)
try:
    s = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    s(custom_mode="AUTO.LAND")
    print("[land_cmd] AUTO.LAND sent")
except Exception as e:
    print(f"[land_cmd] WARNING: {e}")
