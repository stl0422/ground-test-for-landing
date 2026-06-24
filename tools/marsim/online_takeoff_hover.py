#!/usr/bin/env python3
"""
online_takeoff_hover.py
无人机起飞到指定高度并保持悬停，用于 batch_eval_online_10trials.sh
扫描积累 FAST-LIO 地图期间维持 offboard setpoint。
"""
import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

def main():
    hover_z    = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
    hold_secs  = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

    rospy.init_node("online_takeoff_hover", anonymous=True)
    pub       = rospy.Publisher("/mavros/setpoint_position/local",
                                PoseStamped, queue_size=10)
    arm_srv   = rospy.ServiceProxy("/mavros/cmd/arming",  CommandBool)
    mode_srv  = rospy.ServiceProxy("/mavros/set_mode",    SetMode)
    rate      = rospy.Rate(20)

    sp = PoseStamped()
    sp.header.frame_id = "map"
    sp.pose.position.x = 0.0
    sp.pose.position.y = 0.0
    sp.pose.position.z = hover_z

    print(f"[hover] pre-sending setpoints for 5s (z={hover_z}m)...")
    for _ in range(100):
        sp.header.stamp = rospy.Time.now()
        pub.publish(sp)
        rate.sleep()

    print("[hover] switching to OFFBOARD + ARM...")
    try:
        mode_srv(custom_mode="OFFBOARD")
        time.sleep(0.5)
        arm_srv(True)
    except Exception as e:
        print(f"[hover] WARNING: {e}")

    print(f"[hover] hovering for {hold_secs}s...")
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < hold_secs:
        sp.header.stamp = rospy.Time.now()
        pub.publish(sp)
        rate.sleep()

    print("[hover] done, switching to AUTO.LAND...")
    try:
        mode_srv(custom_mode="AUTO.LAND")
    except Exception as e:
        print(f"[hover] WARNING land mode: {e}")

if __name__ == "__main__":
    main()
