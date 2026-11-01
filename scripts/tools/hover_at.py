#!/usr/bin/env python3
"""Arm, take off and hover at a given position (for vision debugging)."""
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode


def wp(x, y, z):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.w = 1.0
    return p


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    z = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5

    rospy.init_node("hover_at")
    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=20)
    rospy.wait_for_service("/mavros/cmd/arming", timeout=30)
    rospy.wait_for_service("/mavros/set_mode", timeout=30)
    arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    setmode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    rate = rospy.Rate(20)

    for _ in range(40):
        pub.publish(wp(0, 0, z))
        rate.sleep()

    for _ in range(10):
        if setmode(custom_mode="OFFBOARD").mode_sent:
            print("[hover] OFFBOARD ok", flush=True)
            break
        rate.sleep()
    for _ in range(10):
        if arm(True).success:
            print("[hover] ARMED", flush=True)
            break
        rate.sleep()

    print(f"[hover] -> ({x:.2f}, {y:.2f}, {z:.2f})", flush=True)
    while not rospy.is_shutdown():
        pub.publish(wp(x, y, z))
        rate.sleep()


if __name__ == "__main__":
    main()
