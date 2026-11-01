#!/usr/bin/env python3
"""Simulation-only mock servo: ack /detect/servo_status with /detect/servo_complete."""
import rospy
from std_msgs.msg import Bool


class MockServo:
    def __init__(self):
        self.pub = rospy.Publisher("/detect/servo_complete", Bool, queue_size=5)
        rospy.Subscriber("/detect/servo_status", Bool, self.cb, queue_size=5)
        self.n = 0
        rospy.loginfo("mock_servo: listening on /detect/servo_status")

    def cb(self, msg):
        if not msg.data:
            return
        self.n += 1
        rospy.loginfo("mock_servo: release request #%d -> ack", self.n)
        rospy.Timer(rospy.Duration(0.5),
                    lambda _e: self.pub.publish(Bool(data=True)),
                    oneshot=True)


if __name__ == "__main__":
    rospy.init_node("mock_servo")
    MockServo()
    rospy.spin()
