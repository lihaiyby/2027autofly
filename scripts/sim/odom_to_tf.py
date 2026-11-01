#!/usr/bin/env python3
"""Broadcast TF camera_init -> body from FAST-LIO /Odometry（去重同一时间戳）。"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomToTf:
    def __init__(self):
        self.parent = rospy.get_param("~parent_frame", "camera_init")
        self.child = rospy.get_param("~child_frame", "body")
        self.topic = rospy.get_param("~odom_topic", "/Odometry")
        self.br = tf2_ros.TransformBroadcaster()
        self.last_stamp = None
        self.skipped = 0
        rospy.Subscriber(self.topic, Odometry, self.cb, queue_size=10)
        rospy.loginfo("odom_to_tf: %s -> %s from %s (dedup by stamp)",
                      self.parent, self.child, self.topic)

    def cb(self, msg):
        # FAST-LIO 偶尔同一时间戳发两帧，重复 TF 会刷 TF_REPEATED_DATA 警告
        stamp = msg.header.stamp.to_nsec()
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.skipped += 1
            if self.skipped % 200 == 1:
                rospy.logdebug("odom_to_tf: skip duplicate stamp (%d skipped)", self.skipped)
            return
        self.last_stamp = stamp

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    OdomToTf()
    rospy.spin()
