#!/usr/bin/env python3
"""Relay FAST-LIO /Odometry -> /mavros/vision_pose/pose (PX4 EKF2 external vision).

- 输入: /Odometry (camera_init -> body)
- 输出: /mavros/vision_pose/pose (frame map 或 ~target_frame)
- 解锁前把姿态固定为初始朝向（stabilize_yaw_until_armed:=true），
  避免 PX4 因 "heading estimate not stable" 拒绝解锁；解锁后转发 LIO 姿态。
"""
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


class OdomToVision:
    def __init__(self):
        self.target = rospy.get_param("~target_frame", "map")
        self.source_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/vision_pose/pose")
        self.timeout = rospy.get_param("~tf_timeout", 0.02)
        # 解锁后若立刻放开姿态，PX4 会因航向抖动自动上锁；因此锁姿态持续到离地
        self.stabilize = rospy.get_param("~stabilize_until_takeoff", True)
        self.airborne_z = rospy.get_param("~airborne_z", 0.30)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.warned = False
        self.n = 0
        self.armed = False
        self.z = 0.0

        rospy.Subscriber(self.source_topic, Odometry, self.cb, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=5)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=5)
        rospy.loginfo("odom_to_vision: %s -> %s [%s] stabilize_until_takeoff=%s (z>%.2f)",
                      self.source_topic, self.output_topic, self.target,
                      self.stabilize, self.airborne_z)

    def state_cb(self, msg):
        self.armed = msg.armed

    def pose_cb(self, msg):
        self.z = msg.pose.position.z

    def cb(self, msg):
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = msg.header.frame_id or "camera_init"
        ps.pose = msg.pose.pose

        if ps.header.frame_id != self.target:
            try:
                tr = self.buf.lookup_transform(self.target, ps.header.frame_id,
                                               rospy.Time(0), rospy.Duration(self.timeout))
                ps = tf2_geometry_msgs.do_transform_pose(ps, tr)
            except Exception as exc:  # noqa: BLE001
                if not self.warned:
                    rospy.logwarn("odom_to_vision: TF %s<-%s unavailable (%s); "
                                  "publishing identity-frame pose",
                                  self.target, ps.header.frame_id, exc)
                    self.warned = True
                ps.header.frame_id = self.target

        if self.stabilize and (not self.armed or self.z < self.airborne_z):
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

        self.pub.publish(ps)
        self.n += 1
        if self.n % 200 == 0:
            rospy.loginfo("odom_to_vision: forwarded %d poses (armed=%s)", self.n, self.armed)


if __name__ == "__main__":
    rospy.init_node("odom_to_vision_pose")
    OdomToVision()
    rospy.spin()
