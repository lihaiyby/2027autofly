#!/usr/bin/env python3
"""GT 桥：用 Gazebo 真值同时喂 PX4 外部视觉与 TF。

为什么需要：PX4 原来吃 FAST-LIO 的 `/Odometry`。在官方地图（模型重、RTF 低）下
LIO 偶尔丢跟踪并停在原地，PX4 就以为自己在悬停，实际机体被控制器推着越飞越远
（实测 Gazebo 真值跑到 (−36, 246)，PX4 报 (1.06, −0.69)），相机随之对着空天出灰图。

发布：
  /mavros/vision_pose/pose   PoseStamped（map 系，30 Hz）
  TF camera_init -> body     供视觉投影使用（30 Hz）
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped

PUB_HZ = 30.0


class GtBridge:
    def __init__(self):
        self.model = rospy.get_param("~model", "iris_mid360")
        self.frame = rospy.get_param("~frame_id", "map")
        self.parent = rospy.get_param("~parent_frame", "camera_init")
        self.child = rospy.get_param("~child_frame", "body")
        self.offset_z = float(rospy.get_param("~offset_z", 0.0))

        self.pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        self.pose = None
        self.n = 0

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=2)
        rospy.Timer(rospy.Duration(1.0 / PUB_HZ), self._tick)
        rospy.loginfo("gt_bridge: model=%s frame=%s -> %s/%s @%.0fHz",
                      self.model, self.frame, self.parent, self.child, PUB_HZ)

    def _cb(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name == self.model:
                self.pose = pose
                return

    def _tick(self, _evt):
        if self.pose is None:
            return
        p = self.pose
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.frame
        ps.pose = p
        self.pub.publish(ps)

        ts = TransformStamped()
        ts.header.stamp = ps.header.stamp
        ts.header.frame_id = self.parent
        ts.child_frame_id = self.child
        ts.transform.translation.x = p.position.x
        ts.transform.translation.y = p.position.y
        ts.transform.translation.z = p.position.z + self.offset_z
        ts.transform.rotation = p.orientation
        self.br.sendTransform(ts)

        self.n += 1
        if self.n % 300 == 0:
            rospy.loginfo("gt_bridge: %d poses (%.2f, %.2f, %.2f)",
                          self.n, p.position.x, p.position.y, p.position.z)


if __name__ == "__main__":
    rospy.init_node("gt_bridge")
    GtBridge()
    rospy.spin()
