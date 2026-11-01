#!/usr/bin/env python3
"""Static TF helper (v2): only body -> downward_camera_optical_frame.

map->camera_init 那条静态 TF 会干扰 Fast-Planner 的 FSM 初始化（实测 FSM 卡在 INIT），
而项目的 mission_frame 本来就是 camera_init，所以视觉链直接用 camera_init 作为 map_frame。
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("liftrace_camera_tf")
    br = tf2_ros.StaticTransformBroadcaster()
    dz = float(rospy.get_param("~camera_offset_z", -0.16))

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "body"
    t.child_frame_id = "downward_camera_optical_frame"
    t.transform.translation.z = dz
    t.transform.rotation.y = 1.0   # 180 deg about Y
    br.sendTransform(t)
    rospy.loginfo("liftrace_camera_tf: body -> downward_camera_optical_frame (dz=%.3f)", dz)
    rospy.spin()


if __name__ == "__main__":
    main()
