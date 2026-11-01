#!/usr/bin/env python3
"""Bridge: livox_laser_simulation/CustomMsg -> livox_ros_driver2/CustomMsg."""
import rospy
from livox_laser_simulation.msg import CustomMsg as SimMsg
from livox_ros_driver2.msg import CustomMsg as DrvMsg


class Bridge:
    def __init__(self):
        self.pub = rospy.Publisher("/livox/lidar_r2", DrvMsg, queue_size=2)
        self.count = 0
        rospy.Subscriber("/livox/lidar", SimMsg, self.cb, queue_size=2,
                         buff_size=2 ** 24)
        rospy.loginfo("livox bridge up: /livox/lidar -> /livox/lidar_r2")

    def cb(self, m):
        out = DrvMsg()
        out.header = m.header
        out.timebase = m.timebase
        out.point_num = m.point_num
        out.lidar_id = m.lidar_id
        out.points = m.points
        self.pub.publish(out)
        self.count += 1
        if self.count % 50 == 0:
            rospy.loginfo("bridged %d msgs, last pts=%d", self.count, m.point_num)


if __name__ == "__main__":
    rospy.init_node("livox_sim_bridge")
    Bridge()
    rospy.spin()
