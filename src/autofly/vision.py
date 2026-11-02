"""视觉接口封装：把 /uav_vision 的几路话题收成一个对象。"""
from __future__ import annotations

import rospy
from std_msgs.msg import Bool, String
from uav_vision.msg import DropReady, TargetCandidateArray


class Vision:
    """订阅 targets / drop_ready / servo_complete，并提供若干发布者。"""

    def __init__(self, align_mode_topic: str = "/uav_vision/align_mode"):
        self.ready = False
        self.reason = "-"
        self.ntgt = 0
        self.servo_done = False
        self._raw_targets = []

        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, self._targets_cb)
        rospy.Subscriber("/uav_vision/drop_ready", DropReady, self._ready_cb)
        rospy.Subscriber("/detect/servo_complete", Bool, self._servo_cb)

        self.mode_pub = rospy.Publisher(align_mode_topic, String, queue_size=2, latch=True)
        self.servo_pub = rospy.Publisher("/detect/servo_status", Bool, queue_size=2)
        self.land_pub = rospy.Publisher("/detect/landing_control", Bool, queue_size=2, latch=True)
        self.ctl_pub = rospy.Publisher("/detect/control", Bool, queue_size=2, latch=True)

    # ---------- 回调 ----------
    def _targets_cb(self, msg: TargetCandidateArray) -> None:
        self.ntgt = len(msg.targets)
        self._raw_targets = list(msg.targets)

    def _ready_cb(self, msg: DropReady) -> None:
        self.ready = msg.ready
        self.reason = msg.reason

    def _servo_cb(self, msg: Bool) -> None:
        if msg.data:
            self.servo_done = True

    # ---------- 查询 ----------
    def confirmed(self, want_classes, min_state: int = 2):
        """返回 [(class, x, y, conf)]，只要已确认且地图有效的目标。"""
        out = []
        for t in self._raw_targets:
            if t.class_name not in want_classes:
                continue
            if getattr(t, "state", 0) < min_state:
                continue
            valid = getattr(t, "current_map_valid", getattr(t, "map_valid", False))
            if not valid:
                continue
            mp = t.map_point
            out.append((t.class_name, mp.x, mp.y, float(getattr(t, "class_confidence", 0.0))))
        return out

    # ---------- 控制 ----------
    def set_mode(self, mode: str) -> None:
        self.mode_pub.publish(String(data=mode))

    def enable_detect(self, on: bool) -> None:
        self.ctl_pub.publish(Bool(data=on))

    def enable_landing(self, on: bool) -> None:
        self.land_pub.publish(Bool(data=on))

    def release(self, times: int = 5) -> None:
        self.servo_done = False
        for _ in range(times):
            self.servo_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

    def reset_ready(self) -> None:
        self.ready = False
        self.reason = "-"
