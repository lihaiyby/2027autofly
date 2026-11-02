"""视觉接口封装：把 /uav_vision 的几路话题收成一个对象。"""
from __future__ import annotations

import math

import rospy
from std_msgs.msg import Bool, String
from uav_vision.msg import DropReady, TargetCandidateArray, TargetDetectionArray


class Vision:
    """订阅 targets / drop_ready / servo_complete，并提供若干发布者。"""

    def __init__(self, align_mode_topic: str = "/uav_vision/align_mode",
                 on_confirmed=None, on_detection=None):
        self.on_confirmed = on_confirmed      # 记忆里的已确认目标
        self.on_detection = on_detection      # 逐帧投影（推荐）
        self.ready = False
        self.reason = "-"
        self.ntgt = 0
        self.servo_done = False
        self._raw_targets = []
        self._want_classes = ()
        self.frame_targets = {}      # class -> (x, y) 最新逐帧投影
        self.center_gate_px = 150.0  # 只接受偏移小于此值的检测
        self.image_cx = 320.0        # 640x480 相机中心
        self.image_cy = 240.0

        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, self._targets_cb)
        # 逐帧投影：比记忆的融合点可靠得多（实测标准差 2~3 cm）
        rospy.Subscriber("/uav_vision/detections_mapped", TargetDetectionArray,
                         self._mapped_cb)
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
        # 关键：每帧都累积，避免"采样最后一条消息"时正好为空
        if self.on_confirmed is not None:
            try:
                items = self.confirmed(self._want_classes)
                if items:
                    self.on_confirmed(items)
            except Exception:      # noqa: BLE001  回调不应影响订阅
                pass

    def _mapped_cb(self, msg: TargetDetectionArray) -> None:
        if self.on_detection is None:
            return
        best = {}
        for d in msg.detections:
            if not getattr(d, "map_valid", False):
                continue
            cls = d.class_name
            if self._want_classes and cls not in self._want_classes:
                continue
            conf = float(getattr(d, "geometry_confidence", 0.0))
            # 中心偏移门限：斜视时投影误差会放大
            cen = getattr(d, "center", None)
            if cen is not None:
                if math.hypot(cen.x - self.image_cx, cen.y - self.image_cy) > self.center_gate_px:
                    continue
            if cls not in best or conf > best[cls][2]:
                best[cls] = (cls, d.map_point.x, d.map_point.y, conf)
        if best:
            for cls, x, y, _c in best.values():
                self.frame_targets[cls] = (x, y)
            try:
                self.on_detection(list(best.values()))
            except Exception:      # noqa: BLE001
                pass

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

    def latest_frame(self, class_name: str):
        """该类别最新一帧的地图坐标（逐帧投影，非记忆）。"""
        return self.frame_targets.get(class_name)

    def set_want_classes(self, classes) -> None:
        self._want_classes = tuple(classes)

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
