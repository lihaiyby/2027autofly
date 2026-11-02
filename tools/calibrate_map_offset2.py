#!/usr/bin/env python3
"""标定 v2：直接读逐帧投影 /uav_vision/detections_mapped（非粘性），而非记忆里的融合点。

同时对比两种来源（记忆 vs 逐帧），确认哪个坐标可信。
"""
from __future__ import annotations

import json
import math
import os
import sys

import rospy
from uav_vision.msg import TargetCandidateArray, TargetDetectionArray


def _find_root(start):
    cur = os.path.abspath(start)
    for _ in range(6):
        if os.path.isdir(os.path.join(cur, "src", "autofly")):
            return cur
        cur = os.path.dirname(cur)
    return os.path.dirname(os.path.abspath(start))


ROOT = _find_root(__file__)
sys.path.insert(0, os.path.join(ROOT, "src"))

from autofly.flight import Flight   # noqa: E402


class Collector:
    """同时收集记忆点（sticky）与逐帧投影点（per-frame）。"""

    def __init__(self):
        self.memory = {}     # class -> (x, y, conf)
        self.frame = {}      # class -> (x, y)
        rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, self._mem_cb)
        rospy.Subscriber("/uav_vision/detections_mapped", TargetDetectionArray, self._frame_cb)

    def _mem_cb(self, msg):
        for t in msg.targets:
            if getattr(t, "state", 0) < 2:
                continue
            if not getattr(t, "current_map_valid", False):
                continue
            self.memory[t.class_name] = (t.map_point.x, t.map_point.y,
                                         float(getattr(t, "class_confidence", 0.0)))

    def _frame_cb(self, msg):
        best = {}
        for d in msg.detections:
            if not getattr(d, "map_valid", False):
                continue
            conf = float(getattr(d, "geometry_confidence", 0.0))
            if d.class_name not in best or conf > best[d.class_name][2]:
                best[d.class_name] = (d.map_point.x, d.map_point.y, conf)
        self.frame = {k: (v[0], v[1]) for k, v in best.items()}


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/liftrace_sim/scenes/scene_42.json")
    z = float(sys.argv[2]) if len(sys.argv) > 2 else 2.3
    meta = json.load(open(scene, encoding="utf-8"))
    targets = [(t["name"], float(t["x"]), float(t["y"])) for t in meta["targets"]]

    rospy.init_node("calibrate_map_offset2")
    flight = Flight()
    col = Collector()
    flight.wait_services()
    flight.set_params({
        "COM_ARM_EKF_YAW": 3.14, "COM_DISARM_PRFLT": -1.0, "COM_OF_LOSS_T": 5.0,
        "MPC_XY_VEL_MAX": 0.8, "MPC_XY_CRUISE": 0.8,
    })
    flight.hold(0.0, 0.0, 1.5, 3.0)
    for _ in range(30):
        if flight.arm():
            break
        rospy.sleep(1.0)
    for _ in range(30):
        if flight.set_offboard():
            break
        rospy.sleep(0.5)
    if not flight.takeoff(1.5, retries=3):
        print("[cal2] 起飞失败")
        return 1
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 120:
        if col.frame or col.memory:
            break
        flight.rate.sleep()

    rows = []
    for name, x, y in targets:
        print("[cal2] 悬停 %s 真值 (%.2f, %.2f, %.2f)" % (name, x, y, z))
        if not flight.goto(x, y, z, tol=0.35, timeout=120.0, label="cal-" + name):
            rows.append((name, x, y, None, None, None, None))
            continue
        flight.hold(x, y, z, 10.0)
        fm = col.frame.get(name)
        mm = col.memory.get(name)
        rows.append((name, x, y,
                     (fm[0], fm[1]) if fm else None,
                     (mm[0], mm[1]) if mm else None,
                     flight.x, flight.y))

    print("\n[cal2] ===== 逐帧投影 vs 记忆 =====")
    print("%-10s %16s %22s %22s" % ("class", "truth", "per-frame(map)  Δ",
                                    "memory(map)  Δ"))
    fdx, fdy, mdx, mdy = [], [], [], []
    for name, x, y, fm, mm, ex, ey in rows:
        s_f = "%s" % ("-" if fm is None else "(%.2f,%.2f) (%.2f,%.2f)" % (fm[0], fm[1], fm[0] - x, fm[1] - y))
        s_m = "%s" % ("-" if mm is None else "(%.2f,%.2f) (%.2f,%.2f)" % (mm[0], mm[1], mm[0] - x, mm[1] - y))
        print("%-10s (%5.2f,%5.2f) %22s %22s" % (name, x, y, s_f, s_m))
        if fm:
            fdx.append(fm[0] - x)
            fdy.append(fm[1] - y)
        if mm:
            mdx.append(mm[0] - x)
            mdy.append(mm[1] - y)

    def stat(dxs, dys, tag):
        if not dxs:
            print("[cal2] %s: 无数据" % tag)
            return
        n = len(dxs)
        mx, my = sum(dxs) / n, sum(dys) / n
        sx = math.sqrt(sum((d - mx) ** 2 for d in dxs) / n)
        sy = math.sqrt(sum((d - my) ** 2 for d in dys) / n)
        print("[cal2] %s: 平均偏差 (%.2f, %.2f) 标准差 (%.2f, %.2f)  样本 %d"
              % (tag, mx, my, sx, sy, n))

    stat(fdx, fdy, "逐帧投影")
    stat(mdx, mdy, "记忆融合")
    flight.auto_land()
    return 0


if __name__ == "__main__":
    sys.exit(main())
