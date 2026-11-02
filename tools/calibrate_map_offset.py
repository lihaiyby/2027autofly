#!/usr/bin/env python3
"""标定 /uav_vision/targets 的 map_point 相对真值的偏移。

用法：
    python3 tools/calibrate_map_offset.py <scene.json> [高度]

流程：起飞到指定高度 → 依次悬停在场景真值坐标上方 → 记录该位置报出的目标地图坐标
      → 输出 (报出 - 真值) 偏差表与统计。
用途：如果偏差在各靶标上一致，就是常量补偿；否则说明相机外参/标定有问题。
"""
from __future__ import annotations

import json
import math
import os
import sys

import rospy

def _find_root(start):
    """向上查找含 src/autofly 的目录。"""
    cur = os.path.abspath(start)
    for _ in range(6):
        if os.path.isdir(os.path.join(cur, "src", "autofly")):
            return cur
        cur = os.path.dirname(cur)
    return os.path.dirname(os.path.abspath(start))


ROOT = _find_root(__file__)
sys.path.insert(0, os.path.join(ROOT, "src"))

from autofly.flight import Flight          # noqa: E402
from autofly.vision import Vision          # noqa: E402


def load_targets(path):
    meta = json.load(open(path, encoding="utf-8"))
    return [(t["name"], float(t["x"]), float(t["y"])) for t in meta["targets"]]


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/liftrace_sim/scenes/scene_42.json")
    z = float(sys.argv[2]) if len(sys.argv) > 2 else 2.3
    targets = load_targets(scene)
    print("[cal] 场景真值:", ", ".join("%s(%.2f,%.2f)" % t for t in targets))

    rospy.init_node("calibrate_map_offset")
    flight = Flight()
    seen = {}
    vision = Vision(on_confirmed=lambda items: [seen.setdefault(c, (x, y, cf)) for c, x, y, cf in items])
    vision.set_want_classes(tuple(t[0] for t in targets))

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
        print("[cal] 起飞失败")
        return 1
    # 等视觉链
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 120:
        if vision.ntgt > 0:
            break
        flight.rate.sleep()

    rows = []
    for name, x, y in targets:
        print("[cal] 飞往 %s 真值 (%.2f, %.2f, %.2f)" % (name, x, y, z))
        if not flight.goto(x, y, z, tol=0.35, timeout=120.0, label="cal-" + name):
            rows.append((name, x, y, None, None, None, None))
            continue
        flight.hold(x, y, z, 12.0)
        got = seen.get(name)
        if got is None:
            rows.append((name, x, y, None, None, None, None))
            continue
        mx, my, _cf = got
        rows.append((name, x, y, mx, my, mx - x, my - y))

    print("\n[cal] ===== 结果 =====")
    print("%-10s %8s %8s %8s %8s %7s %7s" % ("class", "truth_x", "truth_y", "map_x", "map_y", "dx", "dy"))
    dxs, dys = [], []
    for name, x, y, mx, my, dx, dy in rows:
        if mx is None:
            print("%-10s %8.2f %8.2f %8s %8s %7s %7s" % (name, x, y, "-", "-", "-", "-"))
            continue
        print("%-10s %8.2f %8.2f %8.2f %8.2f %7.2f %7.2f" % (name, x, y, mx, my, dx, dy))
        dxs.append(dx)
        dys.append(dy)
    if dxs:
        n = len(dxs)
        mx = sum(dxs) / n
        my = sum(dys) / n
        sp = math.sqrt(sum((d - mx) ** 2 for d in dxs) / n)
        spy = math.sqrt(sum((d - my) ** 2 for d in dys) / n)
        print("\n[cal] 平均偏差 dx=%.2f dy=%.2f  (标准差 %.2f / %.2f)" % (mx, my, sp, spy))
        print("[cal] 若标准差 < 0.3 m，可在 config 里用常量补偿 map_offset: [%.2f, %.2f]" % (mx, my))
        print("[cal] 否则说明是外参/标定问题，需要重标相机")
    flight.auto_land()
    return 0


if __name__ == "__main__":
    sys.exit(main())
