#!/usr/bin/env python3
"""Liftrace 任务驱动 v5：起飞 → 搜索发现三靶 → 逐靶投递 → 走廊 → 尽头降落。

与 v4 的区别：起飞时**不知道靶标位置**，先按割草机航线搜索，
从 `/uav_vision/targets` 读取已确认目标的**地图坐标**，找齐 3 个不同类别的靶标后再去投递。

用法: mission_driver.py [search_z] [drop_z]
   search_z 搜索高度，默认 2.4（保证圆环完整入画）
   drop_z   投递对准高度，默认 2.3
"""
import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue, State, StatusText
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from std_msgs.msg import Bool, String
from uav_vision.msg import DropReady, TargetCandidateArray

S = {"x": 0.0, "y": 0.0, "z": 0.0,
     "ready": False, "reason": "", "servo_done": False,
     "found": {},            # class -> (x, y, conf)
     "ntgt": 0}

# 搜索用的靶标类别（red_cross 不是投递目标）
WANT_CLASSES = ("tent", "pillbox", "bridge", "panzer")

# 割草机航线：x 从 0.8 到 7.2，y 往返 -4.0 / +4.0
SEARCH_LANES = [(0.8, -4.0), (0.8, 4.0), (2.6, 4.0), (2.6, -4.0),
                (4.4, -4.0), (4.4, 4.0), (6.2, 4.0), (6.2, -4.0), (7.2, -4.0)]

CORRIDOR = [
    (8.70, 4.30, 1.20),
    (9.10, 3.60, 1.10),
    (9.15, 2.80, 1.00),
    (8.80, 2.20, 1.00),
    (8.45, 1.60, 1.00),      # 门 A
    (8.45, 0.90, 1.00),
    (8.45, 0.20, 1.00),
    (8.80, -0.70, 1.00),
    (9.15, -1.60, 1.00),     # 门 B
    (8.85, -2.50, 1.00),
    (8.60, -3.40, 1.00),
]
EXIT_PAD = (8.75, -4.20)


def pose_cb(m):
    p = m.pose.position
    S["x"], S["y"], S["z"] = p.x, p.y, p.z


def ready_cb(m):
    S["ready"] = m.ready
    S["reason"] = m.reason


def servo_cb(m):
    if m.data:
        S["servo_done"] = True


def targets_cb(m):
    S["ntgt"] = len(m.targets)
    for t in m.targets:
        if t.class_name not in WANT_CLASSES:
            continue
        if getattr(t, "state", 0) < 2:          # 只接受 CONFIRMED
            continue
        if not getattr(t, "current_map_valid", getattr(t, "map_valid", False)):
            continue
        mp = t.map_point
        if not (math.isfinite(mp.x) and math.isfinite(mp.y)):
            continue
        if abs(mp.x) > 12 or abs(mp.y) > 12:
            continue
        old = S["found"].get(t.class_name)
        conf = float(getattr(t, "class_confidence", 0.0))
        if old is None or conf >= old[2]:
            S["found"][t.class_name] = (mp.x, mp.y, conf)


def statustext_cb(m):
    txt = getattr(m, "text", "")
    if any(k in txt.lower() for k in ("preflight", "denied", "arming")):
        rospy.logwarn("[px4] %s", txt)


def main():
    rospy.init_node("liftrace_mission_driver")
    search_z = float(sys.argv[1]) if len(sys.argv) > 1 else 2.4
    drop_z = float(sys.argv[2]) if len(sys.argv) > 2 else 2.3

    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=20)
    mode_pub = rospy.Publisher("/uav_vision/align_mode", String, queue_size=2, latch=True)
    servo_pub = rospy.Publisher("/detect/servo_status", Bool, queue_size=2)
    land_pub = rospy.Publisher("/detect/landing_control", Bool, queue_size=2, latch=True)
    ctl_pub = rospy.Publisher("/detect/control", Bool, queue_size=2, latch=True)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/uav_vision/targets", TargetCandidateArray, targets_cb)
    rospy.Subscriber("/uav_vision/drop_ready", DropReady, ready_cb)
    rospy.Subscriber("/detect/servo_complete", Bool, servo_cb)
    rospy.Subscriber("/mavros/statustext/recv", StatusText, statustext_cb)
    threading.Thread(target=rospy.spin, daemon=True).start()

    rospy.wait_for_service("/mavros/cmd/arming", timeout=60)
    rospy.wait_for_service("/mavros/set_mode", timeout=60)
    arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    try:
        pset = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        pset(param_id="COM_ARM_EKF_YAW", value=ParamValue(integer=0, real=3.14))
        pset(param_id="COM_DISARM_PRFLT", value=ParamValue(integer=0, real=-1.0))
        pset(param_id="COM_OF_LOSS_T", value=ParamValue(integer=0, real=5.0))
        pset(param_id="MPC_XY_VEL_MAX", value=ParamValue(integer=0, real=0.6))
        pset(param_id="MPC_XY_CRUISE", value=ParamValue(integer=0, real=0.6))
    except Exception as exc:  # noqa: BLE001
        rospy.logwarn("[driver] param set failed: %s", exc)
    rate = rospy.Rate(20)

    def sp(x, y, z):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        pub.publish(m)

    def hold(x, y, z, sec):
        end = rospy.Time.now() + rospy.Duration(sec)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            sp(x, y, z)
            rate.sleep()

    def ensure_flight():
        """掉出 OFFBOARD 或掉锁时自动恢复。"""
        try:
            st = rospy.wait_for_message("/mavros/state", State, timeout=3)
        except rospy.ROSException:
            return
        if not st.armed:
            rospy.logwarn("[driver] not armed -> re-arm")
            arm(True)
            rospy.sleep(1.0)
        if st.mode != "OFFBOARD":
            rospy.logwarn("[driver] mode=%s -> re-OFFBOARD", st.mode)
            set_mode(custom_mode="OFFBOARD")
            rospy.sleep(0.5)

    def goto(x, y, z, tol=0.40, timeout=120.0, label=""):
        end = rospy.Time.now() + rospy.Duration(timeout)
        i_tick = 0
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if abs(S["x"]) > 50.0 or abs(S["y"]) > 50.0 or S["z"] < -5.0:
                rospy.logerr("[driver] diverged (%.1f,%.1f,%.1f), abort", S["x"], S["y"], S["z"])
                raise SystemExit(3)
            sp(x, y, z)
            i_tick += 1
            if i_tick % 40 == 0:
                ensure_flight()
            d = math.sqrt((S["x"] - x) ** 2 + (S["y"] - y) ** 2 + (S["z"] - z) ** 2)
            if d < tol:
                rospy.loginfo("[driver] %s reached (%.2f,%.2f,%.2f) d=%.2f", label, x, y, z, d)
                return True
            rate.sleep()
        rospy.logwarn("[driver] %s timeout (%.2f,%.2f,%.2f)", label, x, y, z)
        return False

    # ---------- 1. 解锁 + 起飞 ----------
    rospy.loginfo("[driver] 1/5 ARM + takeoff")
    hold(0.0, 0.0, 1.5, 3.0)
    armed, t0 = False, rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 90.0:
        if arm(True).success:
            armed = True
            break
        sp(0.0, 0.0, 1.5)
        rospy.sleep(1.0)
    for _ in range(60):
        if set_mode(custom_mode="OFFBOARD").mode_sent:
            break
        sp(0.0, 0.0, 1.5)
        rospy.sleep(0.5)
    ok = False
    for attempt in range(1, 4):
        if goto(0.0, 0.0, 1.5, tol=0.35, timeout=45.0, label=f"takeoff-{attempt}"):
            ok = True
            break
        if not rospy.wait_for_message("/mavros/state", State, timeout=5).armed:
            arm(True)
        set_mode(custom_mode="OFFBOARD")
        hold(0.0, 0.0, 1.5, 2.0)
    rospy.loginfo("[driver] takeoff ok=%s", ok)

    # 等视觉链
    t_v = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t_v).to_sec() < 150.0:
        if S["ntgt"] > 0 or rospy.Time.now() - t_v > rospy.Duration(20):
            break
        rate.sleep()

    # ---------- 2. 搜索 ----------
    rospy.loginfo("[driver] 2/5 SEARCH (未知靶标位置，割草机扫描)")
    goto(0.0, 0.0, search_z, tol=0.40, timeout=120.0, label="search-up")
    t_search = rospy.Time.now()
    for i, (lx, ly) in enumerate(SEARCH_LANES, 1):
        if len(S["found"]) >= 3:
            break
        goto(lx, ly, search_z, tol=0.55, timeout=90.0, label=f"lane-{i}")
        rospy.loginfo("[driver]   found=%s", {k: (round(v[0], 2), round(v[1], 2))
                                             for k, v in S["found"].items()})
        if (rospy.Time.now() - t_search).to_sec() > 420:
            rospy.logwarn("[driver] search timeout")
            break

    found = sorted(S["found"].items(), key=lambda kv: -kv[1][2])[:3]
    rospy.loginfo("[driver] 搜索完成 %.0fs，发现 %d 个目标: %s",
                  (rospy.Time.now() - t_search).to_sec(), len(found),
                  [(k, round(v[0], 2), round(v[1], 2)) for k, v in found])

    # ---------- 3. 逐靶投递 ----------
    dropped = []
    for idx, (cls, (tx, ty, _c)) in enumerate(found, 1):
        ok_all = False
        for attempt in range(1, 4):
            rospy.loginfo("[driver] 3/5 drop %d/%d %s @ (%.2f, %.2f) attempt %d",
                          idx, len(found), cls, tx, ty, attempt)
            if not goto(tx, ty, drop_z, tol=0.45, timeout=120.0, label=f"{cls}-approach{attempt}"):
                rospy.logwarn("[driver]   %s approach failed", cls)
                continue
            hold(tx, ty, drop_z, 2.0)
            mode_pub.publish(String(data="drop_circle"))
            ctl_pub.publish(Bool(data=True))
            S["servo_done"] = False
            S["ready"] = False
            stable, okd, last = 0, False, None
            t_a = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - t_a).to_sec() < 150.0:
                sp(tx, ty, drop_z)
                if S["reason"] != last:
                    last = S["reason"]
                    rospy.loginfo("[driver]   %s align -> %s (targets=%d)", cls, last, S["ntgt"])
                stable = stable + 1 if S["ready"] else 0
                if stable >= 3:
                    for _ in range(5):
                        servo_pub.publish(Bool(data=True))
                        rate.sleep()
                    okd = True
                    break
                rate.sleep()
            if okd:
                endw = rospy.Time.now() + rospy.Duration(10.0)
                while not rospy.is_shutdown() and rospy.Time.now() < endw and not S["servo_done"]:
                    rate.sleep()
            ok_all = bool(okd and S["servo_done"])
            rospy.loginfo("[driver]   %s attempt %d dropped=%s servo=%s",
                          cls, attempt, okd, S["servo_done"])
            ctl_pub.publish(Bool(data=False))
            mode_pub.publish(String(data="disabled"))
            if ok_all:
                break
            # 重试前挪开一点，重新观察（避免停在同一点反复 stale）
            goto(tx + 0.8, ty + 0.8, drop_z, tol=0.45, timeout=60.0, label=f"{cls}-reset{attempt}")
        dropped.append(ok_all)

    # ---------- 4. 走廊 ----------
    rospy.loginfo("[driver] 4/5 corridor")
    for i, (cx, cy, cz) in enumerate(CORRIDOR, 1):
        if not goto(cx, cy, cz, tol=0.35, timeout=90.0, label=f"corridor-{i}"):
            break
    goto(8.60, -3.60, 2.20, tol=0.40, timeout=90.0, label="corridor-lift")
    goto(EXIT_PAD[0], EXIT_PAD[1], 1.60, tol=0.45, timeout=120.0, label="corridor-end")

    # ---------- 5. 尽头降落 ----------
    rospy.loginfo("[driver] 5/5 land on corridor-end pad")
    land_pub.publish(Bool(data=True))
    mode_pub.publish(String(data="landing"))
    tgt_land = [EXIT_PAD[0], EXIT_PAD[1]]
    t_l = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t_l).to_sec() < 40.0:
        sp(tgt_land[0], tgt_land[1], 1.60)
        rate.sleep()
    for z in (1.20, 0.90, 0.60, 0.40):
        goto(tgt_land[0], tgt_land[1], z, tol=0.30, timeout=45.0, label=f"descend-{z}")
    mode_pub.publish(String(data="disabled"))
    land_pub.publish(Bool(data=False))
    set_mode(custom_mode="AUTO.LAND")
    end = rospy.Time.now() + rospy.Duration(90.0)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        if S["z"] < 0.15:
            break
        rate.sleep()
    rospy.loginfo("[driver] DONE found=%d drops=%s final=(%.2f,%.2f,%.2f)",
                  len(found), dropped, S["x"], S["y"], S["z"])


if __name__ == "__main__":
    main()
