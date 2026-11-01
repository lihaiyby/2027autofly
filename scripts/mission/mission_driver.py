#!/usr/bin/env python3
"""Liftrace 整场任务驱动 v4：起飞 → 三靶投递 → 走廊（北口进）→ 走廊尽头降落。

用法:
  mission_driver.py "<x1,y1;x2,y2;x3,y3>" [cruise] [exit_x,exit_y]

  参数1 目标列表（分号分隔，投递顺序）
  参数2 巡航高度，默认 1.5
  参数3 走廊尽头降落点，默认 "8.75,-4.20"（landing_h_clone）

流程:
  1) 起飞到巡航高度
  2) 依次飞到各靶标上方 → 视觉对准 → 投递
  3) 飞到走廊北口 → 蛇形穿 6 段 → 爬到 2.2 m 越过 Wall_13 → 到走廊尽头
  4) 视觉锁定走廊尽头的 H 板 → 分级下降 → AUTO.LAND
"""
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue, StatusText
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from std_msgs.msg import Bool, String
from uav_vision.msg import DropReady, TargetCandidateArray

S = {"x": 0.0, "y": 0.0, "z": 0.0, "mark": None, "mark_t": None, "ntgt": 0,
     "land": None, "land_t": None, "ready": False, "reason": "",
     "servo_done": False}

# 走廊：北口 (8.70, 4.30) 进，往南蛇形；门 A y=1.6 走 x<8.8，门 B y=-1.6 走 x>8.7
CORRIDOR = [
    (8.70, 4.30, 1.20),
    (9.15, 2.80, 1.00),
    (8.45, 1.60, 1.00),
    (8.45, 0.20, 1.00),
    (9.15, -1.60, 1.00),
    (8.60, -3.40, 1.00),
]


def pose_cb(m):
    p = m.pose.position
    S["x"], S["y"], S["z"] = p.x, p.y, p.z


def mark_cb(m):
    S["mark"] = (m.pose.position.x, m.pose.position.y)
    S["mark_t"] = rospy.Time.now()


def land_cb(m):
    S["land"] = (m.pose.position.x, m.pose.position.y)
    S["land_t"] = rospy.Time.now()


def targets_cb(m):
    S["ntgt"] = len(m.targets)


def ready_cb(m):
    S["ready"] = m.ready
    S["reason"] = m.reason


def servo_cb(m):
    if m.data:
        S["servo_done"] = True


def statustext_cb(m):
    txt = getattr(m, "text", "")
    if any(k in txt.lower() for k in ("preflight", "denied", "arming")):
        rospy.logwarn("[px4] %s", txt)


def parse_targets(arg):
    out = []
    for part in (arg or "").split(";"):
        part = part.strip()
        if not part:
            continue
        xs, ys = part.split(",")
        out.append((float(xs), float(ys)))
    return out or [(7.05, -4.13), (4.64, -3.30), (4.90, -0.71)]


def main():
    rospy.init_node("liftrace_mission_driver")
    targets = parse_targets(sys.argv[1] if len(sys.argv) > 1 else "")
    cruise = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
    drop_z = float(sys.argv[4]) if len(sys.argv) > 4 else 2.3
    if len(sys.argv) > 3:
        ex, ey = sys.argv[3].split(",")
        exit_xy = (float(ex), float(ey))
    else:
        exit_xy = (8.75, -4.20)

    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=20)
    mode_pub = rospy.Publisher("/uav_vision/align_mode", String, queue_size=2, latch=True)
    servo_pub = rospy.Publisher("/detect/servo_status", Bool, queue_size=2)
    land_pub = rospy.Publisher("/detect/landing_control", Bool, queue_size=2, latch=True)
    ctl_pub = rospy.Publisher("/detect/control", Bool, queue_size=2, latch=True)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/detect/waypoint_mark_point", PoseStamped, mark_cb)
    rospy.Subscriber("/detect/land_mark_point", PoseStamped, land_cb)
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
        # 关键：px4 在解锁后若预检仍不通过会自动上锁（Disarmed by auto preflight disarming）
        pset(param_id="COM_DISARM_PRFLT", value=ParamValue(integer=0, real=-1.0))
        # 视觉链满载时 20 Hz 设定点循环会卡顿，PX4 默认 OFFBOARD 失联超时很短
        pset(param_id="COM_OF_LOSS_T", value=ParamValue(integer=0, real=5.0))
        pset(param_id="MPC_XY_VEL_MAX", value=ParamValue(integer=0, real=0.8))
        pset(param_id="MPC_XY_CRUISE", value=ParamValue(integer=0, real=0.8))
        pset(param_id="MPC_JERK_MAX", value=ParamValue(integer=0, real=2.0))
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

    def goto(x, y, z, tol=0.35, timeout=120.0, label=""):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if abs(S["x"]) > 50.0 or abs(S["y"]) > 50.0 or S["z"] < -5.0:
                rospy.logerr("[driver] state diverged (%.1f, %.1f, %.1f), abort",
                             S["x"], S["y"], S["z"])
                raise SystemExit(3)
            sp(x, y, z)
            d = ((S["x"] - x) ** 2 + (S["y"] - y) ** 2 + (S["z"] - z) ** 2) ** 0.5
            if d < tol:
                rospy.loginfo("[driver] %s reached (%.2f, %.2f, %.2f) d=%.2f", label, x, y, z, d)
                return True
            rate.sleep()
        rospy.logwarn("[driver] %s timeout at (%.2f, %.2f, %.2f)", label, x, y, z)
        return False

    # ---------- 1. 解锁 + 起飞 ----------
    rospy.loginfo("[driver] 1/5 ARM (before OFFBOARD) + takeoff")
    hold(0.0, 0.0, cruise, 3.0)
    armed, t0 = False, rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 90.0:
        if arm(True).success:
            armed = True
            break
        sp(0.0, 0.0, cruise)
        rospy.sleep(1.0)
    rospy.loginfo("[driver] armed=%s (%.0fs)", armed, (rospy.Time.now() - t0).to_sec())

    # 等 2 秒确认没被自动上锁；掉了就补一次
    for _ in range(3):
        rospy.sleep(2.0)
        st = rospy.wait_for_message("/mavros/state", __import__("mavros_msgs.msg", fromlist=["State"]).State, timeout=5)
        if st.armed:
            break
        rospy.logwarn("[driver] auto-disarmed, re-arm")
        arm(True)
    offboard = False
    for _ in range(60):
        if set_mode(custom_mode="OFFBOARD").mode_sent:
            offboard = True
            break
        sp(0.0, 0.0, cruise)
        rospy.sleep(0.5)
    rospy.loginfo("[driver] OFFBOARD=%s", offboard)
    ok_takeoff = False
    for attempt in range(1, 4):
        if goto(0.0, 0.0, cruise, tol=0.35, timeout=45.0, label=f"takeoff-{attempt}"):
            ok_takeoff = True
            break
        rospy.logwarn("[driver] takeoff attempt %d failed, re-arm + re-offboard", attempt)
        st = rospy.wait_for_message("/mavros/state",
                                    __import__("mavros_msgs.msg", fromlist=["State"]).State,
                                    timeout=5)
        if not st.armed:
            arm(True)
        set_mode(custom_mode="OFFBOARD")
        hold(0.0, 0.0, cruise, 2.0)
    rospy.loginfo("[driver] takeoff ok=%s", ok_takeoff)

    # 等视觉链就绪（第一个靶之前），避免对准窗口还没等到视觉就超时
    rospy.loginfo("[driver] waiting for vision chain ...")
    cnt = {"n": 0}
    rospy.Subscriber("/uav_vision/detections_mapped", rospy.AnyMsg,
                     lambda _m: cnt.__setitem__("n", cnt["n"] + 1))
    t_v = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t_v).to_sec() < 150.0:
        if cnt["n"] > 5:
            break
        sp(0.0, 0.0, cruise)
        rate.sleep()
    rospy.loginfo("[driver] vision ready after %.0fs (%d frames)", (rospy.Time.now() - t_v).to_sec(), cnt["n"])

    # ---------- 2. 三靶投递 ----------
    dropped = []
    for idx, (tx, ty) in enumerate(targets, 1):
        rospy.loginfo("[driver] 2/5 target %d/%d (%.2f, %.2f)", idx, len(targets), tx, ty)
        if not goto(tx, ty, cruise, tol=0.40, timeout=180.0, label=f"target{idx}"):
            dropped.append(False)
            continue
        # 抬高到投递高度：太低会让圆环被画面裁掉，无法精修
        goto(tx, ty, drop_z, tol=0.30, timeout=60.0, label=f"target{idx}-up")
        hold(tx, ty, drop_z, 1.0)
        mode_pub.publish(String(data="drop_circle"))
        ctl_pub.publish(Bool(data=True))     # 兼容桥据此发 /detect/waypoint_mark_point
        S["servo_done"] = False
        S["ready"] = False
        tgt = [tx, ty]
        stable, ok = 0, False
        # 先等视觉记忆确认目标（最多 120 秒），再开始对准计时
        t_wait = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t_wait).to_sec() < 120.0:
            if S["ntgt"] > 0:
                break
            sp(tx, ty, drop_z)
            rate.sleep()
        rospy.loginfo("[driver] target %d: targets=%d after %.0fs",
                      idx, S["ntgt"], (rospy.Time.now() - t_wait).to_sec())
        t_align = rospy.Time.now()
        last_reason = None
        i_tick = 0
        while not rospy.is_shutdown() and (rospy.Time.now() - t_align).to_sec() < 240.0:
            i_tick += 1
            if i_tick % 40 == 0:                     # 每 2 秒重申一次对准模式
                mode_pub.publish(String(data="drop_circle"))
            if S["reason"] != last_reason:
                last_reason = S["reason"]
                rospy.loginfo("[driver] target %d align reason -> %s", idx, last_reason)
            # 不再追 mark：悬停在指令点，靠放宽的对准门限判定（避免来回摆）
            sp(tgt[0], tgt[1], drop_z)
            stable = stable + 1 if S["ready"] else 0
            if stable >= 3:
                for _ in range(5):
                    servo_pub.publish(Bool(data=True))
                    rate.sleep()
                ok = True
                break
            rate.sleep()
        if ok:
            end = rospy.Time.now() + rospy.Duration(10.0)
            while not rospy.is_shutdown() and rospy.Time.now() < end and not S["servo_done"]:
                rate.sleep()
        dropped.append(bool(ok and S["servo_done"]))
        rospy.loginfo("[driver] target %d dropped=%s servo=%s reason=%s",
                      idx, ok, S["servo_done"], S["reason"])
        ctl_pub.publish(Bool(data=False))
        mode_pub.publish(String(data="disabled"))

    # ---------- 3. 走廊（北口进 → 蛇形 → 尽头） ----------
    rospy.loginfo("[driver] 3/5 corridor: enter at north (%.2f, %.2f)", CORRIDOR[0][0], CORRIDOR[0][1])
    for i, (cx, cy, cz) in enumerate(CORRIDOR, 1):
        if not goto(cx, cy, cz, tol=0.30, timeout=90.0, label=f"corridor-{i}"):
            rospy.logwarn("[driver] corridor leg %d failed, going to exit pad anyway", i)
            break

    # 爬升越过 Wall_13（高 1.5 m）再往尽头
    goto(8.60, -3.60, 2.20, tol=0.35, timeout=90.0, label="corridor-lift")
    goto(exit_xy[0], exit_xy[1], 1.60, tol=0.40, timeout=120.0, label="corridor-end")

    # ---------- 4. 尽头 H 板视觉降落 ----------
    rospy.loginfo("[driver] 4/5 visual landing on corridor-end pad (%.2f, %.2f)", exit_xy[0], exit_xy[1])
    land_pub.publish(Bool(data=True))
    mode_pub.publish(String(data="landing"))
    tgt_land = [exit_xy[0], exit_xy[1]]
    t_lock = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t_lock).to_sec() < 45.0:
        lm, lt = S["land"], S["land_t"]
        age = 9.9 if lt is None else (rospy.Time.now() - lt).to_sec()
        if lm is not None and age < 1.0:
            dx, dy = lm[0] - exit_xy[0], lm[1] - exit_xy[1]
            if dx * dx + dy * dy <= 1.5 * 1.5:
                tgt_land = [lm[0], lm[1]]
        sp(tgt_land[0], tgt_land[1], 1.60)
        rate.sleep()
    rospy.loginfo("[driver] pad lock -> (%.2f, %.2f)", tgt_land[0], tgt_land[1])

    for z in (1.20, 0.90, 0.60, 0.40):
        goto(tgt_land[0], tgt_land[1], z, tol=0.25, timeout=45.0, label=f"descend-{z}")
    mode_pub.publish(String(data="disabled"))
    land_pub.publish(Bool(data=False))

    # ---------- 5. 触地 ----------
    set_mode(custom_mode="AUTO.LAND")
    end = rospy.Time.now() + rospy.Duration(90.0)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        if S["z"] < 0.15:
            break
        rate.sleep()
    rospy.loginfo("[driver] DONE drops=%s final=(%.2f, %.2f, %.2f)",
                  dropped, S["x"], S["y"], S["z"])


if __name__ == "__main__":
    main()
