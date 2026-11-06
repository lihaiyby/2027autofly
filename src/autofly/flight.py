"""MAVROS 飞行原语：解锁、OFFBOARD、位置设定点、模式守护。"""
from __future__ import annotations

import math

# LIO 在高于约 2.1 m 时几何约束不足会发散（实测 2.45 m 起��现上百米偏差），
# 因此把设定点高度硬夹在此上限内。
MAX_SAFE_Z = 2.00
from typing import Callable, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode


class Flight:
    """把"发设定点 + 保模式"收敛成几个方法，供各 phase 复用。"""

    def __init__(self, rate_hz: int = 20):
        self.rate = rospy.Rate(rate_hz)
        self.pub = rospy.Publisher("/mavros/setpoint_position/local",
                                   PoseStamped, queue_size=20)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.param_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self.x = self.y = self.z = 0.0
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)

    # ---------- 状态 ----------
    def _pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.x, self.y, self.z = p.x, p.y, p.z

    def state(self) -> Optional[State]:
        try:
            return rospy.wait_for_message("/mavros/state", State, timeout=3)
        except rospy.ROSException:
            return None

    def wait_services(self, timeout: float = 30.0) -> bool:
        """等待 MAVROS 服务；超时返回 False（不抛异常，交给上层重试）。"""
        for name in ("/mavros/cmd/arming", "/mavros/set_mode"):
            try:
                rospy.wait_for_service(name, timeout=timeout)
            except rospy.ROSException:
                rospy.logwarn("[flight] 等待服务 %s 超时", name)
                return False
        return True

    def wait_connected(self, timeout: float = 120.0) -> bool:
        """等 MAVROS 与飞控连上（connected: True）。"""
        end = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            st = self.state()
            if st is not None and st.connected:
                return True
            rospy.sleep(1.0)
        return False

    # ---------- 参数 ----------
    def set_params(self, params: dict) -> None:
        """PX4 参数（名字 -> float），失败只告警不中断。"""
        for name, value in params.items():
            try:
                self.param_srv(param_id=name, value=ParamValue(integer=0, real=float(value)))
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("[flight] 参数 %s 设置失败: %s", name, exc)

    # ---------- 设定点 ----------
    def sp(self, x: float, y: float, z: float) -> None:
        z = min(z, MAX_SAFE_Z)
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        self.pub.publish(m)

    def hold(self, x: float, y: float, z: float, seconds: float) -> None:
        end = rospy.Time.now() + rospy.Duration(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self.sp(x, y, z)
            self.rate.sleep()

    # ---------- 模式 ----------
    def arm(self) -> bool:
        try:
            return bool(self.arm_srv(True).success)
        except Exception:  # noqa: BLE001
            return False

    def set_offboard(self) -> bool:
        try:
            return bool(self.mode_srv(custom_mode="OFFBOARD").mode_sent)
        except Exception:  # noqa: BLE001
            return False

    def auto_land(self) -> None:
        try:
            self.mode_srv(custom_mode="AUTO.LAND")
        except Exception:  # noqa: BLE001
            pass

    def ensure(self, setpoint: Callable[[], None]) -> None:
        """掉锁补解锁、掉出 OFFBOARD 重建（每 2 秒调一次即可）。"""
        st = self.state()
        if st is None:
            return
        if not st.armed:
            rospy.logwarn("[flight] 未解锁，补解锁")
            self.arm()
            rospy.sleep(1.0)
        if st.mode != "OFFBOARD":
            rospy.logwarn("[flight] 模式 %s，重建 OFFBOARD", st.mode)
            self.set_offboard()
            rospy.sleep(0.5)
        setpoint()

    # ---------- 航点 ----------
    def goto(self, x: float, y: float, z: float, tol: float = 0.40,
             timeout: float = 120.0, label: str = "") -> bool:
        end = rospy.Time.now() + rospy.Duration(timeout)
        tick = 0
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if abs(self.x) > 50.0 or abs(self.y) > 50.0 or self.z < -5.0:
                rospy.logerr("[flight] %s 状态发散 (%.1f, %.1f, %.1f)", label, self.x, self.y, self.z)
                raise SystemExit(3)
            self.sp(x, y, z)
            tick += 1
            if tick % 40 == 0:
                self.ensure(lambda: self.sp(x, y, z))
            d = math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2 + (self.z - z) ** 2)
            if d < tol:
                rospy.loginfo("[flight] %s 到达 (%.2f, %.2f, %.2f) d=%.2f", label, x, y, z, d)
                return True
            self.rate.sleep()
        rospy.logwarn("[flight] %s 超时 (%.2f, %.2f, %.2f)", label, x, y, z)
        return False

    def takeoff(self, z: float, retries: int = 3) -> bool:
        """解锁 + OFFBOARD + 爬升，失败自动重试。"""
        for attempt in range(1, retries + 1):
            if self.goto(0.0, 0.0, z, tol=0.35, timeout=45.0, label=f"takeoff-{attempt}"):
                return True
            st = self.state()
            if st is not None and not st.armed:
                self.arm()
            self.set_offboard()
            self.hold(0.0, 0.0, z, 2.0)
        return False
