"""任务编排：起飞 -> 搜索 -> 投递 -> 走廊 -> 降落。

每个 phase 是一个方法，失败返回 False，由 `run()` 汇总。
"""
from __future__ import annotations

import math
import rospy

from .config import MissionConfig
from .flight import Flight
from .search import TargetBook, lawnmower, order_by_travel
from .vision import Vision


class Mission:
    def __init__(self, cfg: MissionConfig):
        self.cfg = cfg
        self.flight = Flight()
        self.book = TargetBook(cfg.want_classes)
        self.vision = Vision(on_detection=self._on_detections)
        self.vision.center_gate_px = cfg.center_gate_px
        self.vision.set_want_classes(cfg.want_classes)
        self.dropped = []

    # ---------- 内部工具 ----------
    def _log(self, msg: str, *args) -> None:
        rospy.loginfo("[mission] " + msg, *args)

    def _on_detections(self, items) -> None:
        """逐帧投影回调：加常量补偿后入簿记（比记忆融合点可靠）。"""
        ox, oy = self.cfg.map_offset
        self.book.add_many([(c, x + ox, y + oy, cf) for c, x, y, cf in items])

    def _sync_book(self) -> None:
        """兼容保留：累积已在 Vision 回调里完成。"""
        return

    def _wait_vision(self) -> None:
        """等视觉链**有数据**即可，不等"有目标"。

        起飞点下方没有靶标，若等 ntgt>0 必然耗满超时（150 仿真秒，RTF 0.3 时约 8 分钟墙钟），
        表现为"任务卡在起飞后不动"。
        """
        limit = min(30.0, float(self.cfg.vision_wait))
        end = rospy.Time.now() + rospy.Duration(limit)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if getattr(self.vision, "frames", 0) > 3:
                self._log("  视觉链已就绪（已收 %d 帧）", self.vision.frames)
                return
            self.flight.sp(0.0, 0.0, self.cfg.takeoff_z)
            self.flight.rate.sleep()
        rospy.logwarn("[mission] 视觉链 %.0fs 内未见数据，继续搜索", limit)

    # ---------- phase 1：起飞 ----------
    def takeoff(self) -> bool:
        self._log("1/5 起飞")
        if not self.flight.wait_connected(120.0):
            rospy.logerr("[mission] MAVROS 未连接，放弃")
            return False
        ok_srv = False
        for _ in range(3):
            if self.flight.wait_services(20.0):
                ok_srv = True
                break
            rospy.sleep(2.0)
        if not ok_srv:
            rospy.logerr("[mission] MAVROS 服务不可用，放弃")
            return False
        self.flight.set_params({
            "COM_ARM_EKF_YAW": 3.14,
            "COM_DISARM_PRFLT": -1.0,     # 关掉解锁后自动上锁
            "COM_OF_LOSS_T": 5.0,         # OFFBOARD 失联容忍
            "MPC_XY_VEL_MAX": self.cfg.max_xy_speed,
            "MPC_XY_CRUISE": self.cfg.max_xy_speed,
        })
        self.flight.hold(0.0, 0.0, self.cfg.takeoff_z, 3.0)
        armed = False
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 90.0:
            if self.flight.arm():
                armed = True
                break
            self.flight.sp(0.0, 0.0, self.cfg.takeoff_z)
            rospy.sleep(1.0)
        self._log("  armed=%s", armed)
        for _ in range(60):
            if self.flight.set_offboard():
                break
            self.flight.sp(0.0, 0.0, self.cfg.takeoff_z)
            rospy.sleep(0.5)
        ok = self.flight.takeoff(self.cfg.takeoff_z, self.cfg.takeoff_retries)
        self._log("  起飞成功=%s", ok)
        if ok:
            self._wait_vision()
        return ok

    # ---------- phase 2：搜索 ----------
    def search(self) -> int:
        self._log("2/5 搜索（起飞前不知道靶标位置）")
        self.flight.goto(0.0, 0.0, self.cfg.search_z, tol=0.40,
                         timeout=120.0, label="search-up")
        lanes = lawnmower(self.cfg.search_lanes)
        t0 = rospy.Time.now()
        for i, (lx, ly) in enumerate(lanes, 1):
            self._sync_book()
            if len(self.book) >= self.cfg.max_targets:
                break
            if (rospy.Time.now() - t0).to_sec() > self.cfg.search_timeout:
                rospy.logwarn("[mission] 搜索超时")
                break
            self.flight.goto(lx, ly, self.cfg.search_z, tol=self.cfg.lane_tol,
                             timeout=90.0, label=f"lane-{i}")
            self._sync_book()
            self._log("  lane %d/%d 已发现 %s", i, len(lanes), self.book.as_dict())
        self._log("  搜索结束，用时 %.0fs，发现 %d 个目标",
                  (rospy.Time.now() - t0).to_sec(), len(self.book))
        return len(self.book)

    # ---------- phase 3：投递 ----------
    def _deliver_one(self, cls: str, x: float, y: float) -> bool:
        for attempt in range(1, self.cfg.drop_retries + 1):
            self._log("  投递 %s (%0.2f, %0.2f) 第 %d 次", cls, x, y, attempt)
            if not self.flight.goto(x, y, self.cfg.drop_z, tol=0.45, timeout=120.0,
                                    label=f"{cls}-approach{attempt}"):
                continue
            self.flight.hold(x, y, self.cfg.drop_z, 2.0)
            self.vision.set_mode("drop_circle")
            self.vision.enable_detect(True)
            self.vision.reset_ready()
            self.vision.servo_done = False        # 每个靶标重新计回执
            stable = 0
            ok = False
            last = None
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < self.cfg.align_timeout:
                # 追圆环：逐帧投影给出靶心的地图坐标，限幅逼近
                cur = self.vision.latest_frame(cls)
                if cur is not None:
                    dx, dy = cur[0] - x, cur[1] - y
                    dist = (dx * dx + dy * dy) ** 0.5
                    if 0.02 < dist < 2.5:
                        step = min(1.0, self.cfg.chase_gain)
                        x += max(-0.35, min(0.35, dx * step))
                        y += max(-0.35, min(0.35, dy * step))
                self.flight.sp(x, y, self.cfg.drop_z)
                if self.vision.reason != last:
                    last = self.vision.reason
                    self._log("    %s 对准状态 -> %s (targets=%d)", cls, last, self.vision.ntgt)
                stable = stable + 1 if self.vision.ready else 0
                if stable >= self.cfg.ready_frames:
                    self.vision.release()
                    ok = True
                    break
                self.flight.rate.sleep()
            if ok:
                end = rospy.Time.now() + rospy.Duration(10.0)
                while not rospy.is_shutdown() and rospy.Time.now() < end \
                        and not self.vision.servo_done:
                    self.flight.rate.sleep()
            done = bool(ok and self.vision.servo_done)
            self._log("    %s 第 %d 次 投递成功=%s 舵机=%s", cls, attempt, ok, self.vision.servo_done)
            self.vision.enable_detect(False)
            self.vision.set_mode("disabled")
            if done:
                return True
            self.flight.goto(x + self.cfg.drop_reset_offset,
                             y + self.cfg.drop_reset_offset,
                             self.cfg.drop_z, tol=0.45, timeout=60.0,
                             label=f"{cls}-reset{attempt}")
        return False

    def deliver(self) -> list:
        self._log("3/5 投递")
        targets = self.book.best(self.cfg.max_targets)
        start = (self.flight.x, self.flight.y)
        pts = order_by_travel([(t[1], t[2]) for t in targets], start)
        by_xy = {(round(t[1], 2), round(t[2], 2)): t[0] for t in targets}
        self.dropped = []
        for (x, y) in pts:
            cls = by_xy.get((round(x, 2), round(y, 2)), "?")
            self.dropped.append(self._deliver_one(cls, x, y))
        self._log("  投递结果 %s", self.dropped)
        return self.dropped

    # ---------- phase 4：走廊 ----------
    def corridor(self) -> bool:
        self._log("4/5 走廊（%d 段）", len(self.cfg.corridor))
        ok_all = True
        for i, (cx, cy, cz) in enumerate(self.cfg.corridor, 1):
            if not self.flight.goto(cx, cy, cz, tol=0.35, timeout=90.0, label=f"corridor-{i}"):
                ok_all = False
                break
        self.flight.goto(8.60, -3.60, 2.20, tol=0.40, timeout=90.0, label="corridor-lift")
        self.flight.goto(self.cfg.exit_pad[0], self.cfg.exit_pad[1], self.cfg.corridor_entry_z,
                         tol=0.45, timeout=120.0, label="corridor-end")
        return ok_all

    # ---------- phase 5：降落 ----------
    def land(self) -> None:
        self._log("5/5 走廊尽头 H 板降落")
        self.vision.enable_landing(True)
        self.vision.set_mode("landing")
        x, y = self.cfg.exit_pad
        self.flight.hold(x, y, self.cfg.corridor_entry_z, 5.0)
        for z in (1.20, 0.90, 0.60, 0.40):
            self.flight.goto(x, y, z, tol=0.30, timeout=45.0, label=f"descend-{z}")
        self.vision.set_mode("disabled")
        self.vision.enable_landing(False)
        self.flight.auto_land()
        end = rospy.Time.now() + rospy.Duration(90.0)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if self.flight.z < 0.15:
                break
            self.flight.rate.sleep()

    # ---------- 总流程 ----------
    def run(self) -> bool:
        if not self.takeoff():
            return False
        found = self.search()
        if found == 0:
            rospy.logwarn("[mission] 没搜到任何靶标，直接去走廊降落")
        else:
            self.deliver()
        self.corridor()
        self.land()
        self._log("完成：发现 %d，投递 %s，落点 (%.2f, %.2f, %.2f)",
                  found, self.dropped, self.flight.x, self.flight.y, self.flight.z)
        return all(self.dropped) if self.dropped else False
