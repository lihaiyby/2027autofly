"""命令行入口：python -m autofly.cli [--config ...] [--search-z 2.4] [--drop-z 2.3] [--dry-run]"""
from __future__ import annotations

import argparse
import os
import sys

from .config import MissionConfig

DEFAULT_CFG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "config", "mission.yaml")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="autofly", description="搜索-投递-走廊-降落 任务")
    p.add_argument("--config", default=DEFAULT_CFG, help="mission.yaml 路径")
    p.add_argument("--search-z", type=float, default=None, help="搜索高度（默认取配置）")
    p.add_argument("--drop-z", type=float, default=None, help="投递对准高度")
    p.add_argument("--dry-run", action="store_true", help="只打印配置与航线，不启动 ROS")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    cfg = MissionConfig.from_yaml(args.config)
    if args.search_z is not None:
        cfg.search_z = args.search_z
    if args.drop_z is not None:
        cfg.drop_z = args.drop_z

    print("[cli] 配置:", cfg.summary())
    if args.dry_run:
        print("[cli] 搜索航线 %d 点:" % len(cfg.search_lanes), cfg.search_lanes)
        print("[cli] 走廊 %d 段:" % len(cfg.corridor), cfg.corridor)
        print("[cli] 投递目标类别:", cfg.want_classes)
        return 0

    import rospy
    from .mission import Mission

    rospy.init_node("autofly_mission")
    ok = Mission(cfg).run()
    print("[cli] 任务结果:", "SUCCESS" if ok else "INCOMPLETE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
