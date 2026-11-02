"""任务配置：YAML -> dataclass，带默认值。不依赖 ROS。"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

try:  # 允许在没有 PyYAML 的环境里 import 本模块做纯逻辑测试
    import yaml
except Exception:  # pragma: no cover
    yaml = None


DEFAULT_CORRIDOR: List[Tuple[float, float, float]] = [
    (8.70, 4.30, 1.20),
    (9.10, 3.60, 1.10),
    (9.15, 2.80, 1.00),
    (8.80, 2.20, 1.00),
    (8.45, 1.60, 1.00),      # 门 A：Wall_20 覆盖 x 8.8~9.5，只能走 x<8.8
    (8.45, 0.90, 1.00),
    (8.45, 0.20, 1.00),
    (8.80, -0.70, 1.00),
    (9.15, -1.60, 1.00),     # 门 B：Wall_22 覆盖 x 8.0~8.7，只能走 x>8.7
    (8.85, -2.50, 1.00),
    (8.60, -3.40, 1.00),
]

DEFAULT_SEARCH_LANES: List[Tuple[float, float]] = [
    (0.8, -4.0), (0.8, 4.0),
    (2.6, 4.0), (2.6, -4.0),
    (4.4, -4.0), (4.4, 4.0),
    (6.2, 4.0), (6.2, -4.0),
    (7.2, -4.0),
]


@dataclass
class MissionConfig:
    """一次任务的全部可调参数。"""

    # 高度
    takeoff_z: float = 1.5
    search_z: float = 2.4
    drop_z: float = 2.3
    corridor_entry_z: float = 1.6

    # 速度与限幅
    max_xy_speed: float = 0.6
    max_jerk: float = 2.0

    # 目标
    want_classes: Tuple[str, ...] = ("tent", "pillbox", "bridge", "panzer")
    max_targets: int = 3

    # 航线
    search_lanes: List[Tuple[float, float]] = field(default_factory=lambda: list(DEFAULT_SEARCH_LANES))
    corridor: List[Tuple[float, float, float]] = field(default_factory=lambda: list(DEFAULT_CORRIDOR))
    exit_pad: Tuple[float, float] = (8.75, -4.20)

    # 时限（秒）
    search_timeout: float = 420.0
    align_timeout: float = 150.0
    vision_wait: float = 150.0

    # 重试
    takeoff_retries: int = 3
    drop_retries: int = 3
    drop_reset_offset: float = 0.8

    # 逐帧投影只接受圆环靠近画面中心的检测（像素半径门限）
    center_gate_px: float = 150.0

    # 地图投影常量补偿（标定得到：报出值 - 真值 = (-0.13, +0.16)）
    map_offset: Tuple[float, float] = (0.13, -0.16)

    # 验收
    goto_tol: float = 0.40
    lane_tol: float = 0.55
    ready_frames: int = 3

    @classmethod
    def from_yaml(cls, path: str) -> "MissionConfig":
        """从 YAML 载入；文件不存在或 PyYAML 缺失时返回默认配置。"""
        if not path or not os.path.exists(path) or yaml is None:
            return cls()
        raw = yaml.safe_load(open(path, encoding="utf-8")) or {}
        cfg = cls()
        for key, value in raw.items():
            if not hasattr(cfg, key):
                continue
            if key == "want_classes":
                value = tuple(str(v) for v in value)
            elif key in ("search_lanes", "corridor"):
                value = [tuple(float(x) for x in v) for v in value]
            elif key == "map_offset":
                value = tuple(float(x) for x in value)
            elif key == "exit_pad":
                value = tuple(float(x) for x in value)      # 扁平 [x, y]
            setattr(cfg, key, value)
        return cfg

    def summary(self) -> str:
        return (
            "MissionConfig(search_z=%.2f drop_z=%.2f speed=%.2f "
            "lanes=%d corridor=%d targets=%d)"
            % (self.search_z, self.drop_z, self.max_xy_speed,
               len(self.search_lanes), len(self.corridor), self.max_targets)
        )
