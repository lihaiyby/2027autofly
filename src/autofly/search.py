"""搜索与目标簿记：纯逻辑，不依赖 ROS，可单测。"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Target = Tuple[str, float, float, float]   # (class, x, y, conf)


def lawnmower(lanes: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """割草机航线：按给定端点顺序返回，去除相邻重复点。"""
    out: List[Tuple[float, float]] = []
    for p in lanes:
        x, y = float(p[0]), float(p[1])
        if out and abs(out[-1][0] - x) < 1e-6 and abs(out[-1][1] - y) < 1e-6:
            continue
        out.append((x, y))
    return out


def lane_spacing(lanes: Sequence[Tuple[float, float]]) -> float:
    """相邻两条竖直线之间的 x 间距（用于自检是否覆盖足够）。"""
    xs = sorted({round(float(p[0]), 3) for p in lanes})
    if len(xs) < 2:
        return 0.0
    return max(b - a for a, b in zip(xs, xs[1:]))


class TargetBook:
    """记录搜索中发现的靶标：按类别去重、拒绝野值、按置信度取前 N。"""

    def __init__(self, want_classes: Iterable[str],
                 max_abs: float = 12.0, min_conf: float = 0.0):
        self.want = tuple(want_classes)
        self.max_abs = float(max_abs)
        self.min_conf = float(min_conf)
        self._by_class: Dict[str, Target] = {}
        self.seen = 0

    def add(self, class_name: str, x: float, y: float, conf: float = 0.0) -> bool:
        """返回是否更新了簿记。"""
        self.seen += 1
        if class_name not in self.want:
            return False
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        if abs(x) > self.max_abs or abs(y) > self.max_abs:
            return False
        if conf < self.min_conf:
            return False
        old = self._by_class.get(class_name)
        if old is not None and conf < old[3]:
            return False
        self._by_class[class_name] = (class_name, float(x), float(y), float(conf))
        return True

    def add_many(self, items: Iterable[Target]) -> int:
        n = 0
        for cls, x, y, conf in items:
            if self.add(cls, x, y, conf):
                n += 1
        return n

    def best(self, n: int = 3) -> List[Target]:
        """按置信度从高到低取前 n 个（同一类别只出现一次）。"""
        return sorted(self._by_class.values(), key=lambda t: -t[3])[:n]

    def get(self, class_name: str) -> Optional[Target]:
        return self._by_class.get(class_name)

    def __len__(self) -> int:
        return len(self._by_class)

    def __contains__(self, class_name: str) -> bool:
        return class_name in self._by_class

    def as_dict(self) -> Dict[str, Tuple[float, float]]:
        return {k: (round(v[1], 2), round(v[2], 2)) for k, v in self._by_class.items()}


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest(points: Sequence[Tuple[float, float]],
            origin: Tuple[float, float]) -> Optional[Tuple[float, float]]:
    """按距离排序目标，用于规划投递顺序（最近的先投）。"""
    if not points:
        return None
    return min(points, key=lambda p: distance(p, origin))


def order_by_travel(points: Sequence[Tuple[float, float]],
                    start: Tuple[float, float]) -> List[Tuple[float, float]]:
    """贪心最近邻排序：从 start 出发依次取最近点。"""
    remaining = list(points)
    out: List[Tuple[float, float]] = []
    cur = start
    while remaining:
        nxt = nearest(remaining, cur)
        out.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    return out
