#!/usr/bin/env python3
"""场景生成器：随机摆放 5 个靶标 + 4 棵树，输出新世界文件与元数据。

用法:
  scene_gen.py --seed 42 [--out ~/liftrace_sim/scenes]
输出:
  <out>/scene_<seed>.world   —— 可直接用 world:= 传给 launch
  <out>/scene_<seed>.json    —— 靶标/树/H 板坐标，供任务驱动读取

约束（可调常量）:
  · 场地内 0.6 m 边距，避开东侧走廊（x > CORRIDOR_X_MIN）
  · 避开两块 H 板（起飞点 (0,0) 与走廊尽头 (8.75,-4.2)）
  · 靶标之间 ≥ 1.3 m，靶标与树 ≥ 1.0 m，树之间 ≥ 1.5 m
"""
import argparse
import json
import math
import os
import random
import re

SRC = ("/home/lihai/liftrace_sim/final_worlds/"
       "B_05cm_seed38_20261008/worlds/field_with_targets.world")

FIELD = {"x_min": 0.6, "x_max": 7.4, "y_min": -4.4, "y_max": 4.4}
CORRIDOR_X_MIN = 7.6                      # 东侧走廊，禁放
H_PADS = [(0.0, 0.0, 1.2), (8.75, -4.2, 0.9)]

TARGETS = [                               # include 名 → 显示名
    ("random_tent", "tent"),
    ("random_pillbox", "pillbox"),
    ("random_bridge", "bridge"),
    ("random_panzer", "panzer"),
    ("random_red_cross", "red_cross"),
]
TREES = ["juniper_Tree_0", "juniper_Tree_1", "juniper_Tree_2", "toudi2"]


def free_of(x, y, placed, min_d):
    for px, py, _ in placed:
        if math.hypot(x - px, y - py) < min_d:
            return False
    for hx, hy, hr in H_PADS:
        if math.hypot(x - hx, y - hy) < hr:
            return False
    return FIELD["x_min"] <= x <= FIELD["x_max"] and FIELD["y_min"] <= y <= FIELD["y_max"]


def sample(rng, placed, min_d, tries=4000):
    for _ in range(tries):
        x = rng.uniform(FIELD["x_min"], FIELD["x_max"])
        y = rng.uniform(FIELD["y_min"], FIELD["y_max"])
        if x > CORRIDOR_X_MIN:
            continue
        if free_of(x, y, placed, min_d):
            return x, y
    raise RuntimeError("采样失败：约束太紧")


def replace_model_pose(txt, model_name, pose):
    i = txt.find('<model name="%s">' % model_name)
    if i < 0:
        raise RuntimeError("找不到模型 " + model_name)
    j = txt.find("</model>", i)
    body = txt[i:j]
    poses = list(re.finditer(r"<pose>[^<]*</pose>", body))
    if not poses:
        raise RuntimeError("模型里没有 pose: " + model_name)
    last = poses[-1]                      # 模型级 pose 在最后
    body2 = body[:last.start()] + "<pose>%s</pose>" % pose + body[last.end():]
    return txt[:i] + body2 + txt[j:]


def replace_include_pose(txt, name, pose):
    pat = re.compile(r"(<name>%s</name>\s*)<pose>[^<]*</pose>" % re.escape(name))
    new, n = pat.subn(r"\g<1><pose>%s</pose>" % pose, txt, count=1)
    if n != 1:
        raise RuntimeError("找不到 include: " + name)
    return new


def fmt(x, y, z, yaw):
    return "%.6f %.6f %.3f 0 0 %.6f" % (x, y, z, yaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default=os.path.expanduser("~/liftrace_sim/scenes"))
    ap.add_argument("--src", default=SRC)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    txt = open(args.src, encoding="utf-8").read()

    placed = []
    meta = {"seed": args.seed, "world": None,
            "targets": [], "trees": [], "h_pads": [{"name": "landing_h", "x": 0.0, "y": 0.0},
                                                   {"name": "landing_h_clone", "x": 8.75, "y": -4.2}],
            "corridor": {"entrance": [8.70, 4.30], "exit": [8.75, -4.20]}}

    # 树先放：要在靶标之间留出空间
    for t in TREES:
        x, y = sample(rng, placed, 1.5)
        yaw = rng.uniform(-math.pi, math.pi)
        txt = replace_model_pose(txt, t, fmt(x, y, 0.3, yaw))
        placed.append((x, y, t))
        meta["trees"].append({"model": t, "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)})

    # 靶标
    for inc_name, label in TARGETS:
        x, y = sample(rng, placed, 1.3)
        yaw = rng.uniform(-math.pi, math.pi)
        txt = replace_include_pose(txt, inc_name, fmt(x, y, 0.02, yaw))
        placed.append((x, y, label))
        meta["targets"].append({"name": label, "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)})

    os.makedirs(args.out, exist_ok=True)
    world = os.path.join(args.out, "scene_%d.world" % args.seed)
    metaj = os.path.join(args.out, "scene_%d.json" % args.seed)
    open(world, "w", encoding="utf-8").write(txt)
    meta["world"] = world
    json.dump(meta, open(metaj, "w", encoding="utf-8"), ensure_ascii=False, indent=2)

    print("world :", world)
    print("meta  :", metaj)
    print("靶标  :")
    for t in meta["targets"]:
        print("   %-10s (%6.2f, %6.2f) yaw=%6.2f" % (t["name"], t["x"], t["y"], t["yaw"]))
    print("树    :")
    for t in meta["trees"]:
        print("   %-16s (%6.2f, %6.2f)" % (t["model"], t["x"], t["y"]))


if __name__ == "__main__":
    main()
