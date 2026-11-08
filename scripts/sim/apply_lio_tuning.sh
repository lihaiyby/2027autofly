#!/usr/bin/env bash
# 把 FAST-LIO 调优参数写入 launch（幂等，可重复执行）
# 依据：正方形巡航压力测试，偏差中位数 0.224 m -> 0.093 m
set -eu

L=/home/lihai/liftrace_sim/liftrace-visionwork/patrol_uav_ws-patrol_planner/src/FAST_LIO/launch/mapping_mid360_bridge.launch
[ -f "$L" ] || { echo "找不到 $L"; exit 1; }

python3 - "$L" <<'PY'
import io, re, sys
path = sys.argv[1]
cfg = {
    "max_iteration":    "6",      # 3 -> 6：更多迭代，收敛更稳
    "filter_size_surf": "0.25",   # 0.5 -> 0.25：保留更多表面特征
    "filter_size_map":  "0.25",   # 0.5 -> 0.25：地图更精细
    "point_filter_num": "2",      # 3 -> 2：用更多点参与匹配
}
t = io.open(path, encoding="utf-8").read()
changed = []
for k, v in cfg.items():
    t2, n = re.subn(r'(<param name="%s" type="\w+" value=")[^"]*(")' % k,
                    r"\g<1>%s\g<2>" % v, t)
    if n and t2 != t:
        changed.append("%s -> %s" % (k, v))
    t = t2
io.open(path, "w", encoding="utf-8").write(t)
print("已应用:", ", ".join(changed) if changed else "（本来就是调优值）")
PY

grep -E 'max_iteration|filter_size_surf|filter_size_map|point_filter_num' "$L"
