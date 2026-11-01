#!/usr/bin/env bash
# 开工前自检：清残留、查内存、查世界文件、确认 ROS 可启动
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FAIL=0

echo "== 1. 清理残留进程 =="
bash "$ROOT/scripts/sim/cleanup_all.sh" 2>&1 | tail -3

echo "== 2. 内存 =="
free -m | awk 'NR==2{printf "  total=%dMB used=%dMB available=%dMB\n", $2, $3, $7}'
AVAIL=$(free -m | awk 'NR==2{print $7}')
if [ "${AVAIL:-0}" -lt 2000 ]; then
  echo "  [警告] 可用内存 < 2 GB，视觉链可能把 rosmaster 拖死"
  FAIL=1
fi

echo "== 3. 世界文件 =="
W=/home/lihai/liftrace_sim/final_worlds/B_05cm_seed38_20261008/worlds/field_with_targets.world
if [ -f "$W" ]; then echo "  OK $W"; else echo "  [错误] 缺少 $W"; FAIL=1; fi

echo "== 4. 关键可执行文件 =="
for f in /home/lihai/liftrace_sim/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
         /home/lihai/liftrace_sim/mission_driver.py \
         /home/lihai/liftrace_sim/livox_bridge.py; do
  [ -e "$f" ] && echo "  OK $f" || { echo "  [错误] 缺少 $f"; FAIL=1; }
done

echo "== 5. ROS 环境 =="
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1
command -v roslaunch > /dev/null && echo "  OK roslaunch $(rosversion -d 2>/dev/null)" || { echo "  [错误] ROS 不可用"; FAIL=1; }

echo "== 6. 提示 =="
echo "  · 长时间不跑请先 Ctrl-C 结束脚本，避免整夜挂着被系统挂起后残留僵尸进程"
echo "  · 出现 Connection refused / GST_IS_APP_SRC 说明 rosmaster 已死，先跑 312_cleanup_all.sh"

if [ "$FAIL" = "0" ]; then
  echo "PREFLIGHT OK"
else
  echo "PREFLIGHT FAILED"
fi
