#!/usr/bin/env bash
# 验证文档里"终端 1"的命令（不带额外参数，靠 sim_env.sh 的 PX4_ROOT 等）能否起仿真
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT=/home/lihai/liftrace_sim/out/doc_check
mkdir -p "$OUT"
for n in gzserver gzclient rosmaster rosout roslaunch; do pkill -x "$n" 2>/dev/null; done
pkill -x px4 2>/dev/null; sleep 2
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1

roslaunch patrol_control patrol_world.launch \
  world:=$WORLD gui:=true x:=0.0 y:=0.0 z:=0.06 > "$OUT/t1.log" 2>&1 &
PID=$!
sleep 40

echo "== gzclient =="
pgrep -x gzclient > /dev/null && echo "  running" || echo "  DOWN"
echo "== 模型数 =="
timeout 8 rosservice call /gazebo/get_world_properties 2>/dev/null | grep -cE '^  - '
echo "== mavros =="
timeout 8 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -E "connected|mode"
echo "== 日志里的错误 =="
grep -icE "error|failed" "$OUT/t1.log" 2>/dev/null

kill -INT $PID 2>/dev/null; sleep 4
for n in gzserver gzclient rosmaster rosout; do pkill -x "$n" 2>/dev/null; done
pkill -x px4 2>/dev/null
echo DOC_CHECK_DONE
