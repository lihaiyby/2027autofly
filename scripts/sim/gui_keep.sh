#!/usr/bin/env bash
# 只启动仿真并在前台保持：Gazebo 窗口一直开着，Ctrl-C 才退出
# 用法: bash 321_gui_keep.sh [world路径]
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

WORLD=${1:-/home/lihai/liftrace_sim/final_worlds/B_05cm_seed38_20261008/worlds/field_with_targets.world}
OUT=/home/lihai/liftrace_sim/out/gui_keep
mkdir -p "$OUT"

echo "== 自检 =="
bash "$ROOT/scripts/sim/cleanup_all.sh" > /dev/null 2>&1
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1
echo "  DISPLAY=$DISPLAY"
timeout 5 xdpyinfo -display "$DISPLAY" > /dev/null 2>&1 && echo "  X11 OK" || echo "  [警告] X11 不可用，窗口不会出现"
echo "  world=$WORLD"

echo "== 启动 Gazebo(GUI) + PX4 + MAVROS =="
roslaunch patrol_control patrol_world.launch \
  world:="$WORLD" \
  gui:=true x:=0.0 y:=0.0 z:=0.06 \
  px4_root:="$HOME/liftrace_sim/PX4-Autopilot" \
  astra_sim_lib:="$HOME/AstraDroneOpen/simulation/sim_workspace/devel/lib" \
  > "$OUT/world.log" 2>&1 &
LAUNCH_PID=$!

trap 'echo; echo "== 收到 Ctrl-C，清理进程 =="; kill -INT $LAUNCH_PID 2>/dev/null; sleep 3; \
      for n in gzserver gzclient rosmaster rosout roslaunch; do pkill -x "$n" 2>/dev/null; done; \
      pkill -x px4 2>/dev/null; echo "已清理"; exit 0' INT TERM

echo "  等待 Gazebo 窗口出现（约 25–35 秒）…"
for i in $(seq 1 20); do
  sleep 3
  if pgrep -x gzclient > /dev/null 2>&1; then
    echo "  [OK] gzclient 已启动（第 $((i*3)) 秒）"
    break
  fi
done

# 打印模型清单，确认地图加载完整
sleep 8
MODELS=$(timeout 8 rosservice call /gazebo/get_world_properties 2>/dev/null | grep -cE '^  - ')
echo "  Gazebo 内模型数: ${MODELS:-未知}"
echo
echo "======================================================"
echo " Gazebo 窗口已打开，脚本会一直挂着"
echo "   · 鼠标右键拖拽 = 旋转视角，滚轮 = 缩放，中键 = 平移"
echo "   · 想看飞机飞：另开终端跑"
echo "       python3 ~/liftrace_sim/mission_driver.py \"7.05,-4.13\" 1.5"
echo "   · 结束仿真：在本终端按 Ctrl-C"
echo "======================================================"

# 保持前台，直到 Ctrl-C
while kill -0 $LAUNCH_PID 2>/dev/null; do
  sleep 5
done
echo "roslaunch 已退出"
