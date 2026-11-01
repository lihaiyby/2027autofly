#!/usr/bin/env bash
# 建图观察：headless 仿真 + 激光桥接 + FAST-LIO + RViz
# 用法：
#   bash 333_map_view.sh          # 只让飞机悬停，看局部点云
#   bash 333_map_view.sh fly      # 起飞绕方形，让点云铺开（推荐）
# 结束：Ctrl-C（会清理所有进程）
set +u

MODE=${1:-hover}
OUT=/home/lihai/liftrace_sim/out/mapview_$(date +%H%M)
mkdir -p "$OUT"

for n in gzserver gzclient rosmaster rosout roslaunch rviz; do pkill -x "$n" 2>/dev/null; done
pkill -x px4 2>/dev/null; pkill -f fastlio_mapping 2>/dev/null; pkill -f livox_bridge 2>/dev/null
pkill -f odom_to_ 2>/dev/null; pkill -f takeoff_demo 2>/dev/null
sleep 2
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1

cleanup() {
  echo
  echo "== 清理 =="
  kill -INT $T1 2>/dev/null
  sleep 3
  for n in gzserver gzclient rosmaster rosout roslaunch rviz; do pkill -x "$n" 2>/dev/null; done
  pkill -x px4 2>/dev/null
  pkill -f fastlio_mapping 2>/dev/null
  pkill -f livox_bridge 2>/dev/null
  pkill -f takeoff_demo 2>/dev/null
  echo "已清理，日志 $OUT"
  exit 0
}
trap cleanup INT TERM

echo "== 1/4 仿真（headless，把 GPU 留给 rviz）=="
roslaunch patrol_control patrol_world.launch \
  world:="$WORLD" gui:=false x:=0.0 y:=0.0 z:=0.06 > "$OUT/t1_world.log" 2>&1 &
T1=$!
sleep 35

echo "== 2/4 激光桥接 =="
python3 /home/lihai/liftrace_sim/livox_bridge.py > "$OUT/t2_bridge.log" 2>&1 &
sleep 5

echo "== 3/4 FAST-LIO + RViz =="
roslaunch fast_lio mapping_mid360_bridge.launch rviz:=true > "$OUT/t3_lio.log" 2>&1 &
sleep 20
pgrep -x rviz > /dev/null && echo "   RViz 已启动" || echo "   [警告] RViz 没起来，看 $OUT/t3_lio.log"

if [ "$MODE" = "fly" ]; then
  echo "== 4/4 起飞绕方形，让点云铺开 =="
  python3 /home/lihai/liftrace_sim/takeoff_demo.py > "$OUT/t4_demo.log" 2>&1 &
else
  echo "== 4/4 保持悬停（不飞），点云只覆盖机体附近 =="
fi

echo
echo "======================================================"
echo " RViz 窗口里能看到：绿色点云（地板/墙/天花板）、"
echo " 黑色空洞（树箱、墙体）、底部橙红是 H 板附近"
echo "   · 鼠标左键拖动 = 旋转，滚轮 = 缩放，中键 = 平移"
echo "   · 左侧 Displays 面板可调点云大小/颜色"
echo "   · 按 Ctrl-C 结束并清理"
echo "======================================================"
while true; do sleep 5; done
