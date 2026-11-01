#!/usr/bin/env bash
# 清理所有仿真相关进程（含 16 小时前挂掉的残留）
set +u
echo "== 清理前 =="
pgrep -a -x gzserver | head -3
pgrep -a -x px4 | head -3

for n in gzserver gzclient rosmaster rosout roslaunch; do
  pkill -x "$n" 2>/dev/null
done
pkill -x px4 2>/dev/null
pkill -f fastlio_mapping 2>/dev/null
pkill -f fast_lio 2>/dev/null
pkill -f livox_bridge 2>/dev/null
pkill -f odom_to_tf 2>/dev/null
pkill -f odom_to_vision 2>/dev/null
pkill -f camera_tf 2>/dev/null
pkill -f static_tf 2>/dev/null
pkill -f mock_servo 2>/dev/null
pkill -f mission_driver 2>/dev/null
pkill -f target_detector 2>/dev/null
pkill -f detect_compat_bridge 2>/dev/null
pkill -f phase_d 2>/dev/null
pkill -f "rostopic echo" 2>/dev/null
sleep 3

echo "== 清理后 =="
left=0
for n in gzserver gzclient rosmaster px4; do
  if pgrep -x "$n" > /dev/null 2>&1; then
    echo "  仍在运行: $n"
    left=1
  fi
done
[ "$left" = "0" ] && echo "  干净"
echo "== 端口 11311 =="
ss -lntp 2>/dev/null | grep 11311 || echo "  无监听（正常）"
echo CLEANUP_ALL_DONE
