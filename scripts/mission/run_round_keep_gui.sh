#!/usr/bin/env bash
# 起仿真 + 跑完整比赛航线，任务结束后【窗口保留】，Ctrl-C 才清理退出
# 想边看飞机飞边操作，用这个；只看不飞用 321_gui_keep.sh；跑完自动清理用 301_full_round.sh
set +u

REPO=/home/lihai/liftrace_sim/liftrace-visionwork
FW=/home/lihai/liftrace_sim/final_worlds/B_05cm_seed38_20261008
OUT=/home/lihai/liftrace_sim/out/fly_$(date +%m%d_%H%M)
mkdir -p "$OUT"

echo "== T0 开工自检 =="
bash /mnt/d/deepseek-workspace/liftrace_sim_work/312_cleanup_all.sh > /dev/null 2>&1
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1
echo "  DISPLAY=$DISPLAY  world=$(basename "$FW/worlds/field_with_targets.world")"

cp -f /mnt/d/deepseek-workspace/liftrace_sim_work/300_mission_driver_v3.py /home/lihai/liftrace_sim/mission_driver.py
cp -f /mnt/d/deepseek-workspace/liftrace_sim_work/314_odom_to_tf.py /home/lihai/liftrace_sim/odom_to_tf.py
cp -f /mnt/d/deepseek-workspace/liftrace_sim_work/274_odom_to_vision.py /home/lihai/liftrace_sim/odom_to_vision.py
chmod +x /home/lihai/liftrace_sim/mission_driver.py

cleanup() {
  echo
  echo "== 收到 Ctrl-C，清理仿真进程 =="
  kill -INT $T1 2>/dev/null
  sleep 3
  for n in gzserver gzclient rosmaster rosout roslaunch; do pkill -x "$n" 2>/dev/null; done
  pkill -x px4 2>/dev/null
  pkill -f fastlio_mapping 2>/dev/null
  pkill -f livox_bridge 2>/dev/null
  pkill -f odom_to_ 2>/dev/null
  pkill -f camera_tf 2>/dev/null
  pkill -f mock_servo 2>/dev/null
  pkill -f mission_driver 2>/dev/null
  pkill -f target_detector 2>/dev/null
  pkill -f detect_compat_bridge 2>/dev/null
  echo "已清理，日志在 $OUT"
  exit 0
}
trap cleanup INT TERM

echo "== T1 Gazebo(GUI) + PX4 + MAVROS + 下视机架 =="
roslaunch patrol_control patrol_world.launch \
  world:="$FW/worlds/field_with_targets.world" \
  gui:=true x:=0.0 y:=0.0 z:=0.06 px4_root:="$HOME/liftrace_sim/PX4-Autopilot" \
  astra_sim_lib:="$HOME/AstraDroneOpen/simulation/sim_workspace/devel/lib" \
  > "$OUT/t1_world.log" 2>&1 &
T1=$!
sleep 40
echo "  gzclient: $(pgrep -x gzclient > /dev/null && echo running || echo down)"

python3 /home/lihai/liftrace_sim/camera_tf.py > "$OUT/t2_camtf.log" 2>&1 &
python3 /home/lihai/liftrace_sim/livox_bridge.py > "$OUT/t3_bridge.log" 2>&1 &
python3 /home/lihai/liftrace_sim/mock_servo.py > "$OUT/t4_servo.log" 2>&1 &
sleep 5
roslaunch fast_lio mapping_mid360_bridge.launch rviz:=false > "$OUT/t5_lio.log" 2>&1 &
sleep 22
python3 /home/lihai/liftrace_sim/odom_to_tf.py > "$OUT/t6_tf.log" 2>&1 &
python3 /home/lihai/liftrace_sim/odom_to_vision.py > "$OUT/t7_visionpose.log" 2>&1 &
sleep 4

# 视觉链延后启动（否则解锁会被 EKF 航向检查卡住）
( sleep 120; source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1; \
  roslaunch uav_vision phase_d.launch \
    image_topic:=/downward_camera/image_raw \
    camera_info_topic:=/downward_camera/camera_info \
    target_detector_launch_prefix:="$HOME/liftrace_sim/venv38/bin/python" \
    target_model_path:="$UAV_VISION_MODEL_PATH" target_detector_imgsz:=416 \
    map_frame:=camera_init ground_z:=0.0 enable_debug_image:=false \
    start_legacy_compat:=true allow_latest_tf_fallback:=true max_latest_tf_age_sec:=0.30 \
    > "$OUT/t8_vision.log" 2>&1 ) &

echo
echo "======================================================"
echo " 现在开始飞：走廊 → 三靶投递 → H 视觉降落"
echo "   Gazebo 窗口里可以直接看；日志 $OUT/t9_driver.log"
echo "======================================================"
python3 -u /home/lihai/liftrace_sim/mission_driver.py \
  "7.05,-4.13;4.64,-3.30;4.90,-0.71" 1.5 corridor 2>&1 | tee "$OUT/t9_driver.log"

echo
echo "======================================================"
echo " 任务结束。仿真仍然开着，可以继续观察/手动操作"
echo "   想看飞行结果：tail -40 $OUT/t9_driver.log"
echo "   按 Ctrl-C 结束仿真并清理"
echo "======================================================"
while true; do sleep 5; done
