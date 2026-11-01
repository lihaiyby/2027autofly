#!/usr/bin/env bash
set +u
REPO=/home/lihai/liftrace_sim/liftrace-visionwork
FW=/home/lihai/liftrace_sim/final_worlds/B_05cm_seed38_20261008
OUT=/home/lihai/liftrace_sim/out/round_$(date +%m%d_%H%M)
rm -rf "$OUT"; mkdir -p "$OUT"
cp -f /mnt/d/deepseek-workspace/liftrace_sim_work/300_mission_driver_v3.py /home/lihai/liftrace_sim/mission_driver.py
cp -f /mnt/d/deepseek-workspace/liftrace_sim_work/314_odom_to_tf.py /home/lihai/liftrace_sim/odom_to_tf.py
chmod +x /home/lihai/liftrace_sim/mission_driver.py

echo "== T0 开工自检 =="
bash /mnt/d/deepseek-workspace/liftrace_sim_work/313_preflight.sh > "$OUT/t0_preflight.log" 2>&1
tail -3 "$OUT/t0_preflight.log"

for n in gzserver gzclient rosmaster rosout roslaunch; do pkill -x "$n" 2>/dev/null; done
pkill -x px4 2>/dev/null; pkill -f fastlio 2>/dev/null; pkill -f livox_bridge 2>/dev/null
pkill -f odom_to_ 2>/dev/null; pkill -f camera_tf 2>/dev/null; pkill -f mock_servo 2>/dev/null
pkill -f target_detector 2>/dev/null; pkill -f detect_compat 2>/dev/null; pkill -f mission_driver 2>/dev/null
sleep 2
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1

echo "== T1 官方地图 + PX4 + MAVROS + 下视机架 $(date +%H:%M:%S) =="
roslaunch patrol_control patrol_world.launch \
  world:="$FW/worlds/field_with_targets.world" \
  gui:=true x:=0.0 y:=0.0 z:=0.06 px4_root:="$HOME/liftrace_sim/PX4-Autopilot" \
  astra_sim_lib:="$HOME/AstraDroneOpen/simulation/sim_workspace/devel/lib" \
  > "$OUT/t1_world.log" 2>&1 &
T1=$!
sleep 40

python3 /home/lihai/liftrace_sim/camera_tf.py > "$OUT/t2.log" 2>&1 &
python3 /home/lihai/liftrace_sim/livox_bridge.py > "$OUT/t3.log" 2>&1 &
python3 /home/lihai/liftrace_sim/mock_servo.py > "$OUT/servo.log" 2>&1 &
sleep 5
roslaunch fast_lio mapping_mid360_bridge.launch rviz:=false > "$OUT/t4.log" 2>&1 &
sleep 22
python3 /home/lihai/liftrace_sim/odom_to_tf.py > "$OUT/t5.log" 2>&1 &
python3 /home/lihai/liftrace_sim/odom_to_vision.py > "$OUT/t6.log" 2>&1 &
sleep 4

# 视觉链延迟启动，先保证解锁
( sleep 120; source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1; \
  roslaunch uav_vision phase_d.launch \
    image_topic:=/downward_camera/image_raw \
    camera_info_topic:=/downward_camera/camera_info \
    target_detector_launch_prefix:="$HOME/liftrace_sim/venv38/bin/python" \
    target_model_path:="$UAV_VISION_MODEL_PATH" target_detector_imgsz:=416 \
    map_frame:=camera_init ground_z:=0.0 enable_debug_image:=false \
    start_legacy_compat:=true allow_latest_tf_fallback:=true max_latest_tf_age_sec:=0.30 \
    > "$OUT/t7_vision.log" 2>&1 ) &
sleep 6

echo "== T8 整回合任务 $(date +%H:%M:%S) =="
python3 -u /home/lihai/liftrace_sim/mission_driver.py \
  "7.05,-4.13;4.64,-3.30;4.90,-0.71" 1.5 corridor > "$OUT/t8_driver.log" 2>&1 &
T8=$!

WID=$(DISPLAY=:0 xwininfo -root -tree 2>/dev/null | grep -oE '0x[0-9a-f]+ "Gazebo"' | head -1 | cut -d' ' -f1)
for i in $(seq 1 40); do
  sleep 20
  if ! kill -0 $T8 2>/dev/null; then
    echo "[watch] 任务驱动已结束，提前收尾"
    break
  fi
  if ! timeout 5 rostopic list > /dev/null 2>&1; then
    echo "[watchdog] rosmaster 失联，中止运行" | tee -a "$OUT/watchdog.log"
    break
  fi
  printf "[%3ds] " $((i*20)) >> "$OUT/pos.log"
  timeout 5 rostopic echo -n 1 /mavros/local_position/pose/pose/position 2>/dev/null | tr -d '\n ' >> "$OUT/pos.log"
  echo >> "$OUT/pos.log"
  if [ "$i" = "10" ] || [ "$i" = "18" ] || [ "$i" = "24" ]; then
    [ -n "$WID" ] && DISPLAY=:0 import -window "$WID" "$OUT/shot_$i.png" 2>/dev/null
  fi
done

echo "== 驱动日志 =="
cat "$OUT/t8_driver.log"
echo "== 轨迹尾部 =="
tail -10 "$OUT/pos.log"
echo "== 舵机 =="
tail -6 "$OUT/servo.log"
cp -f "$OUT"/shot_*.png /mnt/d/deepseek-workspace/liftrace_sim_work/out/ 2>/dev/null

kill -INT $T8 $T1 2>/dev/null; sleep 4
for n in gzserver gzclient rosmaster rosout; do pkill -x "$n" 2>/dev/null; done
pkill -x px4 2>/dev/null; pkill -f fastlio 2>/dev/null; pkill -f livox_bridge 2>/dev/null
pkill -f odom_to_ 2>/dev/null; pkill -f camera_tf 2>/dev/null; pkill -f mock_servo 2>/dev/null
pkill -f target_detector 2>/dev/null; pkill -f detect_compat 2>/dev/null; pkill -f mission_driver 2>/dev/null
echo FULL_ROUND_DONE
