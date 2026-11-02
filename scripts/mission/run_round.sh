#!/usr/bin/env bash
# 新版比赛回合：随机场景 -> 起飞 -> 三靶投递 -> 走廊(北口进) -> 走廊尽头 H 板降落
# 用法: bash 365_round_v2.sh [seed] [targets]
#   seed    默认 42
#   targets 默认 panzer,pillbox,tent
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

GUI=${GUI:-true}
VEHICLE=${VEHICLE:-iris_mid360}
MEM_MAP=${MEM_MAP:-true}
SEED=${1:-42}
WANT=${2:-panzer,pillbox,tent}
FW=/home/lihai/liftrace_sim/final_worlds/B_05cm_seed38_20261008
SCENES=/home/lihai/liftrace_sim/scenes
OUT=/home/lihai/liftrace_sim/out/round2_$(date +%m%d_%H%M)
mkdir -p "$OUT"

echo "== T0 场景生成 (seed=$SEED) =="
/home/lihai/liftrace_sim/venv38/bin/python \
  "$ROOT/scripts/tools/scene_gen.py" --seed "$SEED" --out "$SCENES" \
  | tee "$OUT/scene.log"
WORLD="$SCENES/scene_$SEED.world"
META="$SCENES/scene_$SEED.json"
[ -f "$WORLD" ] || { echo "场景生成失败"; exit 1; }

# 从 JSON 里取目标坐标（顺序按 WANT）
TARGETS=$(/usr/bin/python3 - "$META" "$WANT" <<'PY'
import json, sys
meta = json.load(open(sys.argv[1]))
want = [w.strip() for w in sys.argv[2].split(",") if w.strip()]
pos = {t["name"]: (t["x"], t["y"]) for t in meta["targets"]}
sel = [w for w in want if w in pos] or list(pos)[:3]
print(";".join("%.2f,%.2f" % pos[n] for n in sel))
PY
)
echo "  投递目标: $TARGETS"
[ -n "$TARGETS" ] || { echo "取目标坐标失败"; exit 1; }

echo "== T1 自检 + 清理 =="
bash "$ROOT/scripts/sim/cleanup_all.sh" > /dev/null 2>&1
source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1

cp -f "$ROOT/scripts/sim/odom_to_tf.py" /home/lihai/liftrace_sim/odom_to_tf.py
cp -f "$ROOT/scripts/sim/odom_to_vision.py" /home/lihai/liftrace_sim/odom_to_vision.py

echo "== T2 仿真（新场景，GUI）$(date +%H:%M:%S) =="
roslaunch patrol_control patrol_world.launch \
  world:="$WORLD" gui:=$GUI x:=0.0 y:=0.0 z:=0.06 vehicle:=$VEHICLE > "$OUT/t1_world.log" 2>&1 &
T1=$!
sleep 40

python3 /home/lihai/liftrace_sim/camera_tf.py > "$OUT/t2_camtf.log" 2>&1 &
python3 /home/lihai/liftrace_sim/livox_bridge.py > "$OUT/t3_bridge.log" 2>&1 &
python3 /home/lihai/liftrace_sim/mock_servo.py > "$OUT/t4_servo.log" 2>&1 &
sleep 5
roslaunch fast_lio mapping_mid360_bridge.launch rviz:=false > "$OUT/t5_lio.log" 2>&1 &
sleep 22
python3 /home/lihai/liftrace_sim/odom_to_tf.py > "$OUT/t6_tf.log" 2>&1 &
python3 /home/lihai/liftrace_sim/odom_to_vision.py > "$OUT/t7_visionpose.log" 2>&1 &
sleep 4

( source /home/lihai/liftrace_sim/sim_env.sh > /dev/null 2>&1; \
  roslaunch uav_vision phase_d.launch \
    image_topic:=/downward_camera/image_raw \
    camera_info_topic:=/downward_camera/camera_info \
    target_detector_launch_prefix:="$HOME/liftrace_sim/venv38/bin/python" \
    target_model_path:="$UAV_VISION_MODEL_PATH" target_detector_imgsz:=416 \
    map_frame:=camera_init ground_z:=0.0 enable_debug_image:=false \
    start_legacy_compat:=true allow_latest_tf_fallback:=true max_latest_tf_age_sec:=2.0 \
    require_map_for_candidates:=$MEM_MAP \
    > "$OUT/t8_vision.log" 2>&1 ) &


sleep 20

echo "== T3 任务：投递 -> 走廊 -> 尽头降落 $(date +%H:%M:%S) =="
PYTHONPATH="$ROOT/src:$PYTHONPATH" python3 -u "$ROOT/scripts/mission/mission_driver.py" 2.4 2.3 \
  > "$OUT/t9_driver.log" 2>&1 &
T9=$!

WID=$(DISPLAY=:0 xwininfo -root -tree 2>/dev/null | grep -oE '0x[0-9a-f]+ "Gazebo"' | head -1 | cut -d' ' -f1)
for i in $(seq 1 90); do
  sleep 20
  if ! kill -0 $T9 2>/dev/null; then echo "  驱动结束"; break; fi
  if ! timeout 5 rostopic list > /dev/null 2>&1; then
    echo "[watchdog] rosmaster 失联"; break
  fi
  printf "[%3ds] " $((i*20)) >> "$OUT/pos.log"
  timeout 5 rostopic echo -n 1 /mavros/local_position/pose/pose/position 2>/dev/null | tr -d '\n ' >> "$OUT/pos.log"
  echo >> "$OUT/pos.log"
  if [ "$i" = "8" ] || [ "$i" = "16" ] || [ "$i" = "24" ]; then
    [ -n "$WID" ] && DISPLAY=:0 import -window "$WID" "$OUT/shot_$i.png" 2>/dev/null
  fi
done

echo "== 驱动日志 =="
cat "$OUT/t9_driver.log"
echo "== 舵机 =="
tail -4 "$OUT/t4_servo.log"
mkdir -p "$ROOT/docs/screenshots"; cp -f "$OUT"/shot_*.png "$ROOT/docs/screenshots/" 2>/dev/null || true

kill -INT $T1 2>/dev/null; sleep 4
for n in gzserver gzclient rosmaster rosout roslaunch; do pkill -x "$n" 2>/dev/null; done
pkill -x px4 2>/dev/null; pkill -f fastlio_mapping 2>/dev/null; pkill -f livox_bridge 2>/dev/null
pkill -f odom_to_ 2>/dev/null; pkill -f camera_tf 2>/dev/null; pkill -f mock_servo 2>/dev/null
pkill -f target_detector 2>/dev/null; pkill -f detect_compat_bridge 2>/dev/null; pkill -f mission_driver 2>/dev/null
echo ROUND2_DONE
