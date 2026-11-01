# 手动启动仿真流程

> 全程在 **WSL `Ubuntu-20.04`** 里操作。会话用错发行版会找不到 ROS（默认 `wsl` 进的是 26.04）。
> 打开方式：桌面双击 **`WSL 仿真终端`**，或命令行 `wsl -d Ubuntu-20.04 --cd /home/lihai/liftrace_sim`。

---

## 0. 先选场景

| 你想干什么 | 用哪个 | 需要开几个终端 |
|---|---|---|
| 只想看 Gazebo 里的场地 | `321_gui_keep.sh` | 1 |
| 只想看 RViz 建图点云（不开 Gazebo 窗口） | `333_map_view.sh fly` | 1 |
| 想让它飞完整套（走廊→三靶投递→H 降落） | `322_gui_fly.sh` | 1 |
| 飞完整套、跑完自动清理 | `301_full_round.sh` | 1 |
| **想一步步手动控制 / 排查问题** | 本文第 2 节 | 5 |

> 手动流程里"不开 Gazebo 窗口"见终端 1，"想看 RViz"见终端 3——都在对应步骤就地说明。

所有脚本都有两个前置习惯，建议照做：

```bash
bash /mnt/d/deepseek-workspace/liftrace_sim_work/312_cleanup_all.sh   # 清残留（很关键）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/313_preflight.sh     # 自检
```

---

## 1. 一条命令的四种用法

```bash
# A. 只看 Gazebo 画面（不飞，窗口一直开着，Ctrl-C 退出）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/321_gui_keep.sh

# B. 只看建图点云（headless 仿真 + RViz + 起飞绕方形，Ctrl-C 退出）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/333_map_view.sh fly

# C. 飞完整套，结束后窗口保留（推荐看效果）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/322_gui_fly.sh

# D. 飞完整套，结束后自动清理
bash /mnt/d/deepseek-workspace/liftrace_sim_work/301_full_round.sh
```

C/D 的飞行序列：起飞 → 走廊蛇形 6 段 → 爬升越墙 → 三靶连投 → 返航 → H 视觉降落。
B 不带视觉链，只跑 LIO 建图。

---

## 2. 手动分终端版（5 个终端）

**按下面的顺序开，前一个看到成功标志再开下一个。**
每个终端的第一行都是 `source ~/liftrace_sim/sim_env.sh`（下文每段都已写全）。

### 终端 1 —— 仿真世界 + 飞控 + MAVROS

```bash
source ~/liftrace_sim/sim_env.sh
roslaunch patrol_control patrol_world.launch \
  world:=$WORLD gui:=true x:=0.0 y:=0.0 z:=0.06
#                        ^^^^^^^^^
#   不想开 Gazebo 窗口（把 GPU 留给 RViz）就改成 gui:=false
```

| | |
|---|---|
| 等多久 | 30–45 秒 |
| 成功标志 | ① 弹出 Gazebo 窗口 ② `rostopic echo -n1 /mavros/state` 显示 `connected: True` |
| 正常噪声 | 日志里的 `ERROR [param] not enough arguments`、`LED: open /dev/led0 failed` 是 PX4 SITL 的固有输出，**可以忽略** |

> **仿真已经在跑、想中途关掉窗口**：`pkill -x gzclient`
> （实测点云仍 9.975 Hz、`/mavros/state` 仍 `connected: True`、`/clock` 继续走）；
> 想开回来 `rosrun gazebo_ros gzclient`。

> **视角**：左键拖动旋转、滚轮缩放、中键平移。默认视角对着墙角，
> 想确认地图完整可看左侧 `World → Models`（应有 ground_plane、toudi2、landing_h、
> landing_h_clone、indoor_context、四个靶、red_cross、iris_mid360）。

### 终端 2 —— 激光桥接（必须有）

```bash
source ~/liftrace_sim/sim_env.sh
python3 ~/liftrace_sim/livox_bridge.py
```

| | |
|---|---|
| 为什么需要 | 仿真发 `livox_laser_simulation/CustomMsg`，FAST-LIO 要 `livox_ros_driver2/CustomMsg` |
| 成功标志 | `rostopic hz /livox/lidar_r2` → **~10 Hz** |

### 终端 3 —— FAST-LIO + 三个坐标变换

一个终端里起四个进程（前 20 秒等 LIO 初始化）：

```bash
source ~/liftrace_sim/sim_env.sh
roslaunch fast_lio mapping_mid360_bridge.launch rviz:=false &
#                                              ^^^^^^^^^^^^
#   想看建图点云就改成 rviz:=true（自动加载 loam_livox.rviz）
sleep 20
python3 ~/liftrace_sim/odom_to_tf.py &        # /Odometry → TF camera_init→body
python3 ~/liftrace_sim/odom_to_vision.py &    # /Odometry → /mavros/vision_pose/pose（PX4 靠它定位）
python3 ~/liftrace_sim/camera_tf.py &         # TF body→downward_camera_optical_frame
wait
```

| | |
|---|---|
| 成功标志 | `rostopic hz /Odometry` → **~10 Hz**；`rostopic hz /mavros/vision_pose/pose` → **~10 Hz** |
| 备用检查 | `rosrun tf tf_echo camera_init downward_camera_optical_frame` 有 Translation |

> **RViz 补充**：如果 LIO 已经在跑、当时没加 `rviz:=true`，可以单独补一个窗口：
>
> ```bash
> source ~/liftrace_sim/sim_env.sh
> rviz -d $(rospack find fast_lio)/rviz_cfg/loam_livox.rviz
> ```
>
> 操作：左键旋转、滚轮缩放、中键平移；左侧 `Displays` 可调点云大小。
> 点云中绿色是墙/地板/天花板，黑色空洞是树箱和墙体，底部橙红是 H 板附近；
> 固定坐标系 `camera_init`（与 FAST-LIO 世界系一致）。实测 WSLg 下 9 fps、`/cloud_registered` 9.95 Hz。
>
> 也可以直接跑 `333_map_view.sh fly`（headless 仿真 + LIO + RViz + 起飞绕方形，一条命令）。
>
> ![RViz 建图结果](out/rviz_map.png)

### 终端 4 —— 任务驱动（让它飞）

```bash
source ~/liftrace_sim/sim_env.sh
python3 ~/liftrace_sim/mission_driver.py \
  "7.05,-4.13;4.64,-3.30;4.90,-0.71" 1.5 corridor
```

参数含义：`目标列表`（分号分隔的 x,y）`巡航高度` `corridor`（带走廊；不加则直接去靶标）。

| | |
|---|---|
| 成功标志 | 依次打印 `armed=True` → `OFFBOARD=True` → `takeoff reached` → `corridor-1..6` → `target N dropped=True` → `H lock` → `DONE` |

> **看到 `armed=True` 之后，再去起终端 5。** 顺序反了会让 PX4 以
> `Preflight Fail: heading estimate not stable` 拒绝解锁（视觉链满载拖慢仿真导致）。

### 终端 5 —— 视觉链 + 舵机 mock

```bash
source ~/liftrace_sim/sim_env.sh
python3 ~/liftrace_sim/mock_servo.py &
roslaunch uav_vision phase_d.launch \
  image_topic:=/downward_camera/image_raw \
  camera_info_topic:=/downward_camera/camera_info \
  target_detector_launch_prefix:="$HOME/liftrace_sim/venv38/bin/python" \
  target_model_path:="$UAV_VISION_MODEL_PATH" \
  target_detector_imgsz:=416 \
  map_frame:=camera_init ground_z:=0.0 enable_debug_image:=false \
  start_legacy_compat:=true allow_latest_tf_fallback:=true max_latest_tf_age_sec:=0.30
```

| | |
|---|---|
| 等多久 | 10–15 秒（YOLO 模型加载） |
| 成功标志 | `rostopic hz /uav_vision/detections` → **几十 Hz**；`rostopic hz /uav_vision/detections_mapped` → **~10 Hz** |
| 投递时 | 终端 4 打印 `drop_ready=true -> release`，接着 `servo_complete=True` |

---

## 3. 起飞前的 5 秒自检

在任意终端（已 source 环境）跑这五行，全绿再飞：

```bash
rostopic echo -n1 /mavros/state | grep connected          # connected: True
rostopic hz /Odometry            | head -2                # ~10 Hz
rostopic hz /livox/lidar_r2      | head -2                # ~10 Hz
rosrun tf tf_echo camera_init downward_camera_optical_frame | head -3   # 有 Translation
rostopic hz /uav_vision/detections_mapped | head -2        # ~10 Hz（视觉链起来后）
```

---

## 4. 正常运行时各话题的样子

| 话题 | 期望 |
|---|---|
| `/mavros/state` | `connected: True`，飞行中 `armed: True / mode: "OFFBOARD"` |
| `/mavros/local_position/pose` | ~30 Hz |
| `/livox/lidar` → `/livox/lidar_r2` | 各 ~10 Hz |
| `/Odometry` | ~10 Hz |
| `/freedom/static_pointcloud` | ~10 Hz |
| `/uav_vision/detections` | 几十 Hz（YOLO 输出） |
| `/uav_vision/detections_mapped` | ~10 Hz |
| `/uav_vision/drop_ready` | 对准时 `reason` 从 `align_disabled` → `offset_exceeds_limit` → `ready: true` |
| `/detect/waypoint_mark_point` | 对准时出现（地图系靶心） |

---

## 5. 停止

```bash
# 一条命令全清（顺序无关）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/312_cleanup_all.sh
```

手动版本：各终端 `Ctrl-C`，顺序 **终端 5 → 4 → 3 → 2 → 1**（先停上层，最后停 Gazebo）。

---

## 6. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 没有 Gazebo 窗口 | WSLg 没起来 / `DISPLAY` 不对 | `echo $DISPLAY` 应为 `:0`；`xdpyinfo -display :0` 应 OK |
| 窗口是灰的 | 相机在顶棚外 / 默认视角对墙角 | 左键拖拽 + 滚轮，或双击模型树里的靶标 |
| 满屏 `GST_IS_APP_SRC` + `Connection refused` | **rosmaster 已死**（常见于挂一夜后机器休眠） | 跑 `312_cleanup_all.sh`，不要直接重启 |
| `Unknown model gazebo-classic_iris_mid360` | airframe 运行时副本缺失 | 从 ROMFS 拷到 `~/.ros/etc/init.d-posix/airframes/` |
| `/Odometry` 没有输出 | 忘了终端 2 的桥接 | 确认 `/livox/lidar_r2` 有 10 Hz |
| `tf_echo` 报 frame 不存在 | 忘了终端 3 的 `odom_to_tf.py` | 补起 |
| PX4 拒绝解锁（`heading estimate not stable`） | 解锁顺序/资源问题 | **先解锁再 OFFBOARD**；视觉链等 `armed=True` 后再起 |
| 走廊段撞墙、位置发散到几百米 | 出口直接横穿 1.5 m 高的 `Wall_13` | 出口先爬到 2.2 m 再横穿（脚本已处理） |
| Gazebo 卡在 0.1× 实时 | 设了 `LIBGL_ALWAYS_SOFTWARE=1` | `unset LIBGL_ALWAYS_SOFTWARE`（`sim_env.sh` 已处理） |
| 世界只加载出 ground_plane | 有残留 gzserver 抢 master | `312_cleanup_all.sh` 后重启 |
| RViz 打不开 / 白屏 | `DISPLAY` 不对或 OpenGL 没走硬件 | `echo $DISPLAY`；`glxinfo -B` 应显示 `D3D12 (NVIDIA ...)` |
| RViz 里点云不动 | LIO 没收到激光，或忘了终端 2 桥接 | `rostopic hz /cloud_registered` 应 ~10 Hz |
| RViz 卡顿 | 和 Gazebo GUI 抢 GPU | 用 `333_map_view.sh`（仿真 headless，只给 RViz 渲染） |

---

## 7. 文件与脚本索引

| 文件 | 作用 |
|---|---|
| `~/liftrace_sim/sim_env.sh` | 所有环境变量（`$WORLD`、`DISPLAY`、模型路径） |
| `~/liftrace_sim/livox_bridge.py` | CustomMsg 类型桥接 |
| `~/liftrace_sim/odom_to_tf.py` | `/Odometry` → TF `camera_init→body`（已按时间戳去重） |
| `~/liftrace_sim/odom_to_vision.py` | `/Odometry` → `/mavros/vision_pose/pose`（解锁前锁姿态） |
| `~/liftrace_sim/camera_tf.py` | TF `body→downward_camera_optical_frame`（FC 下 16 cm、绕 Y 180°） |
| `~/liftrace_sim/mission_driver.py` | 任务驱动（飞航点 + 视觉对准 + 投递 + H 视觉降落） |
| `~/liftrace_sim/mock_servo.py` | 仿真舵机 mock（应答 `/detect/servo_status`） |
| `312_cleanup_all.sh` / `313_preflight.sh` | 清残留 / 开工自检 |
| `321_gui_keep.sh` / `322_gui_fly.sh` / `301_full_round.sh` | 只看画面 / 飞完保留窗口 / 飞完自动清理 |
| **`333_map_view.sh [fly]`** | **看建图结果**：headless 仿真 + LIO + RViz（带 `fly` 会起飞绕方形） |
| `163_flow2.sh` | 只做链路自检（**不飞**，约 95 秒后自动关闭窗口） |
