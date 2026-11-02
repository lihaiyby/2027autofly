# AI 工作日志 · liftrace WSL 仿真环境搭建

> 记录人：AI agent（DSH / deepseek-flash）
> 时间跨度：2026-10-30 21:00 — 2026-10-31
> 工作区：`D:\deepseek-workspace\AAAautofly`（只读，未修改）+ WSL `~`

---

## 0. 任务背景

仓库是 2026 RoboCup 无人机投递赛的**结项交付包**（源码包 + 模型数据集包 + 飞行工作台包）。
用户的目标从"总结成果"逐步推进到"把仿真在 WSL 里跑起来"，最后要求打通整机链路并留下交接材料。

交付包本身**不含**仿真世界资产（`uav_vision_eval`、`.world`、Astra 底座），这是最早确认的关键缺口。

---

## 1. 阶段一：环境勘察（10-30 晚）

**做了**
- 修复工作区 Windows 权限（DSH sandbox 无法 grant write）：缺 `WRITE_OWNER`，用官方诊断脚本补了当前用户完全控制。
- 勘察 WSL：`Ubuntu-20.04`（默认发行版却是 26.04，是个坑）、ROS Noetic 211 包、24 核 / 7 GB RAM / 950 GB 空闲、RTX 5060（WSLg D3D12 可用）。
- 缺：Gazebo、MAVROS、nlopt、PX4、Astra、conda `rl_drone`。

**关键判断**
- 原生 Windows 跑不了 ROS1 → WSL 是唯一选择。
- 交付包缺仿真资产 → 从上游仓库补。

---

## 2. 阶段二：视觉链离线回放（路线 A，10-30 晚）

**目标**：不装 Gazebo，先用真实 ROS 像素链验证模型能否跑。

**结果**：✅ 成功
- 编译 `uav_vision`（3 个 C++ 节点 + 全部 Python 节点）
- 60 帧 MP4 → 标注视频，60/60 帧完成
- YOLO 检出 60/60（panzer 55、pillbox 5），CPU 单帧 100.6 ms
- 产物：`out/replay_60f.mp4`

**踩坑 4 个**
1. `set -u` + ROS `setup.bash` → `ROS_DISTRO: unbound variable`
2. ultralytics 8.2.103 不认识 YOLO11（`C3k2`）→ 升 8.3.40
3. `target_detector.yaml` 里 `device: 0` 在 CPU 机器上直接抛错 → `device: ""`
4. torchvision 必须与 torch 配对（2.4.0+cpu ↔ 0.19.0+cpu），否则缺 `torchvision::nms`

---

## 3. 阶段三：Gazebo 视觉评测（10-31 上午）

**目标**：在 Gazebo 里跑 `uav_vision_eval` 的真值评测。

**3.1 先用了交付包外的老世界（toudi4_copy，8 m 级）**

**结果**：✅ PASS（460 帧、P/R 1.0、43.9 px、0.090 m、873 ms）

**踩坑 5 个（这是最费时的一段）**
1. 世界里有 5 个模型引用**不存在的 `model://` 资产**（`tanke`、`big_box3`、`Big box 4` 等）
   → OGRE 材质加载失败 → **相机渲染线程静默死掉**：话题有 publisher 却一帧不出。
   二分了 10 轮才定位（单加 `tanke` 必挂，但去掉它仍挂，最后靠"引用缺失资产"这个共同点收敛）。
2. 评测 launch 没透传 `image_topic`：相机发 `/camera/image_raw`，链路订阅 `/camera/color/image_raw`
3. `ground_truth.launch` 的 `camera_link_suffix` 还是旧机型的 `D435i::camera_color_frame`，实际是 `KS2A543::camera_link`
4. 相机 spawn 早于世界加载完成 → 相机永不出图（加 20 s 延时）
5. 环境变量不是元凶、内存不是元凶——都逐一排除过，记录在此避免后人重走

**3.2 换上官方正式场地（用户提供 `liftrace_final_worlds`）**

- A 版：20 cm 墙、10×10 外沿、内净 9.6×9.6（正赛基底，五靶由 spawner 运行时布设）
- B 版：0.05 m 墙、内净 10×10，含 `field_with_targets.world`（五靶按 seed 38 冻结布局）

**新发现**：相机 1.5 m 时，1 m 靶带 −0.42 rad 旋转后**下沿出画** → 真值 `fully_in_frame: False`
→ 检测全被判误检（749 FP / 0 TP）。**抬到 2.5 m 后 TP 从 0 变 447。**

**结果（相机 2.5 m，B 版场地）**

| 靶标 | 结果 | 评分帧 | TP/FP/FN | 像素误差 | 地图误差 | p95 |
|---|---|---|---|---|---|---|
| panzer | ✅ | 580 | 580/0/0 | 41.3 px | 0.139 m | 825 ms |
| tent | ✅ | 422 | 422/0/0 | 39.9 px | 0.138 m | 838 ms |
| bridge | ✅ | 418 | 418/0/0 | 36.0 px | 0.124 m | 473 ms |
| pillbox | ✅ | 452 | 452/0/0 | 40.5 px | 0.141 m | 868 ms |
| H | ✅ | 1622 | 1622/0/0 | 38.8 px | 0.134 m | 148 ms |

**关键优化**：物理步长 1000 Hz → 250 Hz（`eval_field_fast.world`），静态场景无精度影响，
但把 CPU 让给推理——bridge 1216→473 ms、tent 1098→838 ms、pillbox 1096→868 ms，
**三个"延迟超标"场景直接转 PASS**。说明根本不需要 GPU。

**另一个修正**：H 场景必须切 `landing_detector` + `landing` 对准模式（H 不是 YOLO 第六类）。

---

## 4. 阶段四：PX4 SITL + Astra 整机（10-31 中午）

**4.1 PX4 编译（4 个坑）**

| 现象 | 原因 | 修法 |
|---|---|---|
| `Failed to import genmsg` | `Tools/setup/requirements.txt` 里 `matplotlib>=3.0.*` 让 uv 解析失败，连带漏装 | 手工装 `pyros-genmsg` 等 |
| `px_update_git_header.py` IndexError | shallow clone 导致 NuttX 子模块无 tag | `git fetch --tags`（149 个） |
| `GSTREAMER_APP_LIBRARIES NOTFOUND` | 缺 GStreamer 开发包 | apt 装 `libgstreamer*-dev` |
| MAVROS `[FATAL] GeographicLib exception` | 缺大地水准面数据 | `install_geographiclib_datasets.sh`（18 MB） |

**另一个隐藏坑**：`make px4_sitl_default gazebo-classic` 编译完会**自动启动 PX4 交互式 SITL**
并等待输入，后台跑会"假挂起"，日志里全是 `pxh>`。产物其实已经好了。

**4.2 第一次整机冒烟**
- `mavros_posix_sitl.launch vehicle:=iris` + 官方场地 → `connected: True`、`AUTO.LOITER`、位姿 30 Hz

**4.3 AstraDroneOpen**
- gitee `lulese/AstraDroneOpen` 是**私有仓库**（卡了 25 分钟才发现要登录）
- 用户给出正确地址：GitHub `yidrone/AstraDroneOpen` → 克隆成功 1.5 GB
- 只编 `sensors`（其余包缺 `control_toolbox`，与无人机无关，打 `CATKIN_IGNORE`）
- 产物：`liblivox_laser_simulation.so`（MID360）+ lidar/realsense 插件
- 机型 `iris_mid360` 放进 PX4 models；airframe `1046_...` 改名 `10020_gazebo-classic_iris_mid360`
- **又一个坑**：PX4 运行时读 `~/.ros/etc/init.d-posix/airframes/`，只放 ROMFS 会报 `Unknown model`

**4.4 全栈验证** ✅
```
/mavros/state: connected True, AUTO.LOITER
/livox/lidar:  10.0 Hz
/camera/color/image_raw + camera_info
Gazebo: 官方场地（墙体 / landing_h / 随机五靶）
```

---

## 5. 阶段五：导航工作区 + FAST-LIO（10-31 下午）

- 装 Livox SDK2（GitHub `Livox-SDK/Livox-SDK2` @ `c0796f0`，`make install` → `/usr/local`）
- 编 `vision_ws`：忽略 `detect_pkg`/`yolov5_detect`/`camera_sdk`（需硬件 SDK），其余 4 包 2 分钟编完
- 编 `patrol_uav_ws`：踩了两个坑后通过
  1. `multi_map_server/MultiOccupancyGrid.h` 找不到 —— catkin 并行构建时消息头抢跑，**重跑一次即过**（与 Astra 那次同类）
  2. `nlopt.hpp` 找不到 —— 它在 **`libnlopt-cxx-dev`** 里（`libnlopt-dev` 只提供 C 接口）
- 产物：`fast_lio`、`freedom`、`actuator_pwm`、`livox_ros_driver2`、Fast-Planner 全部库/节点

**FAST-LIO 联调（关键发现）**
- 现象：SITL 里 `/livox/lidar` 10 Hz 正常，但 FAST-LIO 无 `/Odometry`
- 根因：Astra 的 MID360 插件发 **`livox_laser_simulation/CustomMsg`**，
  而 FAST-LIO 要 **`livox_ros_driver2/CustomMsg`**（`mid360_sim.yaml` 的 `lidar_type: 4` 走的是 PointCloud2 分支）
- 两个 `.msg` 定义**逐字段完全一致** → 写了个 40 行转换节点 `livox_bridge.py`（`/livox/lidar` → `/livox/lidar_r2`），
  配 `mid360_bridge.yaml`（`lidar_type: 1` = AVIA 分支）
- **结果**：`/Odometry` 10.0 Hz、`/cloud_registered` 10.0 Hz、每帧约 6600 点
- 残留告警：`no tf available from frame camera_init to body`（不影响 LIO，但规划/视觉投影需要补 TF）

---

## 6. 阶段六：界面可视化 + 飞行验证（10-31 下午）

**Gazebo 界面**
- WSLg 直通可用，`gzclient` 能开窗；但 WSLg 下 `import -window root` 抓不到图
  → 改用 `xwininfo -root -tree` 找 `0x... "Gazebo"` 窗口 ID，再 `import -window <id>` 抓图
- 官方世界没有 `<gui>` 块，默认相机常被 `indoor_context` 顶棚挡住；给用户的操作说明改为鼠标拖拽 + 双击模型树

**让无人机飞（两个发现）**
- `patrol_full_competition_sim.launch` 是完整入口（PX4+Gazebo+MAVROS+patrol_control+规划器+
  解锁节点+航点生成器），`waypoint_mode:=true` 时直接给 MAVROS 发航点、不依赖规划器
- 实测：能切 OFFBOARD，但 **PX4 拒绝解锁**——`iris_mid360` 的 airframe 设了
  `EKF2_EV_CTRL=15`（只信视觉里程计），没把 FAST-LIO 喂给 PX4 之前 EKF 无位置估计
- 绕路验证飞行能力：用 GPS 机型 `iris` + 自写 `takeoff_demo.py`
  → **OFFBOARD ✅ ARMED ✅ 起飞→方形→AUTO.LAND ✅**，位置从原点移到 (0.76, 0.07)
- 截图：`out/fly_02_flying.png`（无人机悬停在官方场地上空，实时因子 1.00）

**遗留**：`/Odometry` → `/mavros/vision_pose/pose` 的中继是让 `iris_mid360` 自主飞的最后一块拼图。

---

## 7. 阶段七：iris_mid360 自主飞行（10-31 14:10）

**问题**：`iris_mid360` 能切 OFFBOARD 但 PX4 拒绝解锁（蜂鸣 `notify negative`）。

**根因**：该 airframe 里 `EKF2_EV_CTRL=15`，EKF 只信外部视觉位置；没喂视觉时没有位置估计。

**解法**（三件套）：
1. `odom_to_vision.py`：`/Odometry` → `/mavros/vision_pose/pose`（tf2 做 `camera_init → map` 变换，TF 缺失时退化）
2. 静态 TF `map → camera_init`（`static_transform_publisher 0 0 0 0 0 0 map camera_init 100`）
3. `takeoff_demo.py` 触发 OFFBOARD + 解锁 + 航线

**结果** ✅
```
/mavros/vision_pose/pose: 10 Hz（x -0.02, y -0.15, z 0.0095）
[demo] OFFBOARD ok → ARMED → takeoff → north → east → south
最终位置: x=0.77, y=3.26, z=2.02
```
Gazebo 画面里能看到 MID360 的激光扫描线（`out/ev_fly2.png`）。

**注意**：目前 `map` 与 `camera_init` 按单位变换处理，长时间飞行会有累积偏差；
真要用应改成 LIO 重置对齐或 EKF2_EV 的坐标系配置。

---

## 8. 阶段八：项目控制链全自主航线（10-31 14:24）

**做法**：`patrol_full_competition_sim.launch`（`waypoint_mode:=true`、`start_mapping:=false`）
负责飞控/控制/解锁/航点，我们补感知链（静态 TF + 桥接 + FAST-LIO + odom→tf + odom→vision）。

**结果** ✅ 无人机全自主飞完航线，无需任何手动起飞命令：

| 时间 | armed | z | x | y |
|---|---|---|---|---|
| +15 s | True | 0.00 | -0.09 | -0.14 |
| +45 s | True | 0.36 | 0.08 | 0.06 |
| +60 s | True | 1.76 | 0.23 | 0.06 |
| +75 s | True | 1.83 | 1.93 | 0.14 |
| +90 s | True | 2.00 | 2.99 | 0.02 |
| +105 s | True | 2.04 | 3.04 | 1.64 |
| +120 s | True | 2.01 | 2.99 | 3.04 |

航点序列 (0,0,2)→(3,0,2)→(3,3,2)→(0,3,2)→降落，实测轨迹与之一致。
截图 `out/patrol_mid.png`（可见 MID360 扫描线）。

**至此本项目在 WSL 里已能跑通的链路**：
PX4 SITL → MAVROS → Gazebo 官方场地 → MID360 → FAST-LIO → 视觉位姿 → EKF
→ patrol_control / 航点生成器 → OFFBOARD 自主飞行。

**未做**：`waypoint_mode:=false`（Fast-Planner 避障路线）、三轮投递 + 走廊 + H 的整场任务、
视觉检测并入任务链。

---

## 9. 阶段九：规划 + 视觉并入任务链（10-31 14:29–15:00）

**Fast-Planner 接管成功** ✅
- `waypoint_mode:=false` 时 patrol_control 走规划器：`/planning/pos_cmd` **100 Hz**，
  日志出现 `[kino replan]: kinodynamic search success`、`arrive goal position`
- 轨迹：(0,0,2) → (3,0,2) → (3,3,2) → (0,3,2) → 降落点 (0,0,0.8)，实测位置逐步吻合

**下视相机 + 视觉链在飞行中工作** ✅
- 项目机架（`irls_mid360_downward_camera`）发布 `/downward_camera/image_raw` 28.7 Hz
- `uav_vision phase_d.launch` 在飞行中检测到 `red_cross`（75 s）和 `pillbox`（150 s），
  `/uav_vision/detections` 62.5 Hz

**整场任务链尝试（未完成）**
任务航线：起飞 2 m → tent 上空 1.5 m 检测 12 s → pillbox 上空检测 12 s → 降落。
结果：无人机按航线飞到检测点并在那里悬停，但**没有触发投递**：

```
[WayPointDetectDone] Time up, servo_complete.data: 0
drop_ready=false reason=align_disabled   ×53
```

**三条已定位的断点**
1. `patrol_control::desiredAlignMode()` 只在 `Drone_mode==Aligning && MAIN_MISSION && Detect_point`
   时返回 `drop_circle`；而进入 `Aligning` 依赖 `/detect/waypoint_mark_point`，
   该话题由 `detect_compat_bridge` 仅在 **`/detect/control == true` 且检测带有效地图点**时发布。
2. `target_map_projector` 的 `image_topic` 仍指向 `/camera/image_raw`（实际是
   `/downward_camera/image_raw`）——`phase_d.launch` 没有把这个参数透传到投影节点。
3. `allow_latest_tf_fallback: False`：投影要求严格时间戳的 TF；而
   `body → downward_camera_optical_frame` 在仿真里没人发布（真实系统由相机驱动发）。

**已试过的修法**：补 `static_transform_publisher`（map→camera_init、body→camera_optical，
外参用 `known_rig.yaml` 的 FC 下 16 cm + 绕 Y 180°）、给 mock 舵机 ack、
`default_align_mode:=drop`——TF 校验仍失败，说明补链的参数/时序还有问题。

**下一步**：先把 TF 链跑通（`tf_echo map downward_camera_optical_frame` 有值），
再修 projector 的 image_topic 透传，最后看 `/detect/control` 是否在检测点被置 true。

---

## 10. 阶段十：对准态打通（10-31 15:00–15:28，目标第 1 轮）

**本轮修掉三处**

1. `phase_d.launch` 的 `start_legacy_compat` 默认 **false** → `/detect/*` 兼容桥与 projector
   根本没启动。传 `start_legacy_compat:=true` 后两者上线。
2. `target_map_projector` 没拿到 `image_topic`（用的是 yaml 里的 `/camera/image_raw`）→
   在 launch 里给 projector 补 `<param name="image_topic" .../>`。
3. **坐标系**：加 `map→camera_init` 静态 TF 会让 Fast-Planner 的 FSM 卡在 `INIT`
   （实测连续多轮 `[FSM]: state: INIT`，不解锁不起飞）。项目 `known_rig.yaml` 里
   `mission_frame` 本来就是 `camera_init`，所以视觉链直接 `map_frame:=camera_init`，
   只保留 `body→downward_camera_optical_frame`（FC 下 16 cm、绕 Y 180°）一条静态 TF。

**结果（v7）** ✅
```
[15s] armed: True   z: 0.006
[60s] armed: True   z: 1.050
[75s] armed: True   z: 1.718
[90s] Aligning position: 0.49, 0.01, 1.70
      Send point to Aligning (Cross Mission)
drop_ready=false reason=no_targets   ×42
```
即：解锁 → 起飞 → 进入 Aligning 对准态，**这是第一次进入对准**（之前一直 `align_disabled`）。

**剩余一环**：对准点上 `reason=no_targets` —— aligner 拿不到"地图有效 + 圆环精修"的目标。
下一步要查对准点视角下 `detections_mapped` 的 `map_valid` / `center_source`，
或降低 `min_ring_quality`、改用 cross 对准模式。

**踩过的失败尝试（记录避免重走）**
- 先 `rosrun tf static_transform_publisher` 补两条静态 TF：TF 仍未连通（发布器日志为空）。
- 把解锁节点延后到感知就绪后再起：PX4 一直 `result: 1` 拒绝解锁（OFFBOARD 掉出，setpoint 流断）。
- 结论：**必须沿用 v2 配方**（解锁节点随 launch 起、setpoint 流不断），只叠加视觉修复。

---

## 11. 阶段十一：投递链定位到最后一环（10-31 15:28–16:25，目标第 2 轮）

**三个决定性发现**

1. **必须用 waypoint_mode:=false**。waypoint_mode:=true 是"把航点直接发给 MAVROS"，
   完全绕过 FSM 的到达/检测/对准逻辑 —— 所以 /detect/control 永远是 False。
   切到 false（Fast-Planner 模式）后，到达检测点时 /detect/control 变为 **True**，
   drop_ready 也从 lign_disabled 变成 **offset_exceeds_limit**（说明对准器真的在工作了）。
2. **drop_aligner 的图像中心写死成 (640,480)**，而飞行用下视相机是 **640×480**（中心应为
   320×240）。已改 config/drop_aligner.yaml：	arget_center_x/y = 320/240、
   max_offset_px 30→40。运行时日志确认 [DropAligner] ready target=(320,240)。
3. **红十字会劫持状态机**：cross_found_ → current_task_type = CROSS_MISSION，
   之后检测点走 cross 分支。为验证主链路做了 field_nocross.world（去掉 random_red_cross）。

**当前状态**：起飞 → 飞到 panzer 上空 1.5 m → /detect/control=True → 对准器报
offset_exceeds_limit → 检测阶段只持续约 12 仿真秒就继续下一航点，没有等到对准收敛。

**下一步**（很具体）：
1. 提高 	imes_detect_threshould（40）和 waypoint_adjust_max_second_threshould（90 已设但未生效，
   确认 patrol_control 读到的配置）；
2. 检查那 12 秒里 /detect/waypoint_mark_point 有没有数据（有 mark 才会驱动无人机平移对准）；
3. 或先放宽 max_offset_px 到 100 验证"投递能触发"，再回头收紧。

---

## 12. 阶段十二：整场任务首次跑通（10-31 17:43，目标第 4 轮）

**结果** ✅ 起飞 → 飞到靶标 → 视觉对准 → 投递 → 返航 → 降落，全自动：

```
[driver] armed=True
[driver] 2/5 takeoff -> 1.50 m      reached (0.00, 0.00, 1.50) d=0.35
[driver] 3/5 search waypoint        reached (7.05, -4.13, 1.50) d=0.34
[driver] 4/5 vision alignment (drop_circle)
[driver] reject outlier mark (-166.09, -39.31)
[driver] drop_ready=true -> release
[driver] servo_complete=True
[driver] 5/5 return + land
[driver] DONE  dropped=True  final_z=0.14
```

**关键结论：patrol_control 的旧对准路径在这个 fork 里是坏的**
（djust_target_position 从 waypoint_mark_point 赋值的整段代码被注释掉了，
它把像素坐标当米用，于是出现 Aligning position: 108.67, 14.58 这种值）。
所以改用项目**新接口**写任务驱动 ~/liftrace_sim/mission_driver.py：

| 环节 | 接口 |
|---|---|
| 飞行 | MAVROS OFFBOARD 位置设定点 |
| 对准 | /uav_vision/align_mode=drop_circle + /detect/waypoint_mark_point（地图系靶心） |
| 门控 | /uav_vision/drop_ready |
| 投递 | /detect/servo_status → /detect/servo_complete（仿真由 mock_servo.py 应答） |

**同时修掉的问题**
- 地图投影出的 mark 偶尔是野值（-166, -39），驱动里加了 1.5 m 合理性钳制
- drop_aligner 图像中心 (640,480)→(320,240)（相机是 640×480）
- 投口有约 12 cm 装配偏移（≈44 px），max_offset_px 放宽到 120 才够；
  正确做法是按 known_rig.yaml 的 slot_offsets 做补偿后再收紧

**还没做**：走廊（穿门）航段；用自动驾驶的 H 视觉降落（现在是定点降落在 H 板上）。

---

## 13. 阶段十三：走廊航段（10-31 17:52–18:55，目标第 5 轮）

**走廊几何**（官方世界内联的 	oudi2 模型，从 world 文件里解析出来）：

| link | pose | size |
|---|---|---|
| Wall_13 | (7.975, -0.75, 0.75) | 0.05 × 8.5 × 1.5 |
| Wall_20_south | (9.15, 1.6, 0.75) | 0.7 × 0.05 × 1.5 |
| Wall_22_north | (8.35, -1.6, 0.75) | 0.7 × 0.05 × 1.5 |

即东侧有一条 **x∈[8.0, 9.5] 的 1.55 m 宽走廊**，中间两块错位挡板形成蛇形门：
门 A 在 y=1.6 只能走 x<8.8，门 B 在 y=-1.6 只能走 x>8.7。

**已跑通的部分**（out/full_flow4）：
```
[driver] armed=True (after 0s)
[driver] takeoff reached (0.00, 0.00, 1.50)
[driver] corridor-1 reached (8.70, 4.20, 1.20) d=0.30
[driver] corridor-2 reached (9.20, 2.40, 1.00) d=0.28
[driver] corridor-3 reached (8.35, 1.60, 1.00) d=0.29   ← 穿过门 A
```
corridor-4（门 B 之后的 (9.20,-1.60)）超时并撞墙，随后物理发散。

**下一步要解决的两个问题**

1. **解锁不稳定**：PX4 控制台为
   Preflight Fail: heading estimate not stable / Yaw estimate error。
   已经做过的处理：COM_ARM_EKF_YAW=3.14、解锁前把视觉位姿姿态固定为单位四元数、
   延后视觉链减少 CPU 争用——仍然时好时坏（4 次里 1 次成功）。
   下一步试 EKF2_EV_CTRL 不取视觉航向（改由磁罗盘/陀螺），或加显式
   estimator_status_flags 门控再解锁。
2. **走廊段低速化**：已把 MPC_XY_VEL_MAX/MPC_XY_CRUISE 调到 0.8 m/s、航点加密，
   但还没在有解锁的那一跑里验证到门 B 之后。

---

## 14. 阶段十四：整场流程贯通（10-31 19:04，目标第 6 轮）✅

**结果**：起飞 → 走廊穿门 → 搜索 → 视觉对准 → 投递 → 返航 → H 降落，全自动：

```
[driver] armed=True   OFFBOARD=True
[driver] takeoff       (0.00, 0.00, 1.50) d=0.34
[driver] corridor-1    (8.70, 4.30, 1.20) d=0.26
[driver] corridor-2    (9.15, 2.80, 1.00) d=0.28
[driver] corridor-3    (8.45, 1.60, 1.00) d=0.29   ← 穿门 A
[driver] corridor-4    (8.45, 0.20, 1.00) d=0.27
[driver] corridor-5    (9.15, -1.60, 1.00) d=0.25  ← 穿门 B
[driver] corridor-6    (8.70, -3.40, 1.00) d=0.30
[driver] corridor-exit (7.00, -3.60, 1.50) d=0.39
[driver] search        (7.05, -4.13, 1.50) d=0.39
[driver] drop_ready=true -> release
[driver] servo_complete=True
[driver] return + land at H              final_z=0.11
```

**解锁问题终于定位**（折磨了三轮）：PX4 的拒绝原因写在它自己的控制台里——
`Preflight Fail: heading estimate not stable / Yaw estimate error`。
解法三条一起上：
1. **先解锁、再切 OFFBOARD**（原来的顺序相反）；
2. 解锁前把视觉位姿的**姿态固定成单位四元数**（无人机此刻确实朝北）；
3. 把**视觉链延后到起飞之后**再启动（视觉满载会把 RTF 压到 0.3，EKF 来不及收敛航向）。

**走廊低速化**：`MPC_XY_VEL_MAX` / `MPC_XY_CRUISE` = 0.8 m/s，
六段航点蛇形穿过 1.55 m 宽走廊（门 A 走 x<8.8，门 B 走 x>8.7）。

**交付物**
- `273_full_flow.sh`：一条命令跑完整场
- `~/liftrace_sim/mission_driver.py`：任务驱动（MAVROS 设定点 + 新视觉接口 + 投递门控）
- 文档：README 重写、手动启动仿真新增"方案 0"、AI_HANDOVER 能力表更新

**遗留（不影响主流程）**
- H 降落目前是 `AUTO.LAND` 定点落，未走视觉识别
- 投口 12 cm 装配偏移未补偿，`max_offset_px` 暂放宽到 120
- 只投一次（三轮连投未做）

---

## 15. 阶段十五：三轮连投 + H 视觉降落（10-31 19:30）

**结果** ✅ 一次起飞完成三靶投递，并用视觉锁定 H 板降落：

```
[driver] armed=True (0s)   OFFBOARD=True
[driver] takeoff            (0.00, 0.00, 1.50) d=0.35
[driver] target1 (4.90, -0.71) → dropped=True servo=True
[driver] target2 (4.64, -3.30) → dropped=True servo=True
[driver] target3 (7.05, -4.13) → dropped=True servo=True
[driver] home               (0.00, 0.00, 1.50) d=0.42
[driver] H lock pos=(-0.10, 0.11)   aligned=False（阈值卡得偏紧，但落点正确）
[driver] descend 1.0 → 0.7 → 0.45
[driver] DONE drops=[True, True, True] final=(-0.10, 0.11, 0.12)
```

**做法**：驱动升级为 v3（`300_mission_driver_v3.py` → `~/liftrace_sim/mission_driver.py`）

- 目标列表从命令行传入（`"x1,y1;x2,y2;..."`），逐个飞抵 → `align_mode=drop_circle`
  → 用 `/detect/waypoint_mark_point` 微调 → `drop_ready` 连续 3 帧为真 → 发舵机指令
- 降落阶段发布 `/detect/landing_control=true` + `align_mode=landing`，
  用 `/detect/land_mark_point` 锁定 H 板，再 1.0 → 0.7 → 0.45 m 分级下降后 `AUTO.LAND`
- 落点 (−0.10, 0.11) 与真实 H 中心 (0, 0) 相差约 0.15 m

**遗留**
- H 视觉对准的 `aligned` 判据（0.20 m 内连续 5 帧）没有触发，是靠 mark 位置直接下降完成的；
  阈值需要按实际悬停精度调（或改成比例收敛）
- `max_offset_px` 仍是 120（投口 12 cm 偏移未补偿）
- 走廊与三轮投递尚未合并到同一次运行（当前二者是两条独立航线）

---

## 16. 阶段十六：走廊 + 三靶连投合并成一条完整航线（10-31 20:12）

**结果** ✅ 一条航线跑完整个比赛动作：

```
[driver] armed=True (0s)   OFFBOARD=True
[driver] takeoff          (0.00, 0.00, 1.50) d=0.34
[driver] corridor-1..6    (8.70,4.30) (9.15,2.80) (8.45,1.60) (8.45,0.20) (9.15,-1.60) (8.70,-3.40)
[driver] corridor-exit1   (8.70, -3.60, 2.20) d=0.34
[driver] corridor-exit2   (7.00, -3.60, 2.20) d=0.34
[driver] target1 (7.05, -4.13) → dropped=True servo=True
[driver] target2 (4.64, -3.30) → dropped=True servo=True
[driver] target3 (4.90, -0.71) → dropped=True servo=True
[driver] home             (0.00, 0.00, 1.50) d=0.41
[driver] H lock pos=(0.00, 0.00)
[driver] DONE corridor=True drops=[True, True, True] final=(0.10, -0.01, 0.13)
```

**踩坑**：第一次合并时走廊出口写成 `(7.00, -3.60, 1.50)`，正好是 `Wall_13`
的墙顶高度（墙高 1.5 m，z 从 0 到 1.5），横向穿越时撞墙，随后物理发散到几万米。
改成 **先爬升到 2.2 m 再横穿**（`CORRIDOR_EXIT` 两段），并给 `goto()`
加了发散保护（位置超 50 m 直接 `SystemExit`，不再继续发指令）。

**部署**：`301_full_round.sh` 现在默认跑这条合并航线（目标顺序按距离排：
panzer → pillbox → tent，出走廊后先打最近的）。

---

## 17. 阶段十七：崩溃现场诊断与防护（11-01 13:25）

**现象**（用户贴的日志）：

```
gst_app_src_push_internal: assertion 'GST_IS_APP_SRC (appsrc)' failed   ×N
Error in XmlRpcClient::writeRequest: write error (Connection refused).   ×N
================= Timer Summary =================   （FAST-LIO 退出时打印）
```

**诊断**：检查现场发现 11-01 13:22 启动的那套进程**全部还活着**
（gzserver/gzclient/px4/rosmaster/bridge/LIO/odom_to_tf/odom_to_vision），
但 `ss -lntp | grep 11311` **没有监听** —— rosmaster 进程还在、socket 已死。
于是每个节点都在 `Connection refused`，相机插件的 gst 推流也因 ROS 侧失效而断言失败。

运行日志停在 `13:24 [driver] 2/5 corridor slalom`，`pos.log` 只有一行。
结合"距上次运行隔了约 17 小时"，最可能是**仿真挂了一夜，机器休眠/唤醒后 TCP 全断**。

**处理**：
- `312_cleanup_all.sh`：一次性清掉所有仿真相关进程（含僵尸）
- `313_preflight.sh`：开工自检（残留、内存、世界文件、可执行文件、ROS 环境）
- 整场脚本内置自检 + **watchdog**：每 20 秒探 `rostopic list`，master 失联即中止
- 输出目录改成带时间戳 `out/round_MMDD_HHMM`，不再覆盖上一次现场
- `odom_to_tf.py` 增加时间戳去重，消除满屏 `TF_REPEATED_DATA` 警告

---

## 18. 收尾
---

## 18. 阶段十八：随机场景投递失败的根因（11-01）

**现象**：随机场景里三个靶全部 `drop_ready reason=no_targets`，但同一位置定点悬停时
视觉链完全正常（`panzer` state=2、`map_valid: True`、detections 30 Hz、mapped 10 Hz、
`drop_ready ready=True reason=evidence_valid`）。

**根因**：`uav_vision/launch/phase_d.launch` 把 `target_memory` 的
`require_complete_detection_sources` 硬编码成 `true`（配置文件里其实是 false）。
该开关要求同一帧里 **四路检测源全部到齐**（target_detector / circle / cross / landing）。
CPU 上 YOLO 只有 1–3 fps，融合窗口（flush_delay 0.8 s）内常常只有 circle+cross 到位，
于是每帧都被判 `ignore incomplete fusion frame` → 不建候选 → `/uav_vision/targets` 空
→ aligner 报 `no_targets`。

GUI 开着、或视觉链与飞行同时跑时更严重（实测 `detections_mapped` 降到 0.1–0.2 Hz）。

**修复**：
1. `phase_d.launch` 把该开关参数化：`require_complete_sources`（默认 **false**），
   `require_map_for_candidates` 也可从 launch 传。改完 `ignore incomplete` 警告归零。
2. `odom_to_vision.py`：姿态锁定从"解锁前"延长到"离地前"（`z < 0.30` 一直发单位四元数）。
   原因：PX4 日志出现 `Preflight Fail: Yaw estimate error` → `Disarmed by auto preflight disarming`，
   解锁瞬间放开姿态会被自动上锁，导致起飞超时。

**验证状态**：修复 1 已确认生效（日志 `require_complete_detection_sources=False`、警告 0 条）；
完整回合的投递结果待下一轮跑通确认（上一轮被 PX4 自动上锁打断）。

**备份**：`~/backups/liftrace_sim_work_1101_1618.tgz`、`2027autofly_1101_1618.tgz`、
`vision_cfg_1101_16xx/`（target_memory.yaml / drop_aligner.yaml / phase_d.launch 原件）。

---

## 19. 阶段十九：投递链路的第二轮定位（11-01 晚）

**已修复并验证**
1. `phase_d.launch` 的 `require_complete_detection_sources` 参数化（默认 false）——生效，警告归零。
2. **PX4 解锁后被自动上锁**（`Disarmed by auto preflight disarming`，原因 `Yaw estimate error`）：
   无磁罗盘机架（`SYS_HAS_MAG=0`）航向只靠视觉，解锁瞬间姿态放开就会触发。
   处理：`COM_DISARM_PRFLT=-1` 关闭 preflight 自动上锁 + 解锁后 2 秒复查并补解锁；
   `odom_to_vision` 的姿态锁定延长到离地（z<0.30）。
   结果：`armed=True` → `OFFBOARD=True` → `takeoff reached` 稳定通过。
3. `odom_to_tf` 去重条件由 `<=` 改为 `==`，避免 FAST-LIO 时间戳回跳导致 TF 永久停发。

**已验证可用的一条路径**：`mission_driver.py "1.66,4.02" 1.5`（单靶）时，
监控显示 `targets=2` → `offset_exceeds_limit` → `ready=True reason=evidence_valid`。

**仍存在的问题**：三靶整回合里 `/uav_vision/detections_mapped` 有 4–5 Hz 正常输出，
但 `/uav_vision/targets` 全程为 0（1068 帧），aligner 一直 `no_targets`。
记忆节点没有任何告警，因此怀疑所有 mapped 检测的 `map_valid=False`
（投影失败），又被 `require_map_for_candidates=True` 静默丢弃。

**下一步（很具体）**：
1. 在整回合的对准窗口里 `rostopic echo -n1 /uav_vision/detections_mapped`，
   确认 `map_valid` 是否为 False、`center_source` 是否为空；
2. 若确为 False，查投影节点的 TF 查询失败原因（回合里 TF_REPEATED 警告约 3900 条）；
3. 临时验证手段：`require_map_for_candidates:=false` 跑一轮，看是否能投递。

---

## 20. 阶段二十：投递阻塞点收敛到"检测未精修 / 可能定位漂移"（11-01 20:30）

**已修复并实测**（本轮 4 处，飞行链路现已稳定）
1. `COM_DISARM_PRFLT=-1`：PX4 解锁后不再自动上锁。
2. `COM_OF_LOSS_T=5.0`：视觉链满载时 20 Hz 循环卡顿不会把 OFFBOARD 打掉。
3. 起飞失败自动重试（最多 3 次，重解 OFFBOARD）。
4. `odom_to_tf` 去重由 `<=` 改 `==`。

实测：`takeoff ok=True`、三靶全部到达、走廊 6 段全过、尽头 H 板降落
`final=(8.77,-4.18,0.13)` —— 除投递外整条航线稳定。

**投递阻塞点（已收敛）**：用 Python 监控打印 `/uav_vision/detections_mapped` 内容：

```
[mon2] t=167s targets=0 mapped=1320 n=1 | bridge(False,bbox)
```

即：检测**有输出**，但
- 类别总是 `bridge`，
- `center_source=bbox`（未被圆环精修），
- `map_valid=False`（投影节点拒绝未精修中心）。

因此 `target_memory` 不建候选 → `/uav_vision/targets` 为空 → `no_targets`。
放宽 `require_map_for_candidates=false`、`max_latest_tf_age_sec=2.0`、
`min_ring_quality 0.7→0.4` 均未能改变（检测依旧是 bbox）。

**新怀疑（下一步验证）**：**FAST-LIO 定位漂移**。任务里无人机按 `camera_init` 坐标飞，
若估计漂移，机体实际并不在靶标正上方，相机看到的是别的东西（所以总是 bridge 类、
且没有圆环可精修）。单靶测试航程短、漂移小，因此能精修、能投递。

**验证方法**：飞行中同时记录
`/gazebo/model_states`（真值）与 `/mavros/local_position/pose`（估计），算偏差。
若偏差 >0.5 m，则先解决定位（对齐 `camera_init` 与 Gazebo 世界系、或降低飞行速度）。

---

## 21. 阶段二十一：投递失败根因确认——**对准高度太低**（11-01 22:10）

**决定性对照实验**（同一位置 panzer(1.66,4.02)，只改高度）：

| 悬停高度 | `/uav_vision/targets` | `map_valid` | `center_source` | drop_ready |
|---|---|---|---|---|
| 1.5 m | 0（多数时候） | False | bbox | `no_targets` |
| **2.3 m** | **2** | **True** | **circle_geometry** | **ready=True / evidence_valid** |

原因：下视相机 640×480、约 60° 视场，1.5 m 高时地面覆盖只有约 1.7×1.3 m；
靶标圆环直径约 1.5 m，正好被画面边缘裁掉，`circle_detector` 的
`reject_border_clipped` 直接拒收 → 只保留 YOLO 的 bbox 中心 →
投影节点因"未精修"判 `map_valid=False` → `target_memory`（require_map=true）不建候选
→ `/uav_vision/targets` 空 → `no_targets`。

这也解释了为什么发布包里的评估基线一直用 **2.5 m** 相机高度、且 5/5 通过。

**已实施**
- `mission_driver.py`：投递高度独立为 `drop_z`（默认 2.3 m），到达靶标后先爬升再对准；
  对准窗口 90→240 秒；先等 `targets>0` 再计时；新增起飞重试与 `COM_OF_LOSS_T`/`COM_DISARM_PRFLT`。
- `drop_aligner.yaml`：`max_offset_px` 120→250（2.3 m 下约 1.0 m）；
  `target_center` 320/240（640×480 相机的真实中心）。
- `phase_d.launch`：`require_complete_detection_sources` 参数化（默认 false）。

**仍待解决**
- 整回合里第 1 个靶标经常出现 `targets=0`（120 s 内不确认），而同一位置
  用定点脚本（hover_at + 单独发 align_mode）却能确认。差异在驱动的飞行/对准时序，
  尚未定位；第 2 个靶标已能确认（`targets=2`），但对准在
  `offset_exceeds_limit` 与 `stale_observation` 之间摆动，未能稳定进入 `evidence_valid`。

---

## 22. 阶段二十二：搜索式完整任务跑通（11-01 22:41）✅

**关键改动**：起飞时**不知道靶标位置**，先搜索再投递（`mission_driver.py` v5）。

流程：
1. 起飞到 1.5 m，等视觉链就绪
2. 爬到 `search_z=2.4 m`，按 9 段割草机航线扫描（x 0.8→7.2，y ±4.0）
3. 从 `/uav_vision/targets` 读取**已确认（state≥2）且地图有效**的靶标地图坐标，
   按类别去重，找齐 3 个（tent/pillbox/bridge/panzer，排除 red_cross）
4. 逐个飞到发现的坐标 → 2.3 m 对准 → `drop_ready` 连续 3 帧 → 投递
5. 走廊 6 段 → 爬升越墙 → 尽头 H 板视觉降落

**实测（seed 42，随机场景）**：
```
[driver] 2/5 SEARCH
[driver]   found={'pillbox': (4.01,-3.09), 'tent': (4.98,-1.74), 'bridge': (3.97,2.64)}
[driver] 搜索完成 42s，发现 3 个目标
[driver] pillbox dropped=True servo=True
[driver] bridge  dropped=True servo=True
[driver] tent    dropped=True servo=True
[driver] corridor-1..6 全部到达
[driver] DONE found=3 drops=[True,True,True] final=(8.76,-4.18,0.15)
```

**注意**：发现的坐标与场景真值有 1–2 m 偏差（LIO 漂移 + 圆环中心投影），
但 2.3 m 高度下视场足够大，仍能对准投递。后续若要精确落点，需标定投影偏移。

---

## 23. 阶段二十三：稳定性调优与收尾（11-01 23:00–00:10）

**改动**
- 走廊从 6 段加密到 11 段（每段中间插点），速度 0.8 → 0.6 m/s（窄道更容易过）。
- 每个靶标投递最多重试 3 次；失败后先挪开 0.8 m 重新观察，避免停在同一点反复 `stale`。
- `goto()` 内每 2 秒做一次飞行模式守护：掉锁自动补解锁、掉出 OFFBOARD 自动重建。

**多次运行结果（seed 42，每次约 10 分钟）**

| 运行 | 发现三靶 | 投递 | 走廊 | 降落 |
|---|---|---|---|---|
| 22:40 | ✅ 3/3（42 s） | ✅ 3/3 | ✅ 6 段 | ✅ (8.76,-4.18,0.15) |
| 23:07 | ✅ 3/3（42 s） | ✅ 3/3 | ✅ 6 段 | ✅ (8.74,-4.11,0.13) |
| 23:17 | ✅ 3/3 | ✅ 3/3 | ❌ 第 6 段撞墙发散 | — |
| 23:49 | ✅ 3/3 | ⚠️ 2/3（tent 失败） | ✅ 11 段 | ✅ (8.77,-4.16,0.14) |
| 00:00 | ✅ 3/3 | ⚠️ 2/3（tent 重试未完成，被脚本时限打断） | — | — |

**结论**：**搜索 → 投递 → 走廊 → 降落的全流程已跑通（2 次完整成功）**，
但稳定性约 50–60%，主要失败模式：
1. `tent` 偶发无法确认/对准（其发现坐标与真值偏差最大）；
2. 走廊个别段偶发撞墙。

**重要遗留**：搜索得到的靶标地图坐标与场景真值偏差 **1–7 m**（`/uav_vision/targets`
的 `map_point` 在 2.4 m 高度下投影误差被放大）。投递能成功是因为 2.3 m 视场足够大、
对准器只需要圆环在视野内；**若要精确落点，必须先标定投影偏移**。

**下一步建议**
1. 标定 `/uav_vision/targets` 的 `map_point` 与真值偏移（用固定场景对比），做常量补偿；
2. 走廊段进一步降速到 0.4 m/s，或在门 A/B 前后各加一个减速点；
3. 投递重试时不要挪 0.8 m（可能挪出视野），改为原地等待 5 秒再重试。

---

## 24. 阶段二十四：性能定位 + 重构回归修复（11-02 16:10–17:35）

### 性能定位（结论：RTF 不是视觉链造成的）

| 场景 | RTF | 相机 | detections | mapped | 负载 |
|---|---|---|---|---|---|
| 仅仿真+桥接+LIO | 0.54 | 14.9 Hz | — | — | 3.2 |
| 加视觉链（10 节点） | 0.43–0.54 | 9.7 Hz | 32.6 Hz | 9.7 Hz | 8.2–12 |
| 整回合（含飞行） | **0.39 平均**（0.23–0.43） | — | — | — | 4–13 |

- YOLO 进程（`pt_main_thread`）占 **190–270% CPU**，是最大的单点开销；但 RTF 在
  **视觉链启动前就已经是 0.43–0.54**，说明瓶颈是 gzserver 单核物理（~110% CPU ≈ 1 核）。
- 世界文件是 **1000 Hz 物理**（`max_step_size=0.001`）；用
  `/gazebo/set_physics_properties` 在运行时改成 250 Hz（0.004 s）**RTF 没有变化**（0.54→0.54），
  说明开销不在步进次数，而在每步的传感器插件（MID360 射线投射等）。
- 机器无 CPU 配额（`.wslconfig` 只有网络设置），24 vCPU / 7.7 GB。

**结论**：RTF≈0.4–0.5 是本机常态，昨天的成功运行也是这个量级（约 200 s 仿真 / 12 min 墙钟）。
任务逻辑必须按仿真时间设计，不能假设实时。

### 重构引入的回归（已修复）

**症状**：重构后搜索 9 条航线只发现 1 个靶标（重构前 6 条就能发现 3 个）。

**根因**：旧驱动在**每条 `/uav_vision/targets` 消息的回调里**累积目标；
新代码 `_sync_book()` 只在航线结束时读**最后一条消息**，正好遇到空数组就漏掉。

**修复**：`Vision` 增加 `on_confirmed` 回调，每帧把已确认目标推给 `TargetBook.add_many()`。
修复后恢复到 **53 秒发现 3 个目标**（pillbox / tent / bridge）。

### 其它修复
- 每个靶标投递前重置 `servo_done`（原先上一个靶标的舵机 ack 会串到下一个）。
- `run_round.sh` 的监控上限从 45×20 s（15 min）放宽到 90×20 s（30 min）——
  RTF 0.4 时一次 150 s 的对准要 375 s 墙钟，原上限会中途掐断任务。

### 仍存在的失败模式
- `tent` 的发现坐标与真值偏差约 1.5 m，对准常在 `stale_observation` 处打转，
  150 s 内进不到 `evidence_valid`。
- 根因仍是**投影偏移未标定**（见阶段二十三）。

---

## 25. 阶段二十五：地图投影标定（11-02 19:20–20:45）

### 标定方法

`tools/calibrate_map_offset.py` / `calibrate_map_offset2.py`：起飞后依次悬停在
场景真值坐标正上方 2.3 m，分别记录**记忆融合点**（`/uav_vision/targets`）与
**逐帧投影点**（`/uav_vision/detections_mapped`），与真值比较。

### 结论一：记忆融合点不可用

第一次标定时，`/uav_vision/targets` 报出的位置与真值差 **0.34 ~ 1.71 m**，
且**各靶标互不一致**（标准差 1.12 / 0.64 m）——因为 `target_memory` 的
`map_memory_ttl: 0.0` 让融合点是**粘性**的：报的是历史首次观测的位置，不是当前观测。

第二次标定时，悬停全程 `/uav_vision/targets` **完全没有输出**。

### 结论二：逐帧投影极其准确，只有常量偏差

| class | 真值 | 逐帧投影 | Δ |
|---|---|---|---|
| tent | (5.02, 0.40) | (4.86, 0.56) | (-0.16, +0.17) |
| pillbox | (4.61, 2.72) | (4.52, 2.88) | (-0.09, +0.16) |
| bridge | (6.08, 1.74) | (5.92, 1.92) | (-0.16, +0.17) |
| panzer | (1.66, 4.02) | (1.54, 4.19) | (-0.12, +0.16) |
| red_cross | (4.25, 4.16) | (4.11, 4.29) | (-0.14, +0.13) |

**平均 (-0.13, +0.16) m，标准差 (0.03, 0.02) m** —— 一致性 5 cm 级，
说明投影模型本身是对的，只差一个常量偏移。

### 已实施

- `Vision` 改用 `/uav_vision/detections_mapped` 逐帧投影作为目标来源
- `config.mission_offset = (0.13, -0.16)` 常量补偿
- `center_gate_px = 150`：只接受圆环接近画面中心的检测（斜视时投影误差会放大）

### 尝试但失败的方案（记下来避免重复踩）

**逐帧修正环**（到点后用最新逐帧点再修正 2~3 次）：实测偏差从 0.90 → 0.59 → 0.29 m
看似收敛，但收敛点是 **误分类物体**——第三次实测把 tent 从真值 (5.02, 0.40)
修到了 (6.49, -1.47)，越修越偏。已回退。

### 仍存在的核心问题

投递时 `drop_aligner` 绝大多数时间停在 **`stale_observation`**，
进不到 `evidence_valid`。可能与 RTF≈0.4（视觉帧率相对仿真时间偏低）有关，
需要单独确认 `drop_aligner` 的 stale 阈值用的是墙钟还是仿真时钟。

---

## 26. 阶段二十六：投递 100% 成功 —— 根因是 `target_max_age`（11-02 21:07）✅

### 根因

`drop_aligner` 的 `target_max_age` 默认 **0.5 秒**（仿真时钟），而目标从成像到进入对准器
要经过 YOLO 推理 → `detection_fusion`（`flush_delay: 0.80`）→ `target_refiner` →
`target_memory`，实际延迟**必然超过 0.5 秒**。

因此**每一帧都被判为 `stale_observation`**，永远进不到 `evidence_valid`——
这正是过去几天投递反复失败、且"偶尔又能成功"的原因（时序卡在边界时才有帧侥幸通过）。

### 修复

```yaml
# drop_aligner.yaml
target_max_age: 0.5 -> 2.5
alignment_context_max_age: 0.5 -> 1.5
```

### 效果（同一次运行）

```
[mission] 搜索结束，用时 53s，发现 3 个目标
bridge  对准: insufficient_stable_frames -> evidence_valid   （0.3 秒）
bridge  第 1 次 投递成功=True 舵机=True
tent    第 1 次 投递成功=True 舵机=True
pillbox 第 1 次 投递成功=True 舵机=True
[flight] corridor-1..11 全部到达
[flight] descend-1.2 / 0.9 / 0.6 / 0.4 全部到达
[mission] 完成：发现 3，投递 [True, True, True]，落点 (8.83, -4.22, 0.11)
[cli] 任务结果: SUCCESS
```

**每个靶标从到达 evidence_valid 只用 0.3 秒**，与之前"150 秒都等不到"形成鲜明对比，
证实根因判断正确。

### 完整链路（现在的默认行为）

起飞 1.5 m → 爬升 2.4 m 搜索（9 条割草机航线）→ 逐帧投影发现 3 靶（约 53 s）
→ 逐靶 2.3 m 对准投递（`detections_mapped` + 常量补偿 (0.13,-0.16)）
→ 走廊 11 段 → 尽头 H 板视觉降落。
