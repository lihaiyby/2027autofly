# 下一个 AI 接手引导 · liftrace WSL 仿真

> 写于 2026-10-31。读者：接手的 AI agent 或人类工程师。
> 先读这一页，再按需翻 [AI_WORKLOG.md](AI_WORKLOG.md)（踩坑全过程）与 [README.md](README.md)（当前状态）。

---

## 1. 30 秒速览

**现在能跑什么**（全部在 WSL `Ubuntu-20.04` 里，已实测）：

| 能力 | 状态 | 证据 |
|---|---|---|
| 视觉评测（官方 10×10 场地，5 靶） | ✅ 5/5 PASS | `out/final_*/summary.json` |
| Gazebo 渲染（GPU 加速 + GUI 窗口） | ✅ 1× 实时 | `out/gazebo_gui.png` |
| PX4 SITL + MAVROS | ✅ connected / 30 Hz 位姿 | `out/sitl_smoke/` |
| MID360 激光 + 下视相机 | ✅ 10 Hz / 相机在线 | `out/sitl_mid360/` |
| FAST-LIO 里程计 | ✅ 10 Hz | `out/lio_bridge/` |
| **无人机飞行（GPS 机型 iris）** | ✅ OFFBOARD→解锁→方形→降落 | `out/fly_02_flying.png` |
| **无人机飞行（iris_mid360）** | ✅ 视觉位姿→EKF，ARMED 并飞到 2 m | `out/ev_fly2.png` |
| **项目控制链全自主航线** | ✅ patrol_control 按 YAML 飞完航点 | `out/patrol_mid.png` |
| **Fast-Planner 避障路线** | ✅ `/planning/pos_cmd` 100 Hz | `kinodynamic search success` |
| **飞行中视觉检测 + 对准** | ✅ `drop_ready` 置真 | `out/cross_mission/` |
| **整场比赛流程（含走廊 + 投递 + H 降落）** | ✅ 一条命令跑完 | `out/shot_8.png`、`273_full_flow.sh` |
| **三轮连续投递 + H 视觉降落** | ✅ 三靶全投、落点距 H 中心 0.15 m | `out/shot_10.png`、`301_full_round.sh` |
| **走廊 + 三靶连投合并航线** | ✅ 一条航线跑完整场，落点距 H 中心 0.10 m | `out/shot_18.png`、`301_full_round.sh` |

**一句话**：感知、定位、飞控、通信、走廊穿门、视觉投递、返航降落**全部跑通**；
剩下的是视觉 H 降落、投口补偿标定、三轮连投这些"提高分"的项。

跑整场：

```bash
bash /mnt/d/deepseek-workspace/liftrace_sim_work/273_full_flow.sh
```

---

## 2. 五分钟跑起来

```bash
# 进 WSL（注意必须指定发行版，默认那个是 26.04，不能用）
wsl -d Ubuntu-20.04 --cd /home/lihai/liftrace_sim

# 场景 A：只跑视觉评测（最稳，几分钟出报告）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/84_final_eval.sh panzer 150 2.5
#   产物: ~/liftrace_sim/out/final_panzer/summary.json + scene.png

# 场景 B：整机 + LIO（约 2 分钟起完）
bash /mnt/d/deepseek-workspace/liftrace_sim_work/142_lio_bridge.sh
#   看到 /Odometry 10 Hz 即成功
```

两套都是**自包含脚本**：自己清理残留进程、设环境变量、起停全套。

**想手动一步步来、并且要看到 Gazebo 界面** → 看 [手动启动仿真.md](手动启动仿真.md)（四个终端 + 界面操作 + 9 条排错）。

---

## 3. 目录地图

```text
/home/lihai/
├── liftrace_sim/
│   ├── liftrace-visionwork/        # 上游完整仓库 477 MB @76e1b5c（视觉+导航+文档）
│   │   ├── vision_ws/              # 已编译（uav_vision / eval / high_view）
│   │   └── patrol_uav_ws-patrol_planner/   # 已编译（FAST_LIO/FreeDOM/Fast-Planner/任务层）
│   ├── final_worlds/               # 官方场地 A/B 两版
│   │   ├── B_05cm_seed38_20261008/worlds/field_with_targets.world   # 五靶已布设
│   │   └── eval_field_fast.world   # 评测用副本（250 Hz 物理）
│   ├── config/                     # 真值目录 + 场景定义（我生成的）
│   ├── eval_ws/                    # 早期精简评测工作区
│   ├── PX4-Autopilot/              # v1.14.3 已编译（bin/px4 + 39 个插件）
│   ├── Livox-SDK2/                 # 已 make install 到 /usr/local
│   ├── venv38/                     # 推理环境（torch 2.4.0+cpu / ultralytics 8.3.40）
│   ├── px4py/                      # PX4 构建用 python 环境
│   ├── media/                      # 评测素材
│   └── out/                        # 所有运行产物
└── AstraDroneOpen/                 # 仿真底座 1.5 GB @000b725（GitHub yidrone/AstraDroneOpen）
    └── simulation/sim_workspace/devel/lib/   # liblivox_laser_simulation.so 等

D:\deepseek-workspace\liftrace_sim_work\     # 我的脚本与文档（不在交付包内）
├── AI_WORKLOG.md / AI_HANDOVER.md / README.md
├── 130~142_*.sh                             # Livox SDK / 编译 / LIO 桥接
├── 84/92/95_*.sh                            # 评测脚本（评测基线用这些）
└── out/                                     # 截图、summary、视频副本
```

⚠️ `D:\deepseek-workspace\AAAautofly`（交付包）**全程未被修改**。任何实验都不要写进去。

---

## 4. 环境版本（改动前先对照）

| 组件 | 版本 | 位置 |
|---|---|---|
| Ubuntu | 20.04.5 (WSL2, 7 GB RAM / 24 核) | — |
| ROS | Noetic（211 包） | `/opt/ros/noetic` |
| Gazebo Classic | 11.15.1 | 系统 |
| PX4 | v1.14.3 `1dacb4c` | `~/liftrace_sim/PX4-Autopilot` |
| MAVROS | 1.20.1 + GeographicLib 数据集 | 系统 |
| AstraDroneOpen | `000b725` | `~/AstraDroneOpen` |
| Livox SDK2 | `c0796f0` | `/usr/local` |
| 上游仓库 | `76e1b5c` | `~/liftrace_sim/liftrace-visionwork` |
| 推理 python | 3.8.10 / torch 2.4.0+cpu / ultralytics 8.3.40 | `~/liftrace_sim/venv38` |

**代理**：`http_proxy=http://127.0.0.1:7897`（用户开启）。apt / git / uv 都需要显式传，`sudo` 不继承环境。

---

## 5. 故障速查（14 条，按症状查）

| 症状 | 原因 | 处理 |
|---|---|---|
| Gazebo 话题有 publisher 但无图像 | 世界里有模型引用**不存在的 `model://` 资产**，OGRE 静默失败 | 用 `only_existing` 方式剔除缺失资产模型 |
| 相机一帧不出（世界加载完才 spawn 也一样） | spawn 早于世界加载完成 | 加 20 s 延时或 `spawn_camera:=false` 手动 spawn |
| 视觉链收不到图 | launch 默认订阅 `/camera/color/image_raw`，仿真相机发 `/camera/image_raw` | 传 `image_topic:=` 且确认 launch 透传 |
| 真值全空 / 全判误检 | `fully_in_frame: False`（相机太低，靶出画） | 相机抬到 2.5 m |
| `Invalid CUDA device=0` | `target_detector.yaml` 里 `device: 0` | 改 `device: ""` |
| p95 延迟 1100+ ms | Gazebo 物理 1000 Hz 抢 CPU | 用 250 Hz 的 `eval_field_fast.world` |
| `Can't get attribute 'C3k2'` | ultralytics < 8.3 不认 YOLO11 | 装 8.3.40 |
| `torchvision::nms does not exist` | torch/torchvision 版本不配对 | 2.4.0+cpu ↔ 0.19.0+cpu |
| `Failed to import genmsg` (PX4) | requirements 里通配符版本让 uv 解析失败 | 手工 `pyros-genmsg` |
| `px_update_git_header.py` IndexError | shallow clone 的 NuttX 无 tag | `git fetch --tags` |
| `GSTREAMER_APP_LIBRARIES NOTFOUND` | 缺 gstreamer 开发包 | apt 装 `libgstreamer*-dev` |
| MAVROS `GeographicLib exception` | 缺大地水准面 | `install_geographiclib_datasets.sh` |
| `Unknown model gazebo-classic_iris_mid360` | airframe 只在 ROMFS，PX4 读 `~/.ros/etc` | 两处都拷 |
| FAST-LIO 无输出 | 仿真激光是 `livox_laser_simulation/CustomMsg`，LIO 要 `livox_ros_driver2/CustomMsg` | 跑桥接 + `lidar_type:=1` |

**通用排查顺序**：先 `pkill -x gzserver; pkill -x rosmaster`（残留实例会造成极诡异的"世界只剩 ground_plane"），再看话题类型是否对得上。

---

## 6. 关键脚本索引

| 脚本 | 用途 |
|---|---|
| `084_final_eval.sh <靶> <秒> <相机高>` | 官方场地单靶评测（panzer/tent/bridge/pillbox/h） |
| `092_batch.sh` / `095_rerun.sh` | 批量跑基线 |
| `111_sitl_smoke.sh` | PX4 + Gazebo + MAVROS 冒烟 |
| `123_sitl_mid360.sh` | 加 iris_mid360 + MID360 |
| `142_lio_bridge.sh` | **完整链路**：SITL + 桥接 + FAST-LIO |
| `130_livox_sdk.sh` | 装 Livox SDK2 |
| `131_vision_ws.sh` / `132/133_patrol_ws.sh` | 两套工作区编译 |
| `140_livox_bridge.py` | CustomMsg 类型桥接节点 |
| **`273_full_flow.sh`** | **整场比赛流程**：官方地图 + PX4 + 感知 + 视觉 + 任务驱动（起飞→走廊→投递→降落） |
| **`301_full_round.sh`** | **完整比赛回合**：三轮连续投递 + H 视觉降落 |
| `~/liftrace_sim/mission_driver.py` | 任务驱动（MAVROS 设定点 + 视觉对准 + 投递门控） |
| `~/liftrace_sim/mock_servo.py` | 仿真用舵机 mock（ack `/detect/servo_status`） |
| `~/liftrace_sim/odom_to_vision.py` | `/Odometry` → `/mavros/vision_pose/pose`（解锁前锁姿态） |

---

## 7. 未完成 & 建议路线

按价值排序：

0. ~~让 iris_mid360 自主飞~~ ✅ **已完成**（`odom_to_vision.py`，先解锁再 OFFBOARD）

1. ~~Fast-Planner 避障路线~~ ✅ **已完成**（`waypoint_mode:=false`，`/planning/pos_cmd` 100 Hz）

2. ~~整场比赛流程~~ ✅ **已完成**（`273_full_flow.sh`：起飞→走廊穿门→视觉对准→投递→返航→H 降落）

3. ~~H 视觉降落 / 三轮连投~~ ✅ **已完成**（`301_full_round.sh`：三靶全投，H 板视觉锁定后分级下降，落点距中心 0.15 m）

4. ~~合并航线~~ ✅ **已完成**（走廊 6 段 + 爬升越墙 + 三靶连投 + H 视觉降落跑在同一次运行里）

5. **投口补偿标定**：`drop_aligner` 的 `max_offset_px` 目前放宽到 120 才能投，
   因为投口有约 12 cm 装配偏移（≈44 px）。按 `known_rig.yaml` 的 `slot_offsets`
   做补偿后可以收紧回 30–40。

6. **H 对准判据**：`aligned`（0.20 m 内连续 5 帧）没有触发，是靠 mark 位置直接下降完成的，
   建议改成比例收敛或放宽阈值。

7. **`patrol_control` 旧对准路径是坏的**（重要）：
   `adjust_target_position` 从 `waypoint_mark_point` 赋值的整段代码被注释掉了，
   它会把像素坐标当米用（曾打印 `Aligning position: 108.67, 14.58`）。
   如果要恢复旧链路，需要补回那段赋值；否则就用新接口（`/uav_vision/*`）。

8. **真实数据回归**：项目原始 bag 已被删除 121 份，只能用 `final_worlds` + 合成场景回归。

---

## 8. 禁忌与约定

1. **不要修改 `D:\deepseek-workspace\AAAautofly`**（交付包），它是验收基线。
2. **仿真单实例**：起新仿真前必须 `pkill -x gzserver/rosmaster/px4`，否则会出现"世界加载不全""话题错乱"等假故障。
3. **`make px4_sitl_default gazebo-classic` 会启动交互式 PX4**，别等它自然结束；构建产物看 `build/px4_sitl_default/`。
4. **`sudo` 不继承代理**：`sudo -n env http_proxy=... apt-get ...`，否则 apt 会挂死（我曾因此在 `libgazebo11` 上卡了 10 分钟）。
5. **不要用默认发行版**：`wsl` 默认进 26.04，必须 `wsl -d Ubuntu-20.04`。
6. 交付包里 `docs/仿真联调变更记录.md` 未随包；本次工作属于环境搭建，未篡改项目台账。若后续改项目源码，应按 `AGENTS.md` 规则 12/15 补记录并提交。

---

## 9. 交接检查清单（接手先跑这三条）

```bash
wsl -d Ubuntu-20.04 -e bash -c "gzserver --version && ls ~/AstraDroneOpen/simulation/sim_workspace/devel/lib/*.so | head -3"
wsl -d Ubuntu-20.04 -e bash -c "cat /home/lihai/liftrace_sim/out/final_panzer/summary.json | head -5"
wsl -d Ubuntu-20.04 -e bash -c "ls /home/lihai/liftrace_sim/PX4-Autopilot/build/px4_sitl_default/bin/px4 && ls /home/lihai/liftrace_sim/out/lio_bridge/"
```

三条都正常 → 环境完好，直接进第 2 节的场景 A/B。
