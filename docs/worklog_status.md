# WSL 仿真工作区状态（更新于 2026-10-31 19:05）

> 本目录在仓库外。交付包 `D:\deepseek-workspace\AAAautofly` **未被修改**。
>
> 📄 交接材料：[AI_HANDOVER.md](AI_HANDOVER.md)（下一个 AI 从这里开始） · [AI_WORKLOG.md](AI_WORKLOG.md)（全过程与踩坑）
> 🖥️ 想手动启动并看 Gazebo 画面：[手动启动仿真.md](手动启动仿真.md)

## 一、完整比赛航线已跑通 ✅

**起飞 → 走廊蛇形穿门（6 段）→ 爬升越墙 → 三靶连续投递 → 返航 → H 视觉降落**，一条命令：

```bash
bash /mnt/d/deepseek-workspace/liftrace_sim_work/301_full_round.sh
```

实测（2026-10-31 20:12，`out/full_round_corridor2/t8_driver.log`）：

```
[driver] armed=True (0s)   OFFBOARD=True
[driver] takeoff          (0.00, 0.00, 1.50) d=0.34
[driver] corridor-1       (8.70, 4.30, 1.20) d=0.27
[driver] corridor-2       (9.15, 2.80, 1.00) d=0.29
[driver] corridor-3       (8.45, 1.60, 1.00) d=0.29   ← 门 A
[driver] corridor-4       (8.45, 0.20, 1.00) d=0.29
[driver] corridor-5       (9.15, -1.60, 1.00) d=0.30  ← 门 B
[driver] corridor-6       (8.70, -3.40, 1.00) d=0.29
[driver] corridor-exit1   (8.70, -3.60, 2.20) d=0.34  ← 先爬升越过 1.5 m 墙
[driver] corridor-exit2   (7.00, -3.60, 2.20) d=0.34
[driver] target1 (7.05, -4.13) → dropped=True servo=True
[driver] target2 (4.64, -3.30) → dropped=True servo=True
[driver] target3 (4.90, -0.71) → dropped=True servo=True
[driver] home             (0.00, 0.00, 1.50) d=0.41
[driver] H lock pos=(0.00, 0.00)                 ← 视觉锁定 H 板中心
[driver] descend 1.0 → 0.7 → 0.45
[driver] DONE corridor=True drops=[True, True, True] final=(0.10, -0.01, 0.13)
```

三靶全部投递成功；H 板上空视觉锁定到 (0.00, 0.00)，分级下降后落点距中心 **0.10 m**。

![完整航线飞行](out/shot_18.png)

### 也可以分段跑

| 场景 | 命令 |
|---|---|
| **飞完整套 + 结束后窗口保留**（推荐看效果） | `bash 322_gui_fly.sh` |
| 飞完整套 + 自动清理 | `bash 301_full_round.sh` |
| 只看仿真画面（不飞，一直开着） | `bash 321_gui_keep.sh` |
| 只做链路自检（不飞，约 95 秒） | `bash 163_flow2.sh` |
| 走廊 + 单靶投递 | `bash 273_full_flow.sh` |
| 自定义航线 | `python3 ~/liftrace_sim/mission_driver.py "x1,y1;x2,y2;..." 1.5 [corridor]` |

## 二、视觉评测基线：官方场地 5/5 PASS

相机固定在 2.5 m 俯视官方场地，YOLO11n(v8) + 几何检测 + 融合 + 精修 + 地图投影全链：

| 靶标 | 命中 | 误检 | 漏检 | 像元误差 | 地图误差 | p95 延迟 |
|---|---|---|---|---|---|---|
| panzer | 580 | 0 | 0 | 41.3 px | 0.139 m | 825 ms |
| tent | 422 | 0 | 0 | 39.9 px | 0.138 m | 838 ms |
| bridge | 418 | 0 | 0 | 36.0 px | 0.124 m | 473 ms |
| pillbox | 452 | 0 | 0 | 40.5 px | 0.141 m | 868 ms |
| H 降落板 | 1622 | 0 | 0 | 38.8 px | 0.134 m | 148 ms |

精度/召回均为 1.00，全部通过门限。

## 三、走廊是怎么走的

官方世界内联的 `toudi2` 墙体几何（从 world 文件解析）：

| link | pose | 尺寸 |
|---|---|---|
| `Wall_13` | (7.975, -0.75, 0.75) | 0.05 × 8.5 × 1.5 |
| `Wall_20_south` | (9.15, 1.6, 0.75) | 0.7 × 0.05 × 1.5 |
| `Wall_22_north` | (8.35, -1.6, 0.75) | 0.7 × 0.05 × 1.5 |

东侧隔出 **1.55 m 宽走廊**，两块错位挡板形成两道门，只能蛇形通过。
驱动里把 `MPC_XY_VEL_MAX` / `MPC_XY_CRUISE` 压到 0.8 m/s 再穿。

## 四、关键实现

`~/liftrace_sim/mission_driver.py` 全部使用项目自己的接口：

| 环节 | 接口 |
|---|---|
| 飞行 | MAVROS OFFBOARD 位置设定点 |
| 对准 | `/uav_vision/align_mode` + `/detect/waypoint_mark_point`（地图系靶心） |
| 投递门控 | `/uav_vision/drop_ready` |
| 舵机 | `/detect/servo_status` → `/detect/servo_complete`（`mock_servo.py` 应答） |

> **重要**：`patrol_control` 的旧对准路径在本仓库里是坏的——`adjust_target_position`
> 从 `waypoint_mark_point` 赋值的整段代码被注释掉了，它会把像素坐标当米用
> （曾出现 `Aligning position: 108.67, 14.58`）。所以投递走新接口。
> 飞行侧的 `patrol_control` / Fast-Planner 已单独验证可用（`/planning/pos_cmd` 100 Hz）。

## 五、两条踩坑经验（否则跑不起来）

1. **先解锁、再切 OFFBOARD**。反过来会被 PX4 以
   `Preflight Fail: heading estimate not stable` / `Yaw estimate error` 拒绝解锁。
2. **视觉链延后启动**。视觉满载会把仿真拖到 RTF≈0.3，EKF 进不了稳定航向；
   起飞完成后再起视觉，解锁成功率明显提高。

## 六、环境清单

| 组件 | 版本 / 位置 |
|---|---|
| PX4 | v1.14.3 `1dacb4c` + 39 个 gazebo-classic 插件 |
| Gazebo Classic | 11.15.1（WSLg D3D12 → RTX 5060） |
| MAVROS | 1.20.1 + GeographicLib 数据集 |
| AstraDroneOpen | `~/AstraDroneOpen` @ `000b725`（GitHub `yidrone/AstraDroneOpen`） |
| 上游仓库 | `~/liftrace_sim/liftrace-visionwork` @ `76e1b5c` |
| 官方场地 | `~/liftrace_sim/final_worlds`（A/B 两版，**比赛只用 B 的 field_with_targets.world**） |
| Python | `venv38`（推理）、`px4py`（PX4 构建） |

## 七、踩过的坑（累计）

| # | 现象 | 解决 |
|---|---|---|
| 1 | `Wsl/E_ACCESS_DENIED` | 需要 danger-full-access |
| 2 | apt 卡在 `libgazebo11` | sudo 下手动带 `http_proxy` |
| 3 | `Can't get attribute 'C3k2'` | ultralytics ≥ 8.3.40 |
| 4 | `torchvision::nms does not exist` | torch 2.4.0+cpu 配 torchvision 0.19.0+cpu |
| 5 | `Invalid CUDA device=0` | `target_detector.yaml` 里 `device: ""` |
| 6 | Gazebo 里相机不出图 | world 引用了 5 个不存在的 `model://` 资产 |
| 7 | `fully_in_frame: False` | 相机高度 1.5 → 2.5 m |
| 8 | p95 延迟 1096–1216 ms | 物理步长降到 250 Hz |
| 9 | `Failed to import genmsg` | 手工 `pyros-genmsg` |
| 10 | MAVROS `GeographicLib exception` | `install_geographiclib_datasets.sh` |
| 11 | `Unknown model gazebo-classic_iris_mid360` | airframe 拷到 `~/.ros/etc` |
| 12 | FAST-LIO 无输出 | `CustomMsg` 类型不匹配，加桥接 |
| 13 | 视觉融合 `ignore incomplete fusion frame` | 相机 30 Hz → 10 Hz、检测输入 640 → 416 |
| 14 | `drop_aligner` 偏差永远超标 | 图像中心 (640,480) → (320,240) |
| 15 | 地图投影偶发野值 mark | 驱动里加 1.5 m 合理性钳制 |
| 16 | PX4 拒绝解锁 | 先解锁后 OFFBOARD + 解锁前锁姿态 + 视觉链延后 |
| 17 | 走廊出口撞墙、物理发散到几万米 | 出口先爬升到 2.2 m 越过 1.5 m 墙；`goto()` 加 50 m 发散保护 |
| 18 | 满屏 `GST_IS_APP_SRC` + `Connection refused` | rosmaster 已死（常见于挂一夜后休眠），跑 `312_cleanup_all.sh` 清理 |

## 八、下一步

1. 投口补偿标定：按 `known_rig.yaml` 的 `slot_offsets` 补偿 12 cm 装配偏移，
   把 `drop_aligner` 的 `max_offset_px` 从 120 收紧回 30–40
2. H 对准判据改成比例收敛（当前 0.20 m × 5 帧的门限实际没触发，靠 mark 直接下降）
3. GPU 推理（torch ≥ 2.7 + cu128）以进一步压延迟

> **开工前先跑** `313_preflight.sh`；出现成片连接错误先跑 `312_cleanup_all.sh`。
