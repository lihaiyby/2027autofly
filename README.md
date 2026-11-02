# 2027autofly · 投递无人机（WSL 仿真 + 视觉投递）

2027 赛季投递任务工程。当前阶段：**WSL2 仿真链路已打通**，可在官方 10×10 场地上完成
「起飞 → 三靶视觉投递 → 走廊穿越 → 走廊尽头降落」。

## 快速开始

```bash
# 0) 环境（每次新终端）
wsl -d Ubuntu-20.04 --cd /home/lihai/liftrace_sim
source ~/liftrace_sim/sim_env.sh

# 1) 自检（清残留 + 查内存/世界/ROS）
bash scripts/sim/preflight.sh

# 2) 看 Gazebo 画面（窗口常驻）
bash scripts/sim/gui_keep.sh

# 3) 看 RViz 建图（headless 仿真 + 点云）
bash scripts/sim/map_view_rviz.sh fly

# 4) 跑一整回合（随机场景 → 投递 → 走廊 → 尽头降落）
bash scripts/mission/run_round.sh 42 "panzer,pillbox,tent"
```

## 目录结构

```
scripts/
  sim/      起仿真与底层节点
            cleanup_all.sh   清残留进程
            preflight.sh     开工自检
            gui_keep.sh      只起 Gazebo 窗口（常驻）
            map_view_rviz.sh headless 仿真 + RViz 看建图
            livox_bridge.py  MID360 CustomMsg 类型桥接
            odom_to_tf.py    /Odometry → TF camera_init→body
            odom_to_vision.py /Odometry → PX4 视觉位姿
            camera_tf.py     body → 下视相机 TF
  mission/  任务层
            run_round.sh          一整回合（场景生成 + 起飞 + 投递 + 走廊 + 降落）
            run_round_keep_gui.sh 同上，跑完保留窗口
            run_round_autoclean.sh 同上，跑完自动清理
            mission_driver.py     任务驱动（MAVROS 设定点 + 视觉对准 + 投递门控）
            mock_servo.py         仿真舵机 mock
  tools/    场景与调试
            scene_gen.py   随机摆放 5 靶 + 4 树，输出 world + json
            hover_at.py    定点悬停（调视觉用）
docs/      手册、工作日志、交接材料
config/    航线 yaml
```

## 场景随机化

比赛每场靶标与树的位置会变。用 `scripts/tools/scene_gen.py`：

```bash
python3 scripts/tools/scene_gen.py --seed 42
# → ~/liftrace_sim/scenes/scene_42.world   传给 world:= 参数
# → ~/liftrace_sim/scenes/scene_42.json    靶标/树坐标（任务驱动据此飞）
```

约束：场地内 0.6 m 边距、避开东侧走廊、避开两块 H 板、靶间 ≥1.3 m、靶与树 ≥1.0 m。
墙体（`toudi2` 的 7 段 Wall_*）不会被改动。

## 当前状态

| 能力 | 状态 |
|---|---|
| PX4 SITL + MAVROS + Gazebo + MID360 + 下视相机 | ✅ |
| FAST-LIO 定位 + 视觉位姿喂 EKF | ✅ |
| 视觉评测（官方场地 5 靶） | ✅ 5/5 PASS |
| 三靶视觉投递 | ⚠️ 在**随机场景**下 `no_targets`，未投出（旧固定场景可投） |
| 走廊蛇形穿越（6 段） | ✅ |
| 走廊尽头 H 板视觉降落 | ✅ 落点 (8.79, −4.18) |
| 随机场景（5 靶 + 4 树） | ✅ `scene_gen.py` |

## 已知问题

1. **随机场景下投递失败**：`/uav_vision/targets` 在对准时为空 → `drop_ready reason=no_targets`。
   定点悬停测试里同一位置视觉链正常（`panzer` state=2、`map_valid: True`、10.4 Hz），
   说明问题在 aligner 的取值时序，待查。
2. 投口 12 cm 装配偏移未补偿，`drop_aligner.max_offset_px` 暂放宽到 120。
3. 视觉链满载时仿真 RTF 会掉到 0.3–0.5；解锁与起飞要在轻载阶段完成。

## 运行环境

- WSL2 `Ubuntu-20.04` + ROS Noetic + Gazebo Classic 11.15.1（WSLg 直通，无需 X Server）
- PX4 v1.14.3 SITL、MAVROS 1.20.1
- 上游仓库 `~/liftrace_sim/liftrace-visionwork`、仿真底座 `~/AstraDroneOpen`
- 详细环境与踩坑见 `docs/manual_start.md`、`docs/ai_worklog.md`

## 工程结构

```
src/autofly/        任务包（可 import、可单测）
  config.py         YAML -> MissionConfig，无 ROS 依赖
  search.py         割草机航线 + 目标簿记 + 投递排序，无 ROS 依赖
  flight.py         MAVROS 原语：解锁 / OFFBOARD / 设定点 / 模式守护
  vision.py         /uav_vision 话题封装
  mission.py        编排：起飞 -> 搜索 -> 投递 -> 走廊 -> 降落
  cli.py            命令行入口
config/mission.yaml 全部可调参数（高度、速度、航线、时限、重试）
tests/              17 个纯逻辑单测，秒级跑完，不需要 ROS
scripts/            启动脚本与兼容薄壳
legacy/             重构前的单文件驱动（回退用）
```

常用：

```bash
make test       # 单测
make dry-run    # 打印配置与航线（不启动 ROS）
make run        # 跑任务（需仿真已启动）
bash scripts/mission/run_round.sh 42   # 起仿真 + 一整回合
```

细节见 `docs/architecture.md`。
