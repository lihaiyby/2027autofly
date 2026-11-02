# 工程结构（重构后）

```
2027autofly/
├── README.md
├── Makefile                  # make test / compile / dry-run / run
├── config/
│   └── mission.yaml          # 全部可调参数：高度、速度、航线、时限、重试
├── src/autofly/              # 可 import、可单测的任务包
│   ├── __init__.py
│   ├── config.py             纯数据：YAML → MissionConfig（无 ROS 依赖）
│   ├── search.py             纯逻辑：割草机航线、目标簿记、投递排序（无 ROS 依赖）
│   ├── flight.py             MAVROS 原语：解锁 / OFFBOARD / 设定点 / goto / 模式守护
│   ├── vision.py             /uav_vision 话题封装：targets / drop_ready / 舵机
│   ├── mission.py            编排：起飞 → 搜索 → 投递 → 走廊 → 降落
│   └── cli.py                命令行入口
├── scripts/
│   ├── sim/                  # 启动脚本（仿真、桥接、TF、RViz）
│   ├── mission/
│   │   ├── mission_driver.py # 兼容薄壳：转调 autofly.cli
│   │   └── run_round.sh      # 起仿真 + 跑一整回合
│   └── tools/                # scene_gen（场景随机化）、hover_at
├── tests/                    # 纯逻辑单测（17 个，不需要 ROS）
├── legacy/                   # 重构前的单文件驱动（回退用）
└── docs/
```

## 分层约定

| 层 | 规则 | 例 |
|---|---|---|
| 纯逻辑 `config / search` | **禁止 import rospy**，必须可单测 | `TargetBook`、`lawnmower`、`MissionConfig` |
| 设备层 `flight / vision` | 只包 ROS 接口，不含业务判断 | `Flight.goto()`、`Vision.confirmed()` |
| 编排层 `mission` | 只组合设备层，参数全部来自 `MissionConfig` | `Mission.run()` |
| 入口 `cli` | 解析参数、装配、调 mission | `python -m autofly.cli` |

## 常用命令

```bash
make test       # 17 个纯逻辑单测
make dry-run    # 打印配置与航线，不启动 ROS
make run        # 需要仿真已在跑
bash scripts/mission/run_round.sh 42          # 起仿真 + 跑一整回合
```

## 与旧结构的差别

| 旧 | 新 |
|---|---|
| 单文件 400 行驱动，飞行/视觉/业务流程混在一起 | 拆成 config / search / flight / vision / mission / cli 六层 |
| 高度、速度、航线硬编码在 Python 里 | 全部提到 `config/mission.yaml` |
| 无法单测（import 就依赖 ROS） | 纯逻辑层零 ROS 依赖，17 个单测秒级跑完 |
| 改航点要改代码 | 改 YAML，`--config` 可切换不同任务 |
| 失败只能看日志猜 | 每个 phase 返回布尔，`Mission.run()` 汇总；`dropped` 列表逐靶记录 |
