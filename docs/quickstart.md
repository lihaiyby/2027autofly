# 快速启动仿真（默认：官方新地图 + 纯 LIO）

## 一条命令

```bash
cd ~/2027autofly
bash scripts/mission/run_round.sh
```

默认行为：

| 项 | 默认值 | 说明 |
|---|---|---|
| 地图 | **官方固定地图** | `final_worlds/B_05cm_seed38_20261008/worlds/field_with_targets.world` |
| 定位 | **纯 FAST-LIO**（不使用真值） | `USE_GT=0` |
| 界面 | **开 Gazebo 窗口** | `GUI=true` |
| 目标 | red_cross / bridge / panzer | 起飞后搜索确认，再下降投递 |
| 投递高度 | 巡航 1.5 m → 投递 1.9 m → **释放 0.90 m（不触地）** | 高度上限硬夹 2.0 m |
| 走廊 | 11 段，高度 1.30 m | 门 A / 门 B 交替 |
| 降落 | 走廊尽头 H 板，1.2 m 后交 AUTO.LAND | |

## 常用变体

```bash
# 无界面（省资源，跑得更快）
GUI=false bash scripts/mission/run_round.sh

# 用 Gazebo 真值代替 LIO（仿真演示最稳，真机不可用）
USE_GT=1 bash scripts/mission/run_round.sh

# 回退到随机场景（旧行为）
USE_OFFICIAL=0 bash scripts/mission/run_round.sh 42
```

## 跑起来会看到什么

```
== T0 使用官方固定地图 ==
  world=.../field_with_targets.world
== T1 自检 + 清理 ==
== T2 仿真（GUI=true, world=field_with_targets.world）20:17:08 ==
== T3 任务：投递 -> 走廊 -> 尽头降落 ==
[mission] 1/5 起飞
[mission]   视觉链已就绪（已收 104 帧）
[mission] 2/5 搜索（起飞前不知道靶标位置）
[mission]   lane 1/9 已发现 {'red_cross': (0.88, -0.91)}
...
[mission] 3/5 投递
[mission]     panzer 像素对准完成 err=(-7, -20) px
[mission]     panzer 释放高度 z=0.90
[mission] 4/5 走廊（11 段）
[mission] 5/5 走廊尽头 H 板降落
[mission] 完成：发现 3，投递 [True, True, True]，落点 (8.74, -4.00, 0.13)
[cli] 任务结果: SUCCESS
```

`[watch Ns] x/y/z` 每 20 秒一行，是实时位置。

## 结果产物

每次运行写到 `/home/lihai/liftrace_sim/out/round2_<月日_时分>/`：

| 文件 | 内容 |
|---|---|
| `t9_driver.log` | 任务完整日志（同时打到终端） |
| `t1_world.log` … `t8_vision.log` | 各节点日志 |
| `pos.log` | 20 秒一次的位置采样 |
| `scene.log` | 随机场景模式的场景信息 |

## 已知问题

1. **LIO 偶发发散**（约 3 次里 1 次）：表现为某个航段位置估计突然跳到几十米外。
   已做缓解：高度硬夹 2.0 m、走廊 1.30 m、降落只降到 1.2 m。
   彻底规避可加 `USE_GT=1`（用真值，从未发散）。
2. **定位基线误差约 0.21 m**（实测，纯 LIO）。投递靠像素伺服闭环补偿。
3. 靶标圆环直径约 1.5 m，**低于约 2.1 m 会被画面裁掉**；但高于 2.1 m LIO 会发散，
   所以投递高度定在 1.9 m，靠像素伺服而非圆环精修来对准。
