# Gazebo 地图修改指南

> 前提：**不动 WSL**（另一个会话在用）。本文只讲怎么改，命令由你自己在合适的时候执行。
> 文件位置（WSL 内）：`~/liftrace_sim/final_worlds/`

---

## 1. 先看清现在两版到底是什么

两版的 world 文件都是 914 行、结构相同：`toudi2`（墙）→ 4 棵 juniper_Tree → 2 个 landing_h
→ `indoor_context`（场馆）→ 五个靶（一行 include）。

### A 版：`A_20cm_competition/worlds/field_original.world`

| 墙 | 行号 | pose (x y z r p y) | size (x y z) |
|---|---|---|---|
| Wall_1（南墙） | 36 | `0.0 -0.6 0.75 0 0 0` | `10.0 0.2 1.5` |
| Wall_9（北墙） | 57 | `0.0 9.2 0.75 0 0 0` | `10.0 0.2 1.5` |
| Wall_11（东墙） | 78 | `4.9 4.3 0.75 0 0 0` | `0.2 9.6 1.5` |
| Wall_12（西墙） | 99 | `-4.9 4.3 0.75 0 0 0` | `0.2 9.6 1.5` |
| Wall_13（走廊隔墙） | 120 | `0.75 7.5 0.75 0 0 0` | `8.1 0.2 1.5` |
| Wall_20_south（门洞段） | 141 | `-1.6 7.95 0.75 0 0 0` | `0.2 0.70 1.5` |
| Wall_22_north（门洞段） | 162 | `1.6 8.75 0.75 0 0 0` | `0.2 0.70 1.5` |

**特征**：外墙 10×10 外沿、**墙厚 0.20 m**、**墙高 1.5 m**、内净 9.6×9.6、走廊隔墙 1.5 m 高、
两处 0.8 m 门洞错列。**没有内置五靶**（原工程由 spawner 运行时布设）。

### B 版：`B_05cm_seed38_20261008/worlds/field_with_targets.world`

| 墙 | 行号 | pose | size |
|---|---|---|---|
| Wall_1（西墙） | 36 | `-0.525 0 2 0 0 0` | `0.05 10.1 4` |
| Wall_9（东墙） | 57 | `9.525 0 2 0 0 0` | `0.05 10.1 4` |
| Wall_11（南墙） | 78 | `4.5 -5.025 2 0 0 0` | `10 0.05 4` |
| Wall_12（北墙） | 99 | `4.5 5.025 2 0 0 0` | `10 0.05 4` |
| Wall_13（走廊隔墙） | 120 | `7.975 -0.75 0.75 0 0 0` | `0.05 8.5 1.5` |
| Wall_20_south | 141 | `9.15 1.6 0.75 0 0 0` | `0.7 0.05 1.5` |
| Wall_22_north | 162 | `8.35 -1.6 0.75 0 0 0` | `0.7 0.05 1.5` |
| 五靶 | 913 | 一行五个 `<include>` | — |

**特征**：内净 10×10、外沿 10.1、**墙厚 0.05 m**、**外墙高 4 m**（这是场馆墙，不是比赛隔离墙）、
走廊墙 1.5 m。五靶已静态布好。

### 结论：**A 版更接近规则**

规则是：场地 10×10×4 m、隔离墙高 **1.5 m**、通道/入口净宽 1.5 m、门洞 **0.8 m**。
A 的 1.5 m 矮墙 + 20 cm 厚 + 错列门洞就是这个样子；B 的 4 m 外墙是室内场馆的墙。

> 你现在觉得"差距大"，很可能是这两点：
> ① 用的是 B 版（4 m 高场馆墙 + `indoor_context` 大厅/天花板），
> ② 靶位是 seed 38 的随机布局，不是实际场地摆位。

---

## 2. 三种改法，按需要选

### 改法一：直接编辑 XML（最精确，推荐）

针对**尺寸/位置**的修改，改 `<pose>` 和 `<size>` 两个数就行。

```bash
# 先备份
cp ~/liftrace_sim/final_worlds/A_20cm_competition/worlds/field_original.world \
   ~/liftrace_sim/final_worlds/A_20cm_competition/worlds/field_original.world.bak
```

**坐标系约定（务必先确认）**
- A 版：`+x` 横向、`+y` 沿搜索方向；场地 x∈[-5,5]，y∈[-0.7,9.3]
- B 版：`x` 横向、`y` 搜索方向，原点在场地一角；场地 x∈[-0.5,9.5]，y∈[-5,5]
- **两版轴不同，航点不能混用**

**常见改动对照**

| 想改什么 | 改哪里 |
|---|---|
| 场地变大/变小 | 4 面外墙的 `pose` 和 `size`（A 版：Wall_1/9/11/12） |
| 墙厚 | 4 面外墙 `size` 的对应分量 + `pose` 补偿（墙厚变化时中心要跟着挪半厚） |
| 墙高 | `size` 的 z 分量 + `pose` 的 z（= 高度/2） |
| 门洞位置/宽度 | Wall_20_south / Wall_22_north 的 `pose`，以及 `size` 的第二分量 |
| 走廊宽度 | Wall_13 的 `pose`（x 决定墙在哪） |
| 删掉场馆/天花板 | 整段删掉 `<model name="indoor_context">` 到它的 `</model>`（B 版 569–912 行） |
| 删掉树 | 删 `juniper_Tree*` 的 4 段（B 版 182/276/370/464 行起） |
| 移动靶 | 最后一行五个 `<include>` 里的 `<pose>`（B 版 913 行） |
| 加靶到 A 版 | 把 B 版 913 行那一整行 `<include>…</include>` 拷到 A 版的 `</world>` 之前 |

### 改法二：在 Gazebo 界面里摆（直观，但精度低）

```
1. 只起 Gazebo（不要起 PX4）：
   gzserver <world 路径> --verbose &   # 或 gzclient 里 File→Open
2. 左上 "Edit" 进入编辑模式（或按 Ctrl+E）
3. 点选模型 → 用拖动/属性面板改位置
4. File → Save World As... 存成新文件
```

⚠️ 限制：墙是 `toudi2` 这个**模型内部的 link**，GUI 只能整体移动 `toudi2`，不能单独拖某一面墙。
要单独改墙还是得回到**改法一**。

### 改法三：参数化生成（最省事，适合反复调）

给我一组参数（场地长宽、墙厚、墙高、走廊位置、门洞位置与宽度、五个靶的坐标），
我写一个 `build_field.py`，生成 world 文件。以后改一个数字重跑即可，不用手改 XML。

这个脚本我可以直接写在 Windows 侧（`D:\deepseek-workspace\liftrace_sim_work\`），
你检查后自己拷进 WSL，全程不碰你正在跑的会话。

---

## 3. 改完之后必须同步的三处

地图改了，下面这些"绑定了旧坐标"的东西会失配：

| 文件（WSL 内） | 为什么要改 |
|---|---|
| `~/liftrace_sim/config/final_field_catalog.yaml` | 评测真值：五靶的 `fallback_center_world` 是世界坐标 |
| `~/liftrace_sim/config/patrol_field.yaml` | 自主航线航点（`waypoint_mode` 用的） |
| 任何 `site_waypoint_map_*.yaml` | 现场航点/门点/H 点 |

另外：**靶标模型名**（`random_tent` / `random_pillbox` / `random_bridge` / `random_panzer` /
`random_red_cross`）如果改了，真值目录里的 `gazebo_link_name` 也要跟着改。

---

## 4. 改完怎么快速看效果（不用起 PX4）

```bash
# 只起 Gazebo + 界面，最快
source ~/liftrace_sim/sim_env.sh
roslaunch gazebo_ros empty_world.launch gui:=true \
  world_name:=$HOME/liftrace_sim/final_worlds/A_20cm_competition/worlds/field_original.world
```

或者用包里自带的脚本：

```bash
cd ~/liftrace_sim/final_worlds
bash view_world.sh A     # 看 A 版
bash view_world.sh B     # 看 B 版
```

**抓图存档**（WSLg 下 `import -window root` 抓不到，要按窗口 ID）：

```bash
WID=$(DISPLAY=:0 xwininfo -root -tree | grep -oE '0x[0-9a-f]+ "Gazebo"' | head -1 | cut -d' ' -f1)
DISPLAY=:0 import -window "$WID" /tmp/field.png
cp /tmp/field.png /mnt/d/deepseek-workspace/liftrace_sim_work/out/
```

---

## 5. 我需要你确认的信息

要把它改到和真实场地一致，请给我这几项（有多少给多少）：

| 项 | 现在（A 版） | 实际场地 |
|---|---|---|
| 场地外沿 | 10.0 × 10.0 m | ? |
| 隔离墙厚 | 0.20 m | ? |
| 隔离墙高 | 1.50 m | ? |
| 内净尺寸 | 9.6 × 9.6 m | ? |
| 走廊净宽 | 1.5 m（Wall_13 在 x=0.75） | ? |
| 门洞宽 | 0.70 m × 2 处（错列） | ? |
| 门洞位置 | y≈7.95 / y≈8.75 | ? |
| 墙体数量/走向 | 4 面外墙 + 1 道走廊隔墙 | ? |
| 五靶摆位 | B 版 seed38 随机 | ? |
| 障碍物（树/柱/箱） | 4 棵树 + 场馆柱子 | ? |
| 坐标原点约定 | A：x 横向 / y 搜索方向 | ? |

如果方便，直接拍一张**实际场地的俯视草图标尺**（手绘也行），我照着生成 world 最准。
