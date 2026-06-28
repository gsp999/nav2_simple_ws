# nav2_pose_navigator

给一个坐标，机器人自己导航过去。封装 Nav2 导航栈，对外暴露 `/go_to_pose` Action 和 `/desired_yaw` 话题，并提供 RViz 可视化查看 MPPI/Nav2 规划和执行全过程。

## 坐标系

### 场地世界坐标系

**12m × 9.2m** 全场，X 轴向下（-0.4→11.6），Y 轴向右（-4.6→+4.6）。Y=0 中线分割红蓝。

```
                          ◀── Y 负半轴（红方）──│── Y 正半轴（蓝方）──▶
                          │                    │                    │
  Y = -4.6    -4    -3    -2    -1     0     1     2     3     4   4.6
  ──┬────────┬─────┬─────┬─────┬─────┬─┼─┬─────┬─────┬─────┬─────┬─────┬──
    │              │                    │     │     │                    │
X=-0.4┤ 武  馆  (空) │                    │  Y=0│     │   武  馆  (空)     │  ← 场地边界墙
    │  X∈[-0.4,2]  │                    │ 中线 │     │   X∈[-0.4,2]     │
    │              │                    │     │     │                    │
X=1 ┤              │                    │     │     │                    │
    │              │                    │     │     │                    │
X=2 ┤              │    梅 花 林 区      │     │     │    梅 花 林 区      │  ← 梅花林开始
    │              │   ┌──────────┐     │     │     │   ┌──────────┐     │
X=2.8┤              │   │ 桩台障碍  │     │     │     │   │ 桩台障碍  │     │
    │              │   │ X[2.8,7.6]│     │     │     │   │ X[2.8,7.6]│     │
X=3 ┤              │   │ Y[-3.4,  │     │     │     │   │ Y[-0.2,  │     │
    │              │   │   0.2]   │     │     │     │   │   3.4]   │     │
X=4 ┤              │   │          │     │     │     │   │          │     │
    │              │   │ (1200mm  │     │     │     │   │ (1200mm  │     │
X=5 ┤              │   │  桩台)   │     │     │     │   │  桩台)   │     │
    │              │   │          │     │     │     │   │          │     │
X=6 ┤              │   │          │     │     │     │   │          │     │
    │              │   └──────────┘     │     │     │   └──────────┘     │
X=7.6┤              │                    │     │     │                    │  ← 梅花林结束
    │              │                    │     │     │                    │
X=8.9┤              │  斜 坡 (红)        │     │     │  斜 坡 (蓝)         │
    │              │  X[8.9,10.4]       │     │     │  X[8.9,10.4]       │
    │..............├───────────────────┤     │     ├────────────────────┤  ← 擂台开始
    │              │ ╔══平台墙══╗       │     │     │ ╔══平台墙══╗       │
X=9.05┤              ├─╣ X=9.05   ║──────┤     │     ├─╣ X=9.05   ║──────┤  ← 竞技区高台
    │              │ ║ Y[-3.1,   ║      │     │     │ ║ Y[-1.4,  ║      │
    │              │ ║   1.4]   ║      │     │     │ ║   3.1]  ║      │
X=10 ┤              │ ╚═════════╝       │     │     │ ╚═════════╝       │
    │              │                    │     │     │                    │
X=10.4┤..............├───────────────────┤     │     ├────────────────────┤  ← 坡面结束
    │              │                    │     │     │                    │
X=11 ┤              │                    │     │     │                    │
    │              │                    │     │     │                    │
X=11.6┤  场 地 边 界 墙 (底)             │     │     │  场 地 边 界 墙 (底) │  ← 场地底部
  ──┴────────┴─────┴─────┴─────┴─────┴─────┴─┼─┴─────┴─────┴─────┴─────┴─────┴──
    │              │                    │  Y=0│     │                    │
    │  ▲ Y=-4.6   │                    │ 中线 │     │              Y=+4.6▲│
    │ 红方地图     │                    │     │     │     蓝方地图        │
    │ origin:     │                    │     │     │     origin:        │
    │ [-0.4,-4.6,0]│                   │     │     │     [-0.4,-1.4,0] │
    │ 240×120 px  │                    │     │     │     240×120 px     │
```

**图例：**
- `┌──┐` = PGM 障碍物（黑色，Nav2 不可通行）
- `....` = 斜坡范围（PGM 中为空地，仅 ramp_zone_manager 控制悬挂/限速）
- `═══` = 平台墙（PGM 障碍物）

**世界坐标规则：**
- **X**：-0.4 = 武馆边缘 → 11.6 = 擂台底边（单位 m）
- **Y**：-4.6 = 红方左侧边界，0 = 中线，+4.6 = 蓝方右侧边界（单位 m）
- **原点 (0,0)** = 武馆侧中线位置（odin1 坐标系原点）
- 发目标时 `frame_id` 写 `"map"`，直接使用上述世界坐标

**分场地图：**
| 地图 | origin | 覆盖范围 | PGM 尺寸 | 分辨率 |
|------|--------|---------|---------|--------|
| `field_red.yaml` | `[-0.4, -4.6, 0]` | X∈[-0.4,11.6], Y∈[-4.6,1.4] | 240×120 | 0.05 m/px |
| `field_blue.yaml` | `[-0.4, -1.4, 0]` | X∈[-0.4,11.6], Y∈[-1.4,4.6] | 240×120 | 0.05 m/px |

### 地图原点与 PGM 像素映射

地图 YAML 中的 `origin` 是 **PGM 图像左下角** 在世界坐标系中的位置（见上表）。

**PGM 像素 → 世界坐标的映射关系**（`generate_maps.py` 中实现）：

```
col = X / 0.05                    # 列：世界 X → 像素列
row = 119 - (Y - origin_y) / 0.05 # 行：世界 Y → 像素行（PGM 第0行=图像顶部=世界Y最大值）
```

以红方为例（origin_y = -4.6）：
- 世界 Y = -4.6 → row = 119（PGM 最底部）
- 世界 Y =  1.4 → row =   0（PGM 最顶部，场地边界）

### TF 坐标变换链

```
map                         ← 你发目标时用的 frame_id（世界坐标系）
 │  静态 identity TF        ← odom_to_tf_node 发布（map 和 nav_map 完全重合）
 ▼
nav_map                     ← Nav2 内部使用的坐标系
 │  动态 TF（来自 Odin）     ← odom_to_tf_node 订阅 /odin1/relocation 后发布
 ▼
base_link                   ← 机器人自身坐标系
```

> **为什么分 map 和 nav_map？** Nav2 写死使用 `nav_map` 作为 global_frame，但用户发目标用 `map` 更直观。identity TF 保证两者等价，ROS 自动处理转换。你不需要关心这个区别——目标点 frame_id 写 `"map"` 即可。

---

## 快速开始

### 1. 构建

```bash
cd ~/nav2_simple_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 2. 启动

```bash
# 红队
ros2 launch nav2_pose_navigator bringup.launch.py team:=red

# 蓝队
ros2 launch nav2_pose_navigator bringup.launch.py team:=blue

# 自定义地图（优先级最高，覆盖 team 选择）
ros2 launch nav2_pose_navigator bringup.launch.py map:=/path/to/custom.yaml

# 如需临时关闭 RViz 可视化辅助节点
ros2 launch nav2_pose_navigator bringup.launch.py team:=red enable_rviz_viz:=false
```

### 3. 打开 RViz 可视化

```bash
rviz2 -d install/nav2_pose_navigator/share/nav2_pose_navigator/rviz/mppi_nav2_visualization.rviz
```

如果在 Radxa 的 SSH 终端里看到：

```text
qt.qpa.xcb: could not connect to display
Could not load the Qt platform plugin "xcb"
```

说明当前终端没有图形显示环境。`rviz2` 是 GUI 程序，不能在普通无显示 SSH 里直接打开。推荐做法是：

```bash
# Radxa 上只启动导航和可视化话题发布
ros2 launch nav2_pose_navigator bringup.launch.py team:=red

# 在有桌面的电脑上打开 RViz，并连接同一个 ROS 2 网络
source install/setup.bash
rviz2 -d install/nav2_pose_navigator/share/nav2_pose_navigator/rviz/mppi_nav2_visualization.rviz
```

如果必须在 Radxa 上看 RViz，需要使用其中一种图形环境：
- 接显示器并登录桌面后运行 `rviz2`
- 用 VNC/远程桌面登录 Radxa 桌面后运行 `rviz2`
- 用 SSH X11 转发：`ssh -X radxa@radxa-airbox-q900`，并确认本机有 X server

排查命令：

```bash
echo $DISPLAY
```

如果输出为空，当前终端不能直接启动 RViz。

RViz 固定坐标系为 `nav_map`，默认显示：
- `/map`：静态地图
- `/global_costmap/costmap`、`/local_costmap/costmap`：全局/局部代价地图障碍物
- `/plan`：Nav2 全局规划路径
- `/trajectories`、`/optimal_trajectory`、`/transformed_global_plan`：MPPI 采样轨迹、最优轨迹、转换后的局部跟踪路径
- `/viz/robot_markers`：机器人位置、朝向箭头、原始/调整后速度箭头、速度文本
- `/viz/robot_trail`：机器人实际运动轨迹

> MPPI 可视化已在 `nav2_params.yaml` 中启用：`FollowPath.visualize: true`，`TrajectoryVisualizer.trajectory_step: 5`，`time_step: 3`。

### 4. 打开 map_viz_tool 在线可视化

如果不想用 RViz，也可以打开项目自带的 Matplotlib 窗口。它支持点选目标、请求 Nav2 规划、发送目标，并在 `--online` 模式下显示实时机器人位置、朝向、实际轨迹和速度面板。

```bash
# 红方地图 + 在线 ROS 2 数据
ros2 run nav2_pose_navigator map_viz_tool --team red --online

# 蓝方地图 + 在线 ROS 2 数据
ros2 run nav2_pose_navigator map_viz_tool --team blue --online

# 自定义地图
ros2 run nav2_pose_navigator map_viz_tool --map /path/to/custom.yaml --online
```

右侧面板显示：
- `pose/yaw`：来自 `/odin1/relocation`
- `raw /cmd_vel`：Nav2 原始速度
- `adj /cmd_vel_adjusted`：坡面管理后的速度
- `final /t0x0111`：最终发给底盘的 `[vx, vy, wz]`
- `Nav2 plan/trail/costmap`：路径点数、实际轨迹点数、costmap 状态

地图颜色：
- cyan：Nav2 规划路径 `/plan`
- orange：机器人实际运动轨迹
- red：机器人朝向
- blue：调整后速度方向

> `map_viz_tool` 也是 GUI 程序，同样需要桌面、VNC 或 X11 转发；普通无显示 SSH 里不能直接打开。

### 5. 发导航目标

```bash
# 一键发送 (x, y, yaw°) — 不需要手写四元数
ros2 run nav2_pose_navigator nav2_goal 7.0 2.0 90

# yaw 用弧度
ros2 run nav2_pose_navigator nav2_goal 7.0 2.0 1.57 --rad

# 自定义超时 (默认 60s)
ros2 run nav2_pose_navigator nav2_goal 7.0 2.0 90 --timeout 30
```

> 底层等价于 `ros2 action send_goal /go_to_pose ...`，但自动做 yaw→四元数转换，显示实时反馈和最终结果。

### 6. Python 调用

```python
from nav2_pose_navigator_interfaces.action import GoToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math

client = ActionClient(node, GoToPose, "go_to_pose")
goal = GoToPose.Goal()
goal.target_pose = PoseStamped()
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = 7.0
goal.target_pose.pose.position.y = 2.0
goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
client.send_goal_async(goal)
```

---

## 参数参考

### 启动参数（launch 文件传入）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `team` | `red` | `red` 或 `blue`，决定地图和坡道 Y 范围 |
| `map` | (空) | 自定义地图 YAML 路径，设置后覆盖 team 的地图选择 |
| `enable_rviz_viz` | `true` | 是否启动 `/viz/*` RViz 可视化辅助节点 |

### 坡面管理参数（ramp_zone_manager）

**代码位置**：[ramp_zone_manager.py](src/nav2_pose_navigator/nav2_pose_navigator/ramp_zone_manager.py) `__init__` 中的 `declare_parameter`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `team` | `red` | 由 launch 文件传入，决定坡道 Y 范围 |
| `ramp_x_min` | `8.9` | 坡面起始 X 坐标 (m) |
| `ramp_x_max` | `10.4` | 坡面结束 X 坐标 (m) |
| `min_ramp_speed` | `0.25` | 坡面上最小速度 (m/s)，防溜车 |
| `suspension_ramp` | `75.0` | 坡面上悬挂高度 (mm) |
| `suspension_flat` | `30.0` | 平地悬挂高度 (mm) |
| `yaw_kp` | `2.0` | 朝向锁定 P 控制器比例增益 |
| `yaw_max_vel` | `2.0` | 朝向锁定最大角速度 (rad/s) |

**根据 team 自动计算的 Y 范围**（硬编码在代码中，见下方"修改坡面 Y 范围"）：

| team | ramp_y_min | ramp_y_max |
|------|-----------|------------|
| `red` | -4.6 | -3.1 |
| `blue` | 3.1 | 4.6 |

### Nav2 导航参数

**代码位置**：[nav2_params.yaml](src/nav2_pose_navigator/config/nav2_params.yaml)

| 组件 | 插件 | 关键参数 |
|------|------|----------|
| 全局规划器 | SMAC Planner 2D | 最大规划时间 2s |
| 局部控制器 | MPPI Controller | Omni 模型, 56 时间步, 2000 采样/周期 |
| 最大速度 | — | vx/vy: 1.5 m/s, wz: 3.0 rad/s, vx_min: -1.0（可后退） |
| 目标容差 | SimpleGoalChecker | xy: 0.25m, yaw: 0.05 rad |
| 代价地图 | Static + Inflation | 分辨率 0.05m, 机器人半径 0.20m, 膨胀半径 0.40m |
| 局部代价地图 | rolling window | 5m × 5m 滚动窗口 |

### RViz 可视化参数（mppi_rviz_visualizer）

**代码位置**：[mppi_rviz_visualizer.py](src/nav2_pose_navigator/nav2_pose_navigator/mppi_rviz_visualizer.py)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `frame_id` | `nav_map` | RViz Marker/Path 发布坐标系 |
| `pose_topic` | `/odin1/relocation` | 机器人实时位姿输入 |
| `raw_cmd_topic` | `/cmd_vel` | Nav2 原始速度输入 |
| `adjusted_cmd_topic` | `/cmd_vel_adjusted` | 坡面管理后的速度输入 |
| `plan_topic` | `/plan` | Nav2 全局规划路径输入 |
| `trail_max_points` | `1000` | 实际运动轨迹最多保留点数 |
| `trail_min_distance` | `0.02` | 轨迹点最小间距 (m) |
| `publish_frequency_hz` | `10.0` | Marker/Path 发布频率 |
| `robot_radius` | `0.30` | RViz 中机器人圆柱半径 (m) |
| `heading_arrow_length` | `0.70` | 朝向箭头长度 (m) |
| `velocity_arrow_scale` | `0.60` | 速度箭头缩放系数 |

---

## 数据流

```
你发送 GoToPose Action (x, y, yaw)
         │
         ▼
   goto_pose_server
         │  调用 Nav2 NavigateToPose
         ▼
   Nav2 导航栈
     ├── planner_server   (SMAC 2D — 全局路径规划)
     └── controller_server (MPPI Omni — 局部轨迹跟踪 + 轨迹可视化)
         │  输出 /cmd_vel (Twist)
         ▼
   ramp_zone_manager (拦截修改)
         ├── 进坡面 → 升悬挂 + 限最低速
         ├── 监听 /desired_yaw → 锁朝向
         └── 输出 /cmd_vel_adjusted (Twist)
         ▼
   cmd_vel_bridge
         │  Twist → Float32MultiArray [vx, vy, wz]
         ▼
   /t0x0111_action → Odin 底盘执行

   mppi_rviz_visualizer (旁路监听，不参与控制)
     ├── /odin1/relocation → /viz/robot_trail + 机器人位置/朝向 Marker
     ├── /cmd_vel + /cmd_vel_adjusted → 速度箭头 + 速度文本 Marker
     └── /plan → /viz/global_plan + 路径 Marker
```

### Action 接口

| 项 | 内容 |
|----|------|
| Action 名 | `/go_to_pose` |
| 类型 | `nav2_pose_navigator_interfaces/action/GoToPose` |
| 请求 | `target_pose` (PoseStamped, frame_id="map") + `timeout_sec` (float32, 默认0=不限时) |
| 结果 | `success` (bool) + `message` (string) |
| 反馈 | `distance_remaining` (float32) — 剩余距离(米) |

### 硬件话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/odin1/relocation` | 输入 | PoseStamped | Odin SLAM 定位 |
| `/odin1/odometry_highfreq` | 输入 | Odometry | 高频里程计 |
| `/cmd_vel` | Nav2 输出 | Twist | 原始速度 |
| `/cmd_vel_adjusted` | ramp_zone_manager 输出 | Twist | 调整后速度 |
| `/desired_yaw` | 输入 | Float32 | 期望朝向(rad)，发 >900 或 NaN 取消锁定 |
| `/plan` | Nav2 输出 | Path | 全局规划路径 |
| `/trajectories` | MPPI 输出 | MarkerArray | MPPI 采样轨迹（RViz） |
| `/optimal_trajectory` | MPPI 输出 | Path | MPPI 当前最优轨迹 |
| `/transformed_global_plan` | MPPI 输出 | Path | MPPI 局部转换后的全局路径 |
| `/viz/robot_markers` | 可视化输出 | MarkerArray | 机器人、朝向、速度、状态文本 |
| `/viz/robot_trail` | 可视化输出 | Path | 实际运动轨迹 |
| `/viz/global_plan` | 可视化输出 | Path | 统一 frame 后的全局路径 |
| `/t0x0102_action` | 输出 | Float32MultiArray [h,h,h,h] | 悬挂高度 |
| `/targetstate` | 输出 | Int32 | 目标状态（-1=减速） |
| `/t0x0111_action` | 最终输出 | Float32MultiArray [vx,vy,wz] | 底盘速度 |

---

## 修改指南

### 改地图原点

如果你需要移动世界坐标系原点（例如把红方原点从 `[-0.4, -4.6, 0]` 改到其他位置），需要改 **3 个地方**：

#### 1. 地图 YAML — `origin` 字段

文件：`maps/field_red.yaml` / `maps/field_blue.yaml`

```yaml
# 红方当前：原点在 (-0.4, -4.6)，即 PGM 左下角对应世界 Y=-4.6
origin: [-0.4, -4.6, 0.0]

# 例如改成红方原点在 (0, -6)（和旧版一致）：
origin: [0.0, -6.0, 0.0]
```

> `origin: [ox, oy, yaw]` — ox/oy 是 PGM 左下角像素在世界坐标系中的位置。yaw 是地图旋转角（弧度），一般保持 0。

#### 2. 地图生成脚本 — `origin_y` 变量

文件：[generate_maps.py](generate_maps.py) 中的 `generate_red()` / `generate_blue()`

```python
def generate_red():
    oy = -4.6   # ← 改这个值，必须和 YAML origin[1] 一致
    ...

def generate_blue():
    oy = -1.4   # ← 同上
    ...
```

改完后重新生成：
```bash
python generate_maps.py
```

#### 3. 坡面 Y 范围 — ramp_zone_manager

文件：[ramp_zone_manager.py](src/nav2_pose_navigator/nav2_pose_navigator/ramp_zone_manager.py) 第 41-44 行

```python
if self.team == "blue":
    self.ramp_y_min, self.ramp_y_max = 3.1, 4.6
else:
    self.ramp_y_min, self.ramp_y_max = -4.6, -3.1
```

> 坡面 Y 坐标是世界坐标。如果原点变了但坡面实际位置没变，这里不用改。如果原点移动意味着世界坐标变了，则同步更新。

#### 不需要改的地方

- **`odom_to_tf_node.py`** — 发布的是 `map → nav_map` identity TF，和原点无关
- **`nav2_params.yaml`** — 使用 `nav_map` 作为 global_frame，不直接引用原点
- **`goto_pose_server.py`** — 只转发 PoseStamped，不关心原点
- **`bringup.launch.py`** — 不硬编码原点

### 改地图障碍物

#### 方法 1：修改 generate_maps.py（推荐）

文件：[generate_maps.py](generate_maps.py)

修改 `generate_red()` / `generate_blue()` 中的障碍物参数，然后：

```bash
python generate_maps.py
```

函数说明：
- `rect(img, x1, x2, y1, y2, origin_y)` — 填充矩形障碍（世界坐标，单位 m）
- `vline(img, y_w, x1, x2, oy, t)` — 竖线墙，t 是厚度(mm)
- `hline(img, x_w, y1, y2, oy, t)` — 横线墙

#### 方法 2：直接替换 PGM 文件

用自己的 PGM + YAML 替换 `maps/` 下的文件，然后用 `map:=` 指定：

```bash
ros2 launch nav2_pose_navigator bringup.launch.py map:=/path/to/your_map.yaml
```

PGM 要求：
- 格式：P5（binary），灰度 0-255
- 0 = 障碍物（黑色），254 = 空地（白色）
- 分辨率：0.05 m/pixel（和 YAML 中 resolution 一致）
- 尺寸：红蓝各 240×120 px（覆盖 12m×6m）

#### 方法 3：不重新生成，改 YAML 参数

如果只是想调整代价地图的膨胀/机器人半径（不改障碍物本身），改 [nav2_params.yaml](src/nav2_pose_navigator/config/nav2_params.yaml)：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.20       # 机器人半径 (m)
      inflation_layer:
        inflation_radius: 0.40  # 膨胀半径 (m)
        cost_scaling_factor: 2.0
```

### 改坡面参数

#### 改坡面 X 范围

文件：[ramp_zone_manager.py](src/nav2_pose_navigator/nav2_pose_navigator/ramp_zone_manager.py) 第 24-25 行

```python
self.declare_parameter("ramp_x_min", 8.9)
self.declare_parameter("ramp_x_max", 10.4)
```

或者在 launch 中覆盖（需要改 [bringup.launch.py](src/nav2_pose_navigator/launch/bringup.launch.py) 的 `ramp_zone_manager` Node，添加 `parameters`）：

```python
ramp_zone_manager = Node(
    ...
    parameters=[{"team": LaunchConfiguration("team"),
                 "ramp_x_min": 8.9,
                 "ramp_x_max": 10.4}],
)
```

#### 改坡面 Y 范围

文件：[ramp_zone_manager.py](src/nav2_pose_navigator/nav2_pose_navigator/ramp_zone_manager.py) 第 41-44 行

```python
# 当前：根据 team 硬编码
if self.team == "blue":
    self.ramp_y_min, self.ramp_y_max = 3.1, 4.6
else:
    self.ramp_y_min, self.ramp_y_max = -4.6, -3.1
```

#### 改其他坡面参数

同样在 `ramp_zone_manager.py` 中改默认值，或通过 launch 参数覆盖：

| 参数 | 行号 | 当前值 | 作用 |
|------|------|--------|------|
| `min_ramp_speed` | 26 | 0.25 m/s | 坡面最低速度 |
| `suspension_ramp` | 27 | 75.0 mm | 坡面悬挂高度 |
| `suspension_flat` | 28 | 30.0 mm | 平地悬挂高度 |
| `yaw_kp` | 29 | 2.0 | 朝向锁定 P 增益 |
| `yaw_max_vel` | 30 | 2.0 rad/s | 朝向锁定最大角速度 |

### 改 Nav2 导航行为

文件：[nav2_params.yaml](src/nav2_pose_navigator/config/nav2_params.yaml)

常见需要调的参数：

| 参数 | 位置 | 当前值 | 作用 |
|------|------|--------|------|
| `xy_goal_tolerance` | controller_server → goal_checker | 0.25 m | 到达容差 |
| `yaw_goal_tolerance` | controller_server → goal_checker | 0.05 rad | 朝向容差 |
| `vx_max` / `vy_max` | controller_server → FollowPath | 1.5 m/s | 最大线速度 |
| `wz_max` | controller_server → FollowPath | 3.0 rad/s | 最大角速度 |
| `max_planning_time` | planner_server → GridBased | 2.0 s | 最大规划时间 |
| `robot_radius` | local_costmap / global_costmap | 0.20 m | 机器人半径 |
| `inflation_radius` | inflation_layer | 0.40 m | 障碍物膨胀半径 |

---

## 常用朝向（yaw → 四元数）

| yaw (rad) | 角度 | z=sin(yaw/2) | w=cos(yaw/2) | 面向 |
|-----------|------|-------------|-------------|------|
| 0 | 0° | 0.0 | 1.0 | +X（擂台方向） |
| π/2 | 90° | 0.707 | 0.707 | +Y（蓝方方向） |
| -π/2 | -90° | -0.707 | 0.707 | -Y（红方方向） |
| π | 180° | 1.0 | 0.0 | -X（武馆方向） |

---

## 包结构

```
nav2_pose_navigator/
├── action/
│   └── GoToPose.action
├── nav2_pose_navigator/
│   ├── goto_pose_server.py        # Action 服务器（核心）
│   ├── ramp_zone_manager.py       # 坡面管理（悬挂 + 限速 + 锁朝向）
│   ├── odom_to_tf_node.py         # TF 广播（坐标系桥接）
│   └── cmd_vel_bridge.py          # 速度格式翻译
├── config/
│   └── nav2_params.yaml           # Nav2 完整参数
├── launch/
│   └── bringup.launch.py          # 一键启动
├── maps/                          # 地图文件 (.pgm + .yaml)
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 与 nav2_robocon (原仓库) 的区别

| 功能 | nav2_robocon | nav2_pose_navigator |
|------|-------------|-------------------|
| 导航 | 写死序列 | Action 动态发目标 |
| 坡面管理 | ✓ | ✓ |
| TF 广播 | ✓ | ✓ |
| 速度桥接 | ✓ | ✓ |
| Nav2 参数 | ✓ | ✓ |
| 机械臂/升降控制 | ✓ | ✗ |
| 手动目标中继 (goal_relay) | ✓ | ✗ |
