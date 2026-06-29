# nav2_simple_ws — GoToPose 导航 Action 工作空间

## 一句话

给坐标，导航过去。封装 Nav2 导航栈 + 坡面管理，对外暴露 `/go_to_pose` Action 和 `/desired_yaw` 话题，并用 RViz 显示 MPPI/Nav2 规划、轨迹、机器人姿态和速度。

## 仓库结构

```
nav2_simple_ws/
├── src/
│   ├── nav2_pose_navigator_interfaces/   # CMake 包 — 仅定义 GoToPose.action
│   │   └── action/GoToPose.action
│   └── nav2_pose_navigator/             # Python 包 — 全部业务逻辑
│       ├── nav2_pose_navigator/
│       │   ├── goto_pose_server.py       # Action 服务器（调 Nav2 + 反馈距离；坡后目标默认直接发最终目标）
│       │   ├── ramp_zone_manager.py      # 坡面管理（进坡抬悬挂；巡航速度/角速度兜底；可选锁yaw/最低速）
│       │   ├── odom_to_tf_node.py        # TF: map→nav_map (static) + nav_map→base_link (动态，来自 Odin)
│       │   └── cmd_vel_bridge.py         # 格式翻译: Twist → Float32MultiArray [vx,vy,wz]
│       │   ├── map_viz_tool.py           # 交互式地图可视化（离线A* + 在线Nav2路径/代价地图）
│       │   └── mppi_rviz_visualizer.py   # RViz Marker/Path：机器人、朝向、速度、实际轨迹
│       ├── config/nav2_params.yaml       # Nav2 参数 (MPPI Omni + SMAC 2D + 代价地图)
│       ├── launch/bringup.launch.py      # 一键启动全部
│       ├── maps/                         # 放地图 (.pgm + .yaml), 启动时用
│       ├── rviz/                         # RViz 配置（MPPI/Nav2 全过程可视化）
│       └── README.md                    # 完整文档（坐标系、红蓝差异、Python 示例）
```

## 世界坐标系

```
                    ◀── Y 负（红方）──│── Y 正（蓝方）──▶

Y=-4.6  -4    -3   -2   -1    0    1    2    3    4   4.6
  ──┬───────┬────┬────┬────┬────┬┼┬────┬────┬────┬────┬────┬──
X=-0.4┤ 武馆(空)                     ││   武馆(空)                     ← 场地边
X=2 ┤       ┌──────────┐             ││         ┌──────────┐           ← 梅林
X=2.8┤       │ 梅林区(红)│             ││         │ 梅林区(蓝)│
    │       │X[2.8,7.6]│             ││         │X[2.8,7.6]│
    │       │Y[-3.4,   │             ││         │Y[-0.2,   │
    │       │   0.2]   │             ││         │   3.4]   │
X=7.6┤       └──────────┘             ││         └──────────┘           ← 梅林结束
X=8.9┤  斜坡(红) X[8.9,10.4]          ││  斜坡(蓝) X[8.9,10.4]
    │.........................───────┤├──────.........................┤  ← 擂台
X=9.05┤  ╔══平台墙══╗                ││         ╔══平台墙══╗
     │   ║ X=9.05   ║                ││         ║ X=9.05   ║          ← 高台
     │   ║Y[-3.1,1.4]║               ││         ║Y[-1.4,3.1]║
X=10.4┤  斜坡结束                     ││   斜坡结束
X=11.6┤  场地边界墙(底)                ││   场地边界墙(底)
  ──┴───────┴────┴────┴────┴────┴────┴┼┴────┴────┴────┴────┴────┴──
    红方 origin: [-0.4,-4.6,0]        │  蓝方 origin: [-0.4,-1.4,0]
```

- **X**: -0.4→11.6，武馆→擂台。**Y**: -4.6→+4.6，红负蓝正，Y=0 中线。
- 发目标 `frame_id="map"`，坐标直接用上图刻度。
- PGM: 240×120 px @ 0.05 m/px。┌─┐=障碍 ═=平台墙 ....=斜坡(非障碍)。PGM 四周边界由 `generate_maps.py` 强制刷成 WALL=0，Nav2 会当作静态障碍；抬高平台边界墙也保留为 WALL。
- **注意**：梅林区、平台边界墙障碍物已在真实尺寸基础上向外加宽约 0.1m（防碰撞），PGM 中使用的是加宽后尺寸。真实梅林区 X∈[2.8,7.6]，平台墙真实为 Y=±3.1 单线。详见 `generate_maps.py` 注释。

## 数据流

```
GoToPose Action (x, y, yaw)
  │
  ▼
goto_pose_server → 坡后目标默认直接发送最终目标（一段导航）；设置 enable_pre_ramp_alignment:=true 可启用两段
  │  调用 Nav2 NavigateToPose (含 MPPI GoalAngleCritic 自动对朝向)
  │
  │  /cmd_vel (Twist)
  ▼
ramp_zone_manager   ← 拦截修改速度
  │  监听 /odin1/relocation → 进坡面: 默认升悬挂 / 出坡面: 降悬挂
  │  监听 /desired_yaw → 外部 yaw 锁定（P 控制器覆盖 wz），发 999.0 解除
  │
  │  /cmd_vel_adjusted (Twist)
  ▼
cmd_vel_bridge → /t0x0111_action → Odin 底盘

旁路可视化:
  controller_server(MPPI visualize=true) → /trajectories + /optimal_trajectory + /transformed_global_plan
  mppi_rviz_visualizer → /viz/robot_markers + /viz/robot_trail + /viz/global_plan
```

## Action 接口

```
/go_to_pose  (nav2_pose_navigator_interfaces/action/GoToPose)

请求:  target_pose (PoseStamped, frame_id="map")  — 要去哪
       timeout_sec (float32, 默认 0)             — 总超时秒数，0 或负数 = 不限时
结果:  success (bool) + message (string)          — 成功/失败
反馈:  distance_remaining (float32)               — 剩余距离(米)

行为:  默认直接发送最终目标给 Nav2（一段导航）。
       若 enable_pre_ramp_alignment=true 且目标 X≥8.9，则先到
       (ramp_x_min-pre_ramp_offset, ramp_y_center, yaw=0)，再到最终目标。
       过坡时 ramp_zone_manager 根据实时位置抬升底盘。
       超时检测贯穿整次导航，超时后自动取消 Nav2 目标并返回失败。
```

## 坡面管理 (ramp_zone_manager)

自动根据位置控制硬件：

**坡面专属（默认行为）：**
- **进坡面** (X∈[8.9,10.4] + Y 在己方坡道范围 [红: -4.6~-3.1, 蓝: 3.1~4.6])：升起悬挂到 75mm
- **出坡面**：降下悬挂到 30mm

**全时巡航兜底（默认启用，不限于坡面）：**
- **线速度兜底**：导航活跃 + 距目标 > 0.25m 时，非零线速度被放大到 ≥1.50 m/s
- **角速度兜底**：导航活跃 + 无 yaw 覆盖时，非零角速度被放大到 ≥0.45 rad/s

**可选功能（默认关闭）：**
- 坡面锁 yaw=0（`enable_ramp_yaw_lock`）
- 坡面最低速 0.25 m/s（`enable_ramp_speed_floor`）
- 进/出坡发 targetstate=-1（`publish_ramp_target_state`）
- **外部 yaw 锁定**：收到 `/desired_yaw` 后 P 控制器覆盖 Nav2 角速度；发 999.0 解除锁定

可通过参数配置：`team`, `ramp_x_min/max`, `ramp_y_min/max`(根据team自动), `min_ramp_speed`, `suspension_ramp/flat`, `yaw_kp`, `yaw_max_vel`, `min_cruise_speed`, `min_cruise_angular_speed`, `cruise_slowdown_distance`

## 卡住检测与恢复

Nav2 Progress Checker：5 秒内未移动超过 5cm → 判定卡住 → 依次尝试恢复行为：
1. **Spin**（原地旋转）→ 2. **BackUp**（后退）→ 3. **Wait**（等待）
任一恢复成功 → 重新规划路径继续。全部失败 → 导航失败返回 success=false。

无物理碰撞传感器，"撞墙"通过 Progress Checker 间接检测。

## 硬件话题（硬编码，和 nav2_ws/nav2_robocon 一致）

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/odin1/relocation` | 输入 | PoseStamped | 机器人定位 |
| `/odin1/odometry_highfreq` | 输入 | Odometry | 高频里程计 |
| `/cmd_vel` | Nav2 输出 | Twist | 原始速度 |
| `/cmd_vel_adjusted` | ramp_zone_manager 输出 | Twist | 调整后的速度 |
| `/desired_yaw` | 输入 | Float32 | 期望朝向（rad），发 999.0 取消锁定 |
| `/plan` | Nav2 输出 | Path | 全局规划路径 |
| `/trajectories` | MPPI 输出 | MarkerArray | MPPI 采样轨迹（RViz） |
| `/optimal_trajectory` | MPPI 输出 | Path | MPPI 当前最优轨迹 |
| `/transformed_global_plan` | MPPI 输出 | Path | MPPI 转换后的局部全局路径 |
| `/viz/robot_markers` | 可视化输出 | MarkerArray | 机器人位置、朝向、速度箭头、速度文本 |
| `/viz/robot_trail` | 可视化输出 | Path | 实际运动轨迹 |
| `/viz/global_plan` | 可视化输出 | Path | 统一 frame 后的全局路径 |
| `/t0x0112_action` | 输出 | Float32MultiArray [h,h,h,h] | 悬挂高度 |
| `/targetstate` | 输出 | Int32 | 目标状态（-1=减速模式） |
| `/t0x0111_action` | 最终输出 | Float32MultiArray [vx,vy,wz] | 底盘速度 |

## ⚠️ 环境前置条件

**本机安装了 Miniconda，其 Python 3.13 会污染 PATH，导致 ROS 2 Jazzy（需要 Python 3.12）无法运行。**

使用 ROS 2 前必须确保系统 Python 3.12 优先：

```bash
# 方法 1：直接屏蔽 conda
conda deactivate
export PATH="/usr/bin:$PATH"          # 确保 /usr/bin/python3 (=3.12) 在最前面
source /opt/ros/jazzy/setup.bash

# 方法 2：每次都在子 shell 里操作
bash -c 'export PATH="/usr/bin:$PATH"; source /opt/ros/jazzy/setup.bash; colcon build'
```

如果遇到 `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`，说明 conda 的 Python 3.13 仍在 PATH 里。

## 使用方式

```bash
# 构建（确保 conda 已 deactivate 或 PATH 中 /usr/bin 在最前面）
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# 红队启动
ros2 launch nav2_pose_navigator bringup.launch.py team:=red

# 蓝队启动
ros2 launch nav2_pose_navigator bringup.launch.py team:=blue

# 自定义地图（优先级最高）
ros2 launch nav2_pose_navigator bringup.launch.py map:=/path/to/custom.yaml

# 临时关闭 RViz 可视化辅助节点
ros2 launch nav2_pose_navigator bringup.launch.py team:=red enable_rviz_viz:=false

# 打开 RViz 全过程可视化
rviz2 -d install/nav2_pose_navigator/share/nav2_pose_navigator/rviz/mppi_nav2_visualization.rviz

# 如果 SSH 里报 "qt.qpa.xcb: could not connect to display"
# 说明当前终端没有图形显示环境；Radxa 只跑 bringup，在有桌面的电脑/VNC/X11 转发里打开 rviz2。

# 打开 map_viz_tool 在线可视化（Matplotlib GUI）
# 显示地图、Nav2路径、costmap、机器人位置/朝向、实际轨迹、/cmd_vel 和 /t0x0111_action 速度面板
ros2 run nav2_pose_navigator map_viz_tool --team red --online

# 发目标
ros2 action send_goal /go_to_pose nav2_pose_navigator_interfaces/action/GoToPose \
  "{target_pose: {header: {frame_id: map}, \
    pose: {position: {x: 7.0, y: 1.5}, orientation: {z: 0.707, w: 0.707}}}}"
```

## Python 调用示例

```python
from nav2_pose_navigator_interfaces.action import GoToPose
from rclpy.action import ActionClient

client = ActionClient(node, GoToPose, "go_to_pose")
goal = GoToPose.Goal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = 7.0
goal.target_pose.pose.position.y = 2.0
goal.target_pose.pose.orientation.z = 0.707  # yaw=90°
goal.target_pose.pose.orientation.w = 0.707
goal.timeout_sec = 30.0  # 30 秒超时，0 或负数 = 不限时
client.send_goal_async(goal)
```

## 依赖

构建时额外 pip 安装了 `empy`、`catkin_pkg`、`lark`（conda 环境缺少这些 ROS 2 构建依赖）。

Nav2 系统包（需 sudo apt）：
```
ros-jazzy-nav2-msgs ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller
ros-jazzy-nav2-smac-planner ros-jazzy-nav2-behaviors ros-jazzy-nav2-waypoint-follower
```

RViz 可视化依赖 `visualization_msgs`，通常随 ROS/RViz 安装；固定坐标系使用 `nav_map`。

## 与 nav2_ws 的区别

本仓库是 nav2_ws/src/nav2_robocon 的提炼版：
- **保留**: odom_to_tf、cmd_vel_bridge、ramp_zone_manager、nav2_params.yaml、Nav2 启动、红蓝地图
- **移除**: 任务序列 (third_area_single)、机械臂/升降控制、手动调试 (goal_relay_node)
- **新增**: GoToPose Action 接口（坡后目标默认一段直达）；`team:=red/blue` 启动参数；ramp_zone_manager 进坡默认抬悬挂；map_viz_tool 在线模式 (--online) 实时显示 Nav2 路径和代价地图；RViz MPPI/Nav2 全过程可视化；地图外边界墙 (generate_maps.py)
