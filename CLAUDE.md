# nav2_simple_ws — GoToPose 导航 Action 工作空间

## 一句话

给坐标，导航过去。封装 Nav2 导航栈 + 坡面管理，对外暴露 `/go_to_pose` Action 和 `/desired_yaw` 话题。

## 仓库结构

```
nav2_simple_ws/
├── src/
│   ├── nav2_pose_navigator_interfaces/   # CMake 包 — 仅定义 GoToPose.action
│   │   └── action/GoToPose.action
│   └── nav2_pose_navigator/             # Python 包 — 全部业务逻辑
│       ├── nav2_pose_navigator/
│       │   ├── goto_pose_server.py       # Action 服务器（坡前对齐 + 调 Nav2 + 反馈距离）
│       │   ├── ramp_zone_manager.py      # 坡面管理（悬挂升降 + 锁yaw=0 + 最低速）
│       │   ├── odom_to_tf_node.py        # TF: map→nav_map (static) + nav_map→base_link (动态，来自 Odin)
│       │   └── cmd_vel_bridge.py         # 格式翻译: Twist → Float32MultiArray [vx,vy,wz]
│       │   └── map_viz_tool.py           # 交互式地图可视化（离线A* + 在线Nav2路径/代价地图）
│       ├── config/nav2_params.yaml       # Nav2 参数 (MPPI Omni + SMAC 2D + 代价地图)
│       ├── launch/bringup.launch.py      # 一键启动全部
│       ├── maps/                         # 放地图 (.pgm + .yaml), 启动时用
│       └── README.md                    # 完整文档（坐标系、红蓝差异、Python 示例）
```

## 世界坐标系

```
                    ◀── Y 负（红方）──│── Y 正（蓝方）──▶

  Y = -6    -5   -4   -3   -2   -1    0    1    2    3    4    5    6
  ──┬───────┬────┬────┬────┬────┬────┬┼┬────┬────┬────┬────┬────┬────┬──
X=0 ┤  武馆 (空)                      ││    武馆 (空)                     ← 场地边
X=2 ┤       ┌──────────┐              ││          ┌──────────┐           ← 梅林
X=3 ┤       │ 梅林区(红)│              ││          │ 梅林区(蓝)│
    │       │ X[3.2,8] │              ││          │ X[3.2,8] │
    │       │ Y[-4.8,  │              ││          │ Y[1.2,   │
    │       │   -1.2]  │              ││          │   4.8]   │
X=8 ┤       └──────────┘              ││          └──────────┘           ← 梅林结束
X=9 ┤  斜坡(红) X[9.3,10.8]           ││   斜坡(蓝) X[9.3,10.8]
X=9.3┤.........................───────┤├──────.........................┤  ← 擂台
X=9.45┤  ╔══平台墙══╗                 ││          ╔══平台墙══╗
    │   ║ X=9.45   ║                 ││          ║ X=9.45   ║          ← 高台
    │   ║ Y[-4.5,0]║                 ││          ║ Y[0,4.5] ║
X=12 ┤  场地边界墙(底)                 ││   场地边界墙(底)
  ──┴───────┴────┴────┴────┴────┴────┴┼┴────┴────┴────┴────┴────┴────┴──
    红方 origin: [0,-6,0]             │  蓝方 origin: [0,0,0]
```

- **X**: 0→12，武馆→擂台。**Y**: -6→+6，红负蓝正，Y=0 中线。
- 发目标 `frame_id="map"`，坐标直接用上图刻度。
- PGM: 240×120 px @ 0.05 m/px。┌─┐=障碍 ═=平台墙 ─=斜坡边界墙(WALL) ....=斜坡(非障碍)

## 数据流

```
GoToPose Action (x, y, yaw)
  │
  ▼
goto_pose_server → 目标在坡后(X≥9.3)时自动两段: ①坡前1m对齐(yaw=0) → ②上坡
  │  调用 Nav2 NavigateToPose (含 MPPI GoalAngleCritic 自动对朝向)
  │
  │  /cmd_vel (Twist)
  ▼
ramp_zone_manager   ← 拦截修改速度
  │  监听 /odin1/relocation → 进坡面: 升悬挂 + 锁yaw=0 + 最低速 / 出坡面: 降悬挂 + 释放yaw
  │  监听 /desired_yaw → 外部 yaw 锁定（P 控制器覆盖 wz），发 999.0 解除
  │
  │  /cmd_vel_adjusted (Twist)
  ▼
cmd_vel_bridge → /t0x0101_action → Odin 底盘
```

## Action 接口

```
/go_to_pose  (nav2_pose_navigator_interfaces/action/GoToPose)

请求:  target_pose (PoseStamped, frame_id="map")  — 要去哪
结果:  success (bool) + message (string)          — 成功/失败
反馈:  distance_remaining (float32)               — 剩余距离(米)

行为:  目标 X≥9.3 (坡后) 时自动两段导航:
       ① 先到 (ramp_x_min-1.0, goal_y, yaw=0) — 坡前对齐
       ② 再到最终目标 — 过坡时 ramp_zone_manager 自动锁 yaw=0
```

## 坡面管理 (ramp_zone_manager)

自动根据位置控制硬件：
- **进坡面** (X∈[9.3,10.8] + Y 在己方坡道范围 [红: -6.0~-4.5, 蓝: 4.5~6.0])：升起悬挂到 75mm，锁 yaw=0（正面朝坡），发 targetstate=-1 减速，强制 ≥0.25 m/s
- **出坡面**：降下悬挂到 30mm，释放 yaw 锁（Nav2 恢复旋转控制到目标朝向）
- **坡面最低速**：强制 ≥ 0.25 m/s（防溜车）
- **外部 yaw 锁定**：收到 `/desired_yaw` 后 P 控制器覆盖 Nav2 角速度；发 999.0 解除锁定

可通过参数配置：`team`, `ramp_x_min/max`, `ramp_y_min/max`(根据team自动), `min_ramp_speed`, `suspension_ramp/flat`, `yaw_kp`, `yaw_max_vel`

## 硬件话题（硬编码，和 nav2_ws/nav2_robocon 一致）

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/odin1/relocation` | 输入 | PoseStamped | 机器人定位 |
| `/odin1/odometry_highfreq` | 输入 | Odometry | 高频里程计 |
| `/cmd_vel` | Nav2 输出 | Twist | 原始速度 |
| `/cmd_vel_adjusted` | ramp_zone_manager 输出 | Twist | 调整后的速度 |
| `/desired_yaw` | 输入 | Float32 | 期望朝向（rad），发 999.0 取消锁定 |
| `/t0x0102_action` | 输出 | Float32MultiArray [h,h,h,h] | 悬挂高度 |
| `/targetstate` | 输出 | Int32 | 目标状态（-1=减速模式） |
| `/t0x0101_action` | 最终输出 | Float32MultiArray [vx,vy,wz] | 底盘速度 |

## 使用方式

```bash
# 构建
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# 红队启动
ros2 launch nav2_pose_navigator bringup.launch.py team:=red

# 蓝队启动
ros2 launch nav2_pose_navigator bringup.launch.py team:=blue

# 自定义地图（优先级最高）
ros2 launch nav2_pose_navigator bringup.launch.py map:=/path/to/custom.yaml

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
client.send_goal_async(goal)
```

## 依赖

构建时额外 pip 安装了 `empy`、`catkin_pkg`、`lark`（conda 环境缺少这些 ROS 2 构建依赖）。

Nav2 系统包（需 sudo apt）：
```
ros-jazzy-nav2-msgs ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller
ros-jazzy-nav2-smac-planner ros-jazzy-nav2-behaviors ros-jazzy-nav2-waypoint-follower
```

## 与 nav2_ws 的区别

本仓库是 nav2_ws/src/nav2_robocon 的提炼版：
- **保留**: odom_to_tf、cmd_vel_bridge、ramp_zone_manager、nav2_params.yaml、Nav2 启动、红蓝地图
- **移除**: 任务序列 (third_area_single)、机械臂/升降控制、手动调试 (goal_relay_node)
- **新增**: GoToPose Action 接口（含坡前自动对齐）；`team:=red/blue` 启动参数；ramp_zone_manager 进坡自动锁 yaw=0；map_viz_tool 在线模式 (--online) 实时显示 Nav2 路径和代价地图；斜坡边界墙 (generate_maps.py)
