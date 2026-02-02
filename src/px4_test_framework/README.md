# PX4 测试框架 - visPlanner 集成

## 概述

本包为 visPlanner 提供了 PX4+Gazebo 软件在环（SITL）测试框架，使 visPlanner 能够与 PX4 状态机正确交互。

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                  PX4 SITL + Gazebo 仿真                       │
│            (提供真实的飞行动力学和环境)                        │
└────────────────────┬─────────────────────────────────────────┘
                     │
                     ↓
┌──────────────────────────────────────────────────────────────┐
│                      MAVROS                                   │
│         (MAVLink ↔ ROS 通信桥梁)                             │
│  • /mavros/local_position/odom                               │
│  • /mavros/setpoint_raw/local                                 │
└────┬───────────────────────────────────────────────────┬─────┘
     │                                                   │
     ↓                                                   ↓
┌─────────────────────────┐              ┌─────────────────────────┐
│  状态机 (FSM)            │              │  visPlanner 规划器      │
│  • 自动起飞              │              │  • 轨迹规划              │
│  • 状态切换              │              │  • 可见性保证            │
│  • 自动降落              │              │    ↓                    │
└─────────────────────────┘              │  轨迹服务器             │
                                          │    ↓                    │
                                          │  PositionCommand        │
                                          │    ↓                    │
                                          │  MAVROS转换器           │
                                          │    ↓                    │
                                          │  PositionTarget         │
                                          └─────────────────────────┘
```

## 核心组件

### 1. 目标发布器 (`target_publisher_node`)

发布目标的位置和速度，支持多种运动模式：
- **静止模式** (mode=0): 固定位置
- **圆周模式** (mode=1): 圆形轨迹，带加速/减速
- **8字模式** (mode=2): 待实现

**输出**：
- `/target/odom`: 目标里程计（兼容 visPlanner）

### 2. 状态监听器 (`state_listener_node`)

监听状态机状态，当进入 TRAJ 状态时自动触发 visPlanner 开始跟踪。

**功能**：
- 监听状态机状态变化
- 进入 TRAJ 状态时发送触发信号
- 自动在指定时间后发送 END_TRAJ 命令

### 3. PositionCommand 到 MAVROS 转换器 (`pos_cmd_to_mavros_node`)

将 visPlanner 的 `quadrotor_msgs/PositionCommand` 转换为 `mavros_msgs/PositionTarget`，以便通过 MAVROS 控制 PX4。

### 4. 空地图发布器 (`empty_map_publisher_node`)

发布空地图（无障碍物）供 visPlanner 规划器使用。

## 使用方法

### 1. 编译

```bash
cd /home/core/wangzimo/visPlanner
catkin_make
source devel/setup.bash
```

### 2. 启动 PX4 SITL

**终端 1**：启动 PX4 SITL + Gazebo
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

### 3. 启动 MAVROS 和目标发布器

**终端 2**：启动 MAVROS + 目标发布器
```bash
cd /home/core/wangzimo/visPlanner
source devel/setup.bash
roslaunch px4_test_framework px4_simulation.launch
```

### 4. 启动 visPlanner 和状态机

**终端 3**：启动 visPlanner + 状态机
```bash
cd /home/core/wangzimo/visPlanner
source devel/setup.bash
roslaunch px4_test_framework visplanner_tracking.launch
```

### 5. 控制流程

1. **自动起飞**：状态机会自动解锁并起飞到指定高度（默认 1.5m）
2. **进入追踪模式**：发送命令切换到 TRAJ 状态
   ```bash
   rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 5"
   ```
3. **自动追踪**：状态监听器会自动触发 visPlanner 开始追踪目标
4. **自动结束**：追踪持续指定时间（默认 20 秒）后，状态监听器会自动发送 END_TRAJ 命令
5. **自动降落**：状态机会自动降落

## 状态机状态

- **INIT (0)**: 初始化状态
- **ARMING (1)**: 解锁和进入 offboard 模式
- **TAKEOFF (2)**: 自动起飞到指定高度
- **GOTO (3)**: 飞向指定位置（可选）
- **HOVER (4)**: 悬停
- **TRAJ (5)**: 追踪模式（由 visPlanner 控制）
- **END_TRAJ (6)**: 追踪结束，悬停等待
- **LAND (7)**: 自动降落
- **DONE (8)**: 完成

## 配置参数

### 目标轨迹参数

编辑 `config/target_params.yaml` 修改目标轨迹：

```yaml
mode: 1                    # 0: 静止, 1: 圆周, 2: 8字
circle_radius: 2.0         # 圆半径 [m]
circle_duration: 3.0       # 单圈时间 [s]
stationary_time: 3.0       # 初始静止时间 [s]
```

### 状态机参数

编辑 `src/offboard_state_machine/config/fsm_tracking.yaml` 修改状态机参数：

```yaml
takeoff_alt: 1.5          # 起飞高度 [m]
takeoff_time: 3.0         # 起飞时间 [s]
end_traj_wait_time: 5.0   # 追踪结束后的等待时间 [s]
```

## 话题列表

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/target/odom` | `nav_msgs/Odometry` | 目标位置和速度 |
| `/state/state_drone_0` | `std_msgs/Int32` | 状态机当前状态 |
| `/state/command_drone_0` | `std_msgs/Int32` | 状态机命令 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | visPlanner 位置命令 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | MAVROS 位置目标 |
| `/drone0/mavros/local_position/odom` | `nav_msgs/Odometry` | 无人机位置和速度 |

## 故障排除

### MAVROS 无法连接

1. 确认 PX4 正在运行：`pgrep -f px4`
2. 检查端口：`netstat -an | grep 14540`
3. 确认 fcu_url 参数正确

### 无人机不响应

1. 检查飞行模式：`rostopic echo /mavros/state`
2. 确认控制频率 > 2Hz
3. 检查状态机是否在 TRAJ 状态

### visPlanner 不开始追踪

1. 检查状态机是否进入 TRAJ 状态
2. 检查触发信号是否发送：`rostopic echo /move_base_simple/goal`
3. 检查目标是否发布：`rostopic echo /target/odom`

## 参考

- [visPlanner 主 README](../README.md)
- [Elastic-Tracker PX4 测试框架](../../Elastic-Tracker/PX4_TEST_FRAMEWORK_SUMMARY.md)
- [PX4 开发指南](https://docs.px4.io/)
- [MAVROS 文档](http://wiki.ros.org/mavros)

