# visPlanner

## 0. 概述
**visPlanner** 是一个可见性感知的轨迹规划框架，可以处理空中追踪中目标的可见性问题。

**作者**: Qianhao Wang, Yuman Gao, Jialin Ji, Chao Xu 和 [Fei Gao](https://ustfei.com/) 来自 [ZJU Fast Lab](http://zju-fast.com/). 

**论文**: [Visibility-aware Trajectory Optimization with Application to Aerial Tracking](https://arxiv.org/abs/2103.06742),  发表于 IEEE International Workshop on Intelligent Robots and Systems (__IROS 2021__).

**视频链接**: [youtube](https://www.youtube.com/watch?v=PhhrOBx54YY) 或 [bilibili](https://www.bilibili.com/video/BV1vh411Q7G9/)

## 1. 仿真追踪

[注意] 请根据您的 GPU 修改 **src/uav_simulator/local_sensing/CMakeLists.txt** 中的 CUDA 选项。

### 准备和可视化
```bash
git clone https://github.com/ZJU-FAST-Lab/visPlanner.git
cd visPlanner
catkin_make
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

### 运行 visPlanner
```bash
roslaunch ego_planner tracking.launch
```

在 rviz 中使用 ``3D Nav Goal`` 触发目标无人机飞行，另一架无人机将追踪目标无人机：

<p align="center">
    <img src="figs/rviz.gif" width="700"/>
</p>

## 2. PX4+Gazebo 软件在环（SITL）测试

visPlanner 现在支持 PX4+Gazebo 软件在环测试，可以正确地和状态机交互。

### 使用方法

**终端 1 - 启动 PX4 SITL**：
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

**终端 2 - 启动 MAVROS 和目标发布器**：
```bash
cd visPlanner
source devel/setup.bash
roslaunch px4_test_framework px4_simulation.launch
```

**终端 3 - 启动 visPlanner 和状态机**：
```bash
cd visPlanner
source devel/setup.bash
roslaunch px4_test_framework visplanner_tracking.launch
```

**控制流程**：
1. 状态机会自动解锁并起飞到指定高度（默认 1.5m）
2. 发送命令切换到追踪模式：`rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 5"`
3. visPlanner 会自动开始追踪目标
4. 追踪持续指定时间（默认 20 秒）后，状态监听器会自动发送 END_TRAJ 命令
5. 状态机会自动降落

详细说明请参考 [px4_test_framework README](src/px4_test_framework/README.md)。

## 3. 状态机控制

visPlanner 现在集成了 PX4 offboard 控制状态机，用于自动化的起飞、追踪和降落流程。

### 使用方法（仿真环境）

**启动状态机**：
```bash
cd visPlanner
source devel/setup.bash
roslaunch offboard_state_machine single_drone.launch
```

### 状态机说明

状态机包含以下状态：
- **INIT (0)**: 初始化状态
- **ARMING (1)**: 解锁和进入 offboard 模式
- **TAKEOFF (2)**: 自动起飞到指定高度
- **GOTO (3)**: 飞向指定位置（可选）
- **HOVER (4)**: 悬停
- **TRAJ (5)**: 追踪模式（由 visPlanner 控制）
- **END_TRAJ (6)**: 追踪结束，悬停等待
- **LAND (7)**: 自动降落
- **DONE (8)**: 完成

### 状态切换命令

通过发布话题切换状态：
```bash
# 切换到追踪模式
rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 5"

# 结束追踪
rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 6"

# 降落
rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 7"
```

### 配置参数

编辑 `src/offboard_state_machine/config/fsm_tracking.yaml` 修改以下参数：
- `takeoff_alt`: 起飞高度（米）
- `takeoff_time`: 起飞时间（秒）
- `goto_max_vel`: 移动最大速度（米/秒）
- `landing_max_vel`: 降落最大速度（米/秒）
- `end_traj_wait_time`: 追踪结束后的等待时间（秒）

## 4. 追踪可视化

为了方便分析和对比追踪效果，我们提供了追踪结果可视化工具。

### 使用方法

**启动可视化节点**：
```bash
roslaunch tracking_visualizer tracking_visualizer.launch
```

**启动 RViz**：
```bash
rviz -d $(rospack find tracking_visualizer)/config/tracking_visualization.rviz
```

### 功能特性

- **实时轨迹显示**：无人机轨迹（蓝色）和目标轨迹（红色）
- **误差可视化**：黄色连线显示当前追踪误差，文本显示误差数值
- **数据记录**：自动保存追踪数据到 `/tmp/visplanner_tracking_data.csv`
- **统计分析**：实时计算平均误差、最大误差、RMS 误差

### 可视化话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/tracking_viz/drone_path` | `nav_msgs/Path` | 无人机历史轨迹 |
| `/tracking_viz/target_path` | `nav_msgs/Path` | 目标历史轨迹 |
| `/tracking_viz/error_markers` | `visualization_msgs/MarkerArray` | 误差可视化标记 |
| `/tracking_viz/distance_error` | `std_msgs/Float64` | 当前距离误差 [m] |
| `/tracking_viz/velocity_error` | `std_msgs/Float64` | 当前速度误差 [m/s] |

## 5. 完整使用流程

### 仿真环境下的完整追踪流程：

**终端 1 - 启动可视化**：
```bash
cd visPlanner
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

**终端 2 - 启动状态机**：
```bash
source devel/setup.bash
roslaunch offboard_state_machine single_drone.launch
```

**终端 3 - 启动追踪可视化**：
```bash
source devel/setup.bash
roslaunch tracking_visualizer tracking_visualizer.launch
```

**终端 4 - 启动 visPlanner 追踪**：
```bash
source devel/setup.bash
roslaunch ego_planner tracking.launch
```

**操作步骤**：
1. 等待无人机自动起飞并悬停
2. 在 rviz 中使用 `3D Nav Goal` 设置目标点
3. 发送命令切换到追踪模式：`rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 5"`
4. 观察追踪效果和可视化
5. 追踪结束后发送命令：`rostopic pub /state/command_drone_0 std_msgs/Int32 "data: 6"`
6. 无人机将自动降落

## 6. 致谢

本项目基于 [Ego-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) 和 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) 进行了改进，使轨迹优化框架工作得更好。

状态机和可视化功能参考了 [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) 的实现。

