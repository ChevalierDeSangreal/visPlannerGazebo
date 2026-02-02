# Tracking Visualizer - Elastic-Tracker 追踪可视化工具

## 概述

这个包为 Elastic-Tracker 提供实时追踪结果可视化和数据记录功能，方便分析和对比跟踪性能。

## 主要功能

- ✅ 实时显示无人机和目标的历史轨迹
- ✅ 可视化跟踪误差（距离、速度）
- ✅ 自动保存追踪数据到 CSV 文件
- ✅ 统计分析（均值、最大值、RMS）
- ✅ RViz 3D 可视化支持

## 快速开始

### 1. 编译

```bash
cd ~/wangzimo/Elastic-Tracker
catkin_make
source devel/setup.bash
```

### 2. 启动可视化

```bash
# 终端1：启动 Elastic-Tracker 仿真
roslaunch px4_test_framework px4_simulation.launch

# 终端2：启动 Elastic Tracker
roslaunch px4_test_framework elastic_tracking.launch

# 终端3：启动可视化节点
roslaunch tracking_visualizer tracking_visualizer.launch
```

### 3. RViz 可视化

```bash
rviz -d $(rospack find tracking_visualizer)/config/tracking_visualization.rviz
```

## 可视化内容

在 RViz 中可以看到：

- **蓝色轨迹**：无人机历史路径
- **红色轨迹**：目标历史路径
- **黄色连线**：当前跟踪误差
- **球体标记**：无人机（蓝色）和目标（红色）的当前位置
- **文本显示**：实时误差数值

## 发布话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/tracking_viz/drone_path` | `nav_msgs/Path` | 无人机历史轨迹 |
| `/tracking_viz/target_path` | `nav_msgs/Path` | 目标历史轨迹 |
| `/tracking_viz/error_markers` | `visualization_msgs/MarkerArray` | 误差可视化标记 |
| `/tracking_viz/distance_error` | `std_msgs/Float64` | 当前距离误差 [m] |
| `/tracking_viz/velocity_error` | `std_msgs/Float64` | 当前速度误差 [m/s] |

## 订阅话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/drone0/mavros/local_position/odom` | `nav_msgs/Odometry` | 无人机位置和速度 |
| `/target/odom` | `nav_msgs/Odometry` | 目标位置和速度 |

## 配置参数

编辑 `config/visualizer_params.yaml`：

```yaml
# 轨迹记录
max_trajectory_points: 2000   # 最大轨迹点数
visualization_rate: 10.0      # 可视化频率 [Hz]

# 数据保存
save_to_file: true            # 是否保存数据
output_file_path: "/tmp/elastic_tracker_tracking_data.csv"

# 话题配置
drone_topic: "/drone0/mavros/local_position/odom"
target_topic: "/target/odom"

# 颜色配置 (RGBA)
drone_color:   # 蓝色
  r: 0.0
  g: 0.0
  b: 1.0
  a: 1.0

target_color:  # 红色
  r: 1.0
  g: 0.0
  b: 0.0
  a: 1.0
```

## 输出数据

CSV 文件包含以下字段：
- `timestamp` - 时间戳
- `drone_x, drone_y, drone_z` - 无人机位置
- `target_x, target_y, target_z` - 目标位置
- `drone_vx, drone_vy, drone_vz` - 无人机速度
- `target_vx, target_vy, target_vz` - 目标速度
- `distance_error` - 位置误差
- `velocity_error` - 速度误差

## 数据分析示例

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
data = pd.read_csv('/tmp/elastic_tracker_tracking_data.csv')

# 绘制误差曲线
plt.figure(figsize=(12, 6))
plt.plot(data['timestamp'], data['distance_error'], linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Distance Error (m)')
plt.title('Elastic-Tracker Tracking Performance')
plt.grid(True)
plt.show()

# 打印统计信息
print(f"Mean error: {data['distance_error'].mean():.4f} m")
print(f"Max error: {data['distance_error'].max():.4f} m")
print(f"RMS error: {(data['distance_error']**2).mean()**0.5:.4f} m")
```

## 实时查看误差

```bash
# 距离误差
rostopic echo /tracking_viz/distance_error

# 速度误差
rostopic echo /tracking_viz/velocity_error
```

## 注意事项

1. 确保 Elastic-Tracker 正常运行并发布位置信息
2. 数据文件在节点关闭时自动保存
3. 可以通过修改参数调整可视化性能
4. CSV 文件默认保存在 `/tmp` 目录

