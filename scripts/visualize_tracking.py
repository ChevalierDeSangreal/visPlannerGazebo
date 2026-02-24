#!/usr/bin/env python3
"""
visPlanner 跟踪性能可视化脚本
参考 Elastic-Tracker 的可视化实现
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def main():
    # 数据文件路径
    # data_file = '/tmp/elastic_tracker_tracking_data.csv'
    data_file = '/home/core/wangzimo/visPlanner/test_log/typeD.csv'
    
    # 检查文件是否存在
    if not os.path.exists(data_file):
        print(f"❌ 错误: 数据文件不存在: {data_file}")
        print("   请确保在TRAJ状态下运行了tracking_visualizer")
        sys.exit(1)
    
    # 读取数据
    print(f"📊 正在读取数据: {data_file}")
    data = pd.read_csv(data_file)
    
    # 打印数据基本信息
    print(f"✅ 数据总行数: {len(data)}")
    print(f"   时间戳范围: {data['timestamp'].min():.2f} - {data['timestamp'].max():.2f}")
    print(f"   时间跨度: {data['timestamp'].max() - data['timestamp'].min():.2f} 秒")
    
    # 检查是否有NaN值
    print(f"\n🔍 缺失值检查:")
    nan_counts = data.isna().sum()
    if nan_counts.sum() > 0:
        print(f"   发现缺失值:")
        for col in data.columns:
            if nan_counts[col] > 0:
                print(f"     {col}: {nan_counts[col]}")
        # 删除包含NaN的行
        data = data.dropna()
        print(f"   删除NaN后数据行数: {len(data)}")
    else:
        print(f"   ✓ 没有缺失值")
    
    if len(data) == 0:
        print("❌ 错误: 没有有效数据")
        sys.exit(1)
    
    # 确保数据按时间排序
    data_sorted = data.sort_values('timestamp').reset_index(drop=True)
    time_sorted = data_sorted['timestamp'] - data_sorted['timestamp'].iloc[0]
    
    # 检查时间戳是否有效
    if time_sorted.max() <= 0.0:
        print("⚠️  警告: 所有时间戳相同，使用数据索引作为时间轴")
        estimated_duration = len(data_sorted) / 10.0  # 假设10Hz采样
        time_sorted = np.arange(len(data_sorted)) / 10.0
        print(f"   使用估算时间轴: 0.00 - {time_sorted.max():.2f} 秒 (假设10Hz采样)")
    else:
        print(f"✅ 时间范围: {time_sorted.min():.2f} - {time_sorted.max():.2f} 秒")
    
    # 计算各项指标
    print(f"\n📐 计算跟踪指标...")
    
    # 1. 距离误差（已经在CSV中计算好了）
    distance_sorted = data_sorted['distance_error']
    
    # 2. 速度差异（已经在CSV中计算好了）
    velocity_error_sorted = data_sorted['velocity_error']
    
    # 3. 计算无人机机体x轴与到目标方向的夹角
    dx_sorted = data_sorted['target_x'] - data_sorted['drone_x']
    dy_sorted = data_sorted['target_y'] - data_sorted['drone_y']
    dz_sorted = data_sorted['target_z'] - data_sorted['drone_z']
    
    drone_roll_sorted = data_sorted['drone_roll']
    drone_pitch_sorted = data_sorted['drone_pitch']
    drone_yaw_sorted = data_sorted['drone_yaw']
    
    # 使用ZYX欧拉角顺序（yaw-pitch-roll）构建旋转矩阵
    cos_roll = np.cos(drone_roll_sorted)
    sin_roll = np.sin(drone_roll_sorted)
    cos_pitch = np.cos(drone_pitch_sorted)
    sin_pitch = np.sin(drone_pitch_sorted)
    cos_yaw = np.cos(drone_yaw_sorted)
    sin_yaw = np.sin(drone_yaw_sorted)
    
    # 将目标方向向量转换到机体坐标系
    direction_body_x = (cos_yaw * cos_pitch * dx_sorted + 
                       sin_yaw * cos_pitch * dy_sorted - 
                       sin_pitch * dz_sorted)
    direction_body_y = ((cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * dx_sorted +
                       (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * dy_sorted +
                       cos_pitch * sin_roll * dz_sorted)
    direction_body_z = ((cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * dx_sorted +
                       (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * dy_sorted +
                       cos_pitch * cos_roll * dz_sorted)
    
    # 归一化机体坐标系中的方向向量
    direction_body_norm = np.sqrt(direction_body_x**2 + direction_body_y**2 + direction_body_z**2)
    direction_body_x_normalized = direction_body_x / (direction_body_norm + 1e-8)
    
    # 计算机体x轴[1,0,0]与目标方向的夹角
    cos_angle = np.clip(direction_body_x_normalized, -1.0, 1.0)
    angle_rad_sorted = np.arccos(cos_angle)
    angle_deg_sorted = np.degrees(angle_rad_sorted)
    
    # 检测目标物体开始移动和停止移动的时刻
    target_velocity_magnitude_sorted = np.sqrt(data_sorted['target_vx']**2 + 
                                               data_sorted['target_vy']**2 + 
                                               data_sorted['target_vz']**2)
    velocity_threshold = 0.05
    is_moving = target_velocity_magnitude_sorted > velocity_threshold
    
    # 找到开始移动的时刻（从静止到移动的转变）
    start_moving_indices = []
    for i in range(1, len(is_moving)):
        if not is_moving[i-1] and is_moving[i]:
            start_moving_indices.append(i)
    
    # 找到停止移动的时刻（从移动到静止的转变）
    stop_moving_indices = []
    for i in range(1, len(is_moving)):
        if is_moving[i-1] and not is_moving[i]:
            stop_moving_indices.append(i)
    
    # 提取对应的时间
    start_moving_times = [time_sorted.iloc[i] for i in start_moving_indices]
    stop_moving_times = [time_sorted.iloc[i] for i in stop_moving_indices]
    
    print(f"✅ 指标计算完成")
    print(f"   开始移动时刻: {start_moving_times} 秒")
    print(f"   停止移动时刻: {stop_moving_times} 秒")
    
    # 创建图表
    print(f"\n🎨 生成可视化图表...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    
    # 子图1: 到目标物体距离随时间变化
    axes[0].plot(time_sorted, distance_sorted, linewidth=2, color='blue', label='Distance to Target')
    axes[0].axhline(y=distance_sorted.mean(), color='r', linestyle='--', 
                    label=f'Mean: {distance_sorted.mean():.4f} m', linewidth=1.5)
    axes[0].fill_between(time_sorted, 
                         distance_sorted.mean() - distance_sorted.std(),
                         distance_sorted.mean() + distance_sorted.std(),
                         alpha=0.2, color='blue', label=f'±1σ: {distance_sorted.std():.4f} m')
    # 标记目标开始移动和停止移动的时刻
    for t in start_moving_times:
        axes[0].axvline(x=t, color='green', linestyle=':', linewidth=2, alpha=0.7)
    for t in stop_moving_times:
        axes[0].axvline(x=t, color='red', linestyle=':', linewidth=2, alpha=0.7)
    # 添加图例说明
    if start_moving_times:
        axes[0].axvline(x=-999, color='green', linestyle=':', linewidth=2, label='Target Start Moving')
    if stop_moving_times:
        axes[0].axvline(x=-999, color='red', linestyle=':', linewidth=2, label='Target Stop Moving')
    axes[0].set_xlabel('Time (s)', fontsize=12)
    axes[0].set_ylabel('Distance (m)', fontsize=12)
    axes[0].set_title('visPlanner: Distance to Target vs Time', fontsize=14, fontweight='bold')
    axes[0].legend(loc='best', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    if time_sorted.max() > time_sorted.min():
        axes[0].set_xlim([time_sorted.min(), time_sorted.max()])
    
    # 子图2: 速度误差随时间变化
    axes[1].plot(time_sorted, velocity_error_sorted, linewidth=2, color='orange', label='Velocity Error')
    axes[1].axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    axes[1].axhline(y=velocity_error_sorted.mean(), color='r', linestyle='--',
                    label=f'Mean: {velocity_error_sorted.mean():.4f} m/s', linewidth=1.5)
    # 标记目标开始移动和停止移动的时刻
    for t in start_moving_times:
        axes[1].axvline(x=t, color='green', linestyle=':', linewidth=2, alpha=0.7)
    for t in stop_moving_times:
        axes[1].axvline(x=t, color='red', linestyle=':', linewidth=2, alpha=0.7)
    # 添加图例说明
    if start_moving_times:
        axes[1].axvline(x=-999, color='green', linestyle=':', linewidth=2, label='Target Start Moving')
    if stop_moving_times:
        axes[1].axvline(x=-999, color='red', linestyle=':', linewidth=2, label='Target Stop Moving')
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].set_ylabel('Velocity Error (m/s)', fontsize=12)
    axes[1].set_title('visPlanner: Velocity Error vs Time', fontsize=14, fontweight='bold')
    axes[1].legend(loc='best', fontsize=10)
    axes[1].grid(True, alpha=0.3)
    if time_sorted.max() > time_sorted.min():
        axes[1].set_xlim([time_sorted.min(), time_sorted.max()])
    
    # 子图3: 无人机机体x轴与到目标方向的夹角
    axes[2].plot(time_sorted, angle_deg_sorted, linewidth=2, color='green', label='Viewing Angle')
    axes[2].axhline(y=angle_deg_sorted.mean(), color='r', linestyle='--', 
                    label=f'Mean: {angle_deg_sorted.mean():.2f}°', linewidth=1.5)
    axes[2].fill_between(time_sorted,
                         angle_deg_sorted.mean() - angle_deg_sorted.std(),
                         angle_deg_sorted.mean() + angle_deg_sorted.std(),
                         alpha=0.2, color='green', label=f'±1σ: {angle_deg_sorted.std():.2f}°')
    # 标记目标开始移动和停止移动的时刻
    for t in start_moving_times:
        axes[2].axvline(x=t, color='green', linestyle=':', linewidth=2, alpha=0.7)
    for t in stop_moving_times:
        axes[2].axvline(x=t, color='red', linestyle=':', linewidth=2, alpha=0.7)
    # 添加图例说明
    if start_moving_times:
        axes[2].axvline(x=-999, color='green', linestyle=':', linewidth=2, label='Target Start Moving')
    if stop_moving_times:
        axes[2].axvline(x=-999, color='red', linestyle=':', linewidth=2, label='Target Stop Moving')
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].set_ylabel('Angle (degrees)', fontsize=12)
    axes[2].set_title('visPlanner: Body X-axis to Target Direction Angle vs Time', 
                     fontsize=14, fontweight='bold')
    axes[2].legend(loc='best', fontsize=10)
    axes[2].grid(True, alpha=0.3)
    if time_sorted.max() > time_sorted.min():
        axes[2].set_xlim([time_sorted.min(), time_sorted.max()])
    
    plt.tight_layout()
    
    # 保存图片
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'visplanner_tracking_performance.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"✅ 图片已保存到: {output_path}")
    
    # 打印统计信息
    print("\n" + "="*60)
    print("📊 visPlanner 跟踪性能统计")
    print("="*60)
    
    print(f"\n【距离统计】")
    print(f"  平均值:   {distance_sorted.mean():.4f} m")
    print(f"  中位数:   {distance_sorted.median():.4f} m")
    print(f"  方差:     {distance_sorted.var():.4f} m²")
    print(f"  标准差:   {distance_sorted.std():.4f} m")
    print(f"  最大值:   {distance_sorted.max():.4f} m")
    print(f"  最小值:   {distance_sorted.min():.4f} m")
    print(f"  RMS:      {np.sqrt((distance_sorted**2).mean()):.4f} m")
    
    print(f"\n【视角统计】(机体x轴与目标方向夹角)")
    print(f"  平均值:   {angle_deg_sorted.mean():.2f}°")
    print(f"  中位数:   {angle_deg_sorted.median():.2f}°")
    print(f"  方差:     {angle_deg_sorted.var():.2f}°²")
    print(f"  标准差:   {angle_deg_sorted.std():.2f}°")
    print(f"  最大值:   {angle_deg_sorted.max():.2f}°")
    print(f"  最小值:   {angle_deg_sorted.min():.2f}°")
    
    print(f"\n【速度误差统计】")
    print(f"  平均值:   {velocity_error_sorted.mean():.4f} m/s")
    print(f"  中位数:   {velocity_error_sorted.median():.4f} m/s")
    print(f"  标准差:   {velocity_error_sorted.std():.4f} m/s")
    print(f"  最大值:   {velocity_error_sorted.max():.4f} m/s")
    print(f"  最小值:   {velocity_error_sorted.min():.4f} m/s")
    print(f"  RMS:      {np.sqrt((velocity_error_sorted**2).mean()):.4f} m/s")
    
    print(f"\n【数据统计】")
    print(f"  总数据点: {len(data_sorted)}")
    print(f"  时间跨度: {time_sorted.max():.2f} 秒")
    print(f"  采样率:   ~{len(data_sorted)/max(time_sorted.max(), 1.0):.1f} Hz")
    
    print("\n" + "="*60)
    print("✅ 可视化完成！")
    print("="*60)
    
    # 生成性能报告
    report_path = os.path.join(script_dir, 'visplanner_tracking_report.txt')
    with open(report_path, 'w') as f:
        f.write("="*60 + "\n")
        f.write("visPlanner 跟踪性能报告\n")
        f.write("="*60 + "\n\n")
        f.write(f"数据文件: {data_file}\n")
        f.write(f"生成时间: {pd.Timestamp.now()}\n\n")
        
        f.write("【距离统计】\n")
        f.write(f"  平均值:   {distance_sorted.mean():.4f} m\n")
        f.write(f"  中位数:   {distance_sorted.median():.4f} m\n")
        f.write(f"  标准差:   {distance_sorted.std():.4f} m\n")
        f.write(f"  最大值:   {distance_sorted.max():.4f} m\n")
        f.write(f"  最小值:   {distance_sorted.min():.4f} m\n")
        f.write(f"  RMS:      {np.sqrt((distance_sorted**2).mean()):.4f} m\n\n")
        
        f.write("【视角统计】(机体x轴与目标方向夹角)\n")
        f.write(f"  平均值:   {angle_deg_sorted.mean():.2f}°\n")
        f.write(f"  中位数:   {angle_deg_sorted.median():.2f}°\n")
        f.write(f"  标准差:   {angle_deg_sorted.std():.2f}°\n")
        f.write(f"  最大值:   {angle_deg_sorted.max():.2f}°\n")
        f.write(f"  最小值:   {angle_deg_sorted.min():.2f}°\n\n")
        
        f.write("【速度误差统计】\n")
        f.write(f"  平均值:   {velocity_error_sorted.mean():.4f} m/s\n")
        f.write(f"  中位数:   {velocity_error_sorted.median():.4f} m/s\n")
        f.write(f"  标准差:   {velocity_error_sorted.std():.4f} m/s\n")
        f.write(f"  最大值:   {velocity_error_sorted.max():.4f} m/s\n")
        f.write(f"  最小值:   {velocity_error_sorted.min():.4f} m/s\n")
        f.write(f"  RMS:      {np.sqrt((velocity_error_sorted**2).mean()):.4f} m/s\n\n")
        
        f.write("【数据统计】\n")
        f.write(f"  总数据点: {len(data_sorted)}\n")
        f.write(f"  时间跨度: {time_sorted.max():.2f} 秒\n")
        f.write(f"  采样率:   ~{len(data_sorted)/max(time_sorted.max(), 1.0):.1f} Hz\n")
    
    print(f"📄 文本报告已保存到: {report_path}")

if __name__ == '__main__':
    main()


