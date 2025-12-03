# 测试脚本说明

## test_velocity_publisher.py

这是一个用于测试轨迹生成节点的Python脚本，它会发布模拟的速度命令。

### 功能

- 发布 `geometry_msgs/Twist` 消息到指定话题
- 支持三种速度模式：
  - **constant**: 恒定速度
  - **sine**: 正弦波速度（X轴方向）
  - **circle**: 圆周运动速度

### 使用方法

1. **赋予执行权限**：

```bash
chmod +x scripts/test_velocity_publisher.py
```

2. **运行脚本（恒定速度）**：

```bash
# 默认参数：向X轴正方向1m/s匀速运动
ros2 run pointcloud_fusion test_velocity_publisher.py
```

3. **自定义参数**：

```bash
# 指定速度
ros2 run pointcloud_fusion test_velocity_publisher.py --ros-args \
  -p linear_x:=2.0 \
  -p linear_y:=0.5 \
  -p pattern:=constant

# 正弦波速度
ros2 run pointcloud_fusion test_velocity_publisher.py --ros-args \
  -p pattern:=sine \
  -p linear_x:=1.5

# 圆周运动
ros2 run pointcloud_fusion test_velocity_publisher.py --ros-args \
  -p pattern:=circle \
  -p linear_x:=1.0
```

### 完整测试流程

1. **启动完整系统**：

```bash
ros2 launch pointcloud_fusion full_pipeline.launch.py
```

2. **在新终端启动测试速度发布器**：

```bash
ros2 run pointcloud_fusion test_velocity_publisher.py
```

3. **启动RViz可视化**：

```bash
rviz2
```

4. **在RViz中添加显示**：
   - Fixed Frame: `camera1`
   - 添加 Path: `/predicted_trajectory`
   - 添加 Marker: `/trajectory_marker`

5. **观察结果**：
   - 你应该能看到一条3米长的绿色直线轨迹
   - 橙色箭头指示运动方向
   - 轨迹会随着发布的速度实时更新

### 参数说明

- **topic**: 发布的话题名称（默认：`/cmd_vel`）
- **frequency**: 发布频率（Hz，默认：10）
- **pattern**: 速度模式（`constant`/`sine`/`circle`，默认：`constant`）
- **linear_x**: X轴线速度（m/s，默认：1.0）
- **linear_y**: Y轴线速度（m/s，默认：0.0）
- **angular_z**: Z轴角速度（rad/s，默认：0.0）

### 故障排除

如果看不到轨迹：

1. 检查速度话题是否正确：
```bash
ros2 topic echo /cmd_vel
```

2. 检查轨迹生成节点是否运行：
```bash
ros2 node list | grep trajectory
```

3. 检查轨迹话题：
```bash
ros2 topic echo /predicted_trajectory
```

4. 确保配置文件中的 `velocity_topic` 与测试脚本发布的话题一致。

