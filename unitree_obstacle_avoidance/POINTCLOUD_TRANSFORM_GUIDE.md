# 点云坐标变换配置指南

## 问题诊断

如果两个相机看到相同的物体，但融合后不重合，通常是坐标变换参数配置错误。

## 坐标系定义

### 机器人坐标系（base_link）
- **X轴**：前方（机器人行驶方向）
- **Y轴**：左方
- **Z轴**：上方

### 相机坐标系
两个相机的坐标系都遵循相同的定义：
- **X轴**：相机朝向（光轴方向）
- **Y轴**：相机左方
- **Z轴**：相机上方

## 配置步骤

### 1. 确定两个相机的相对位置

根据 `unitree_robot` 的 URDF 配置：
- **相机1（小车相机）**：位于小车前方
- **相机2（圆柱体相机）**：位于小车左后方45°、0.5m、上方1.6m

### 2. 计算变换参数

变换参数定义了：**从相机2坐标系到相机1坐标系的变换**

#### 平移部分（translation）
如果相机2的原点在相机1坐标系中的位置是 `(tx, ty, tz)`，则：
- `transform_x = tx`
- `transform_y = ty`
- `transform_z = tz`

对于左后方45°、0.5m、上方1.6m：
```
tx = -0.5 * cos(45°) = -0.354  （后方）
ty =  0.5 * sin(45°) =  0.354  （左方）
tz =  1.6                      （上方）
```

#### 旋转部分（rotation）
如果两个相机朝向相同（都朝前），则：
```
transform_roll  = 0.0
transform_pitch = 0.0
transform_yaw   = 0.0
```

**重要**：旋转角度是相机2相对于相机1的旋转。
- 如果相机2需要绕Z轴旋转θ角才能与相机1对齐，则 `transform_yaw = -θ`

### 3. 验证配置

编译并运行后，查看日志输出：

```bash
colcon build --packages-select unitree_obstacle_avoidance
source install/setup.bash
ros2 launch unitree_obstacle_avoidance pointcloud_fusion.launch.py
```

日志会显示：
1. **变换矩阵**：4x4变换矩阵
2. **点云中心**：变换前后的点云中心位置

## 调试技巧

### 1. 使用RViz可视化
```bash
rviz2
```
添加两个PointCloud2显示：
- `/camera_dog/points`（红色）
- `/camera_person/points`（绿色）
- `/pointcloud_fused`（白色）

观察融合后的点云，相同物体应该重合。

### 2. 检查点云中心偏移
从日志中查看：
```
点云2中心 - 变换前:(x2, y2, z2) -> 变换后:(x2', y2', z2')
点云1中心:(x1, y1, z1)
```

**理想情况**：如果两个相机看到同一个物体，变换后的点云2中心应该与点云1中心接近。

### 3. 常见错误

#### 错误1：符号错误
```yaml
# 错误：后方应该是负值
transform_x: 0.354   # ❌

# 正确
transform_x: -0.354  # ✅
```

#### 错误2：旋转角度错误
如果融合后的点云旋转了，检查：
```yaml
# 如果相机2相对于相机1逆时针旋转了45°
transform_yaw: -0.785  # -45° in radians

# 如果朝向一致
transform_yaw: 0.0
```

#### 错误3：使用了错误的坐标系
确保：
- `topic1` 和 `topic2` 的坐标系与配置匹配
- `target_frame` 设置为合适的参考坐标系

## 手动校准方法

如果自动计算的参数不准确，可以手动调整：

1. **调整平移**：观察RViz中的偏移，逐步调整 `transform_x/y/z`
   ```yaml
   # 每次调整0.1m
   transform_x: -0.354  # 如果物体向前偏移，减小这个值
   transform_y: 0.354   # 如果物体向右偏移，减小这个值
   transform_z: 1.6     # 如果物体向下偏移，减小这个值
   ```

2. **调整旋转**：如果物体旋转了，调整yaw角
   ```yaml
   # 每次调整5°(0.087弧度)
   transform_yaw: 0.0   # 如果物体逆时针旋转，增加这个值
   ```

3. **迭代优化**：
   - 修改参数
   - 重启节点
   - 观察RViz
   - 重复直到重合

## 验证公式

对于任意一个点 `P2 = (x2, y2, z2)` 在相机2坐标系中：

变换后在相机1坐标系中的位置 `P1 = (x1, y1, z1)` 应该满足：
```
P1 = R * P2 + T
```
其中：
- `R` 是旋转矩阵（从roll/pitch/yaw计算）
- `T` 是平移向量 `[transform_x, transform_y, transform_z]`

## 参考

- PCL变换文档：https://pcl.readthedocs.io/en/latest/using_pcl_pcl_config.html
- TF2变换教程：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html

