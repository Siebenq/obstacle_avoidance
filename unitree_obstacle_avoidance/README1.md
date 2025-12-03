

## 依赖项

```bash
sudo apt-get install ros-${ROS_DISTRO}-pcl-ros \
                     ros-${ROS_DISTRO}-pcl-conversions \
                     ros-${ROS_DISTRO}-sensor-msgs \
                     ros-${ROS_DISTRO}-geometry-msgs \
                     ros-${ROS_DISTRO}-visualization-msgs \
                     ros-${ROS_DISTRO}-nav-msgs \
                     ros-${ROS_DISTRO}-tf2 \
                     ros-${ROS_DISTRO}-tf2-ros \
                     ros-${ROS_DISTRO}-tf2-eigen
```

### CasADi库（MPC控制器需要）

```bash
# 方法1: 从源码编译（推荐）
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install

# 方法2: 使用pip安装Python版本
pip3 install casadi

# 验证安装
pkg-config --libs casadi
```

## 编译

1. 将此包放入ROS2工作空间的src目录：

```bash
cd ~/ros2_ws/src
# 将此项目复制或克隆到这里
```

2. 编译包：

```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_fusion
source install/setup.bash
```

## 配置

编辑 `config/fusion_params.yaml` 文件来配置节点参数：

```yaml
pointcloud_fusion_node:
  ros__parameters:
    # 输入话题名称
    topic1: "/camera1/pointcloud"
    topic2: "/camera2/pointcloud"
    
    # 输出话题名称
    output_topic: "/pointcloud_fused"
    
    # 目标坐标系
    target_frame: "base_link"
    
    # 是否使用坐标变换
    use_transform: true
    
    # 从topic2坐标系到topic1坐标系的坐标变换
    # 平移（单位：米）
    transform_x: 0.0
    transform_y: 0.0
    transform_z: 0.0
    
    # 旋转（单位：弧度）- Roll, Pitch, Yaw (绕X, Y, Z轴)
    transform_roll: 0.0
    transform_pitch: 0.0
    transform_yaw: 0.0
```

### 坐标变换说明

- **transform_x/y/z**: 从点云2坐标系原点到点云1坐标系的平移向量（米）
- **transform_roll/pitch/yaw**: 从点云2坐标系到点云1坐标系的旋转（弧度）
  - Roll: 绕X轴旋转
  - Pitch: 绕Y轴旋转
  - Yaw: 绕Z轴旋转
- **use_transform**: 如果设为false，则不进行坐标变换，直接融合点云

## 使用方法

### 方法1：启动完整流程（推荐）

同时启动所有节点（点云融合 + 障碍物检测 + 轨迹生成 + MPC控制）：

```bash
ros2 launch pointcloud_fusion full_pipeline.launch.py
```

使用自定义参数文件：

```bash
ros2 launch pointcloud_fusion full_pipeline.launch.py \
  fusion_params_file:=/path/to/fusion_params.yaml \
  obstacle_params_file:=/path/to/obstacle_params.yaml \
  trajectory_params_file:=/path/to/trajectory_params.yaml \
  mpc_params_file:=/path/to/mpc_params.yaml
```

### 方法2：分别启动节点

启动点云融合节点：

```bash
ros2 launch pointcloud_fusion pointcloud_fusion.launch.py
```

启动障碍物检测节点：

```bash
ros2 launch pointcloud_fusion obstacle_detection.launch.py
```

启动轨迹生成节点：

```bash
ros2 launch pointcloud_fusion trajectory_generator.launch.py
```

启动MPC控制器节点：

```bash
ros2 launch pointcloud_fusion mpc_controller.launch.py
```

### 方法3：直接运行节点

运行点云融合节点：

```bash
ros2 run pointcloud_fusion pointcloud_fusion_node --ros-args --params-file config/fusion_params.yaml
```

运行障碍物检测节点：

```bash
ros2 run pointcloud_fusion obstacle_detection_node --ros-args --params-file config/obstacle_detection_params.yaml
```

运行轨迹生成节点：

```bash
ros2 run pointcloud_fusion trajectory_generator_node --ros-args --params-file config/trajectory_generator_params.yaml
```

运行MPC控制器节点：

```bash
ros2 run pointcloud_fusion mpc_controller_node --ros-args --params-file config/mpc_controller_params.yaml
```

## 话题说明

### 点云融合节点

**订阅的话题：**
- **topic1** (sensor_msgs/PointCloud2): 第一个点云话题
- **topic2** (sensor_msgs/PointCloud2): 第二个点云话题

**发布的话题：**
- **output_topic** (sensor_msgs/PointCloud2): 融合后的点云话题

### 障碍物检测节点

**订阅的话题：**
- **input_topic** (sensor_msgs/PointCloud2): 输入点云话题（通常是融合后的点云）

**发布的话题：**
- **output_obstacle_topic** (pointcloud_fusion/EllipseObstacleArray): 椭圆障碍物数组
- **output_marker_topic** (visualization_msgs/MarkerArray): 障碍物可视化标记

### 轨迹生成节点

**订阅的话题：**
- **velocity_topic** (geometry_msgs/Twist 或 TwistStamped): 速度信息话题

**发布的话题：**
- **output_path_topic** (nav_msgs/Path): 生成的轨迹路径
- **output_marker_topic** (visualization_msgs/Marker): 轨迹可视化标记

### MPC控制器节点

**订阅的话题：**
- **trajectory_topic** (nav_msgs/Path): 参考轨迹
- **obstacle_topic** (pointcloud_fusion/EllipseObstacleArray): 椭圆障碍物数组

**发布的话题：**
- **cmd_vel_topic** (geometry_msgs/Twist): MPC计算的速度命令
- **marker_topic** (visualization_msgs/MarkerArray): 车辆可视化标记

## 测试

### 使用RViz可视化

1. 启动RViz：

```bash
rviz2
```

2. 在RViz中配置显示：
   - 设置Fixed Frame为 `base_link`（或你配置的target_frame）
   - **添加PointCloud2显示**：
     - 点击Add按钮
     - 选择PointCloud2
     - 设置Topic为 `/pointcloud_fused`（融合后的点云）
   - **添加MarkerArray显示（障碍物）**：
     - 点击Add按钮
     - 选择MarkerArray
     - 设置Topic为 `/obstacles_markers`（障碍物标记）
   - **添加Path显示（轨迹）**：
     - 点击Add按钮
     - 选择Path
     - 设置Topic为 `/predicted_trajectory`（预测轨迹）
   - **添加Marker显示（轨迹标记）**：
     - 点击Add按钮
     - 选择Marker
     - 设置Topic为 `/trajectory_marker`（轨迹可视化）
   - **添加MarkerArray显示（MPC车辆）**：
     - 点击Add按钮
     - 选择MarkerArray
     - 设置Topic为 `/mpc_markers`（MPC车辆可视化）

3. 你应该能看到：
   - 融合后的完整点云
   - 检测到的障碍物用椭圆柱体标记
   - 每个障碍物上方显示ID、距离和尺寸信息
   - 绿色直线轨迹（3米长）
   - 橙色箭头指示运动方向
   - 绿色和蓝色圆柱体表示前车和后车

### 查看话题

```bash
# 查看所有话题
ros2 topic list

# 查看融合后的点云话题信息
ros2 topic info /pointcloud_fused

# 查看障碍物标记话题信息
ros2 topic info /obstacles_markers

# 查看点云数据
ros2 topic echo /pointcloud_fused

# 查看障碍物标记数据
ros2 topic echo /obstacles_markers

# 查看轨迹路径数据
ros2 topic echo /predicted_trajectory

# 查看MPC速度命令
ros2 topic echo /cmd_vel_mpc
```

## 性能优化

- 节点使用定时器以100ms的频率进行融合（10Hz）
- 可以通过修改源代码中的定时器周期来调整融合频率
- 对于大点云，建议根据系统性能调整频率

## 故障排除

### 1. 节点启动但没有输出

检查是否正确接收到两个点云话题：

```bash
ros2 topic echo /camera1/pointcloud
ros2 topic echo /camera2/pointcloud
```

### 2. 编译错误

确保所有依赖项已正确安装：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 点云位置不正确

检查并调整config文件中的坐标变换参数。可以使用以下方法确定正确的变换：

- 使用tf2工具查看坐标系关系
- 在RViz中可视化原始点云来确定相对位置
- 逐步调整transform参数

## 示例场景

### 场景1: 两个前置摄像头

假设有两个前置摄像头，相距1米，朝向相同：

```yaml
transform_x: 1.0  # 摄像头2在摄像头1右侧1米
transform_y: 0.0
transform_z: 0.0
transform_roll: 0.0
transform_pitch: 0.0
transform_yaw: 0.0  # 朝向相同
```

### 场景2: 前后摄像头

假设后置摄像头相对于前置摄像头旋转180度：

```yaml
transform_x: -2.0  # 后置摄像头在前置摄像头后方2米
transform_y: 0.0
transform_z: 0.0
transform_roll: 0.0
transform_pitch: 0.0
transform_yaw: 3.14159  # 180度 = π弧度
```

## 障碍物检测参数说明

编辑 `config/obstacle_detection_params.yaml` 来调整障碍物检测行为：

### 关键参数

- **voxel_leaf_size**: 降采样体素大小（米），值越大处理越快但精度越低
- **x/y/z_filter_min/max**: ROI感兴趣区域范围（米）
- **remove_ground**: 是否移除地面点
- **ground_threshold**: 地面平面距离阈值（米）
- **cluster_tolerance**: 聚类容差（米），控制障碍物分离程度
- **min/max_cluster_size**: 有效簇的点数范围
- **min/max_obstacle_height/width**: 有效障碍物的尺寸范围
- **ellipse_scale_factor**: 椭圆放大系数（默认1.2），用于调整椭圆大小以更好地包络障碍物

### 椭圆障碍物说明

障碍物检测节点使用 **主成分分析（PCA）** 将每个障碍物投影到XY平面并简化为椭圆：

- **长半轴（semi_major_axis）**: 障碍物在主方向上的尺寸
- **短半轴（semi_minor_axis）**: 障碍物在次方向上的尺寸
- **旋转角度（rotation_angle）**: 椭圆长轴相对于X轴的角度（弧度）
- **高度信息**: 保留Z轴的最小、最大和高度值

这种简化方式：
- ✅ 大幅减少数据量（每个障碍物只需几个参数）
- ✅ 便于后续避障算法使用
- ✅ 提供障碍物的主要几何特征
- ✅ 支持旋转的障碍物表示

### 自定义消息类型

**EllipseObstacle.msg**:
```
int32 id                      # 障碍物ID
geometry_msgs/Point center    # 中心位置
float64 semi_major_axis       # 长半轴（米）
float64 semi_minor_axis       # 短半轴（米）
float64 rotation_angle        # 旋转角度（弧度）
float64 height                # 高度（米）
float64 z_min, z_max          # Z轴范围
int32 point_count             # 点云数量
float64 distance              # 距离原点
```

**EllipseObstacleArray.msg**:
```
std_msgs/Header header
EllipseObstacle[] obstacles
```

### 参数调优建议

1. **提高处理速度**：增大 `voxel_leaf_size`，减小ROI范围
2. **提高检测精度**：减小 `voxel_leaf_size`，减小 `cluster_tolerance`
3. **过滤小物体**：增大 `min_cluster_size` 和 `min_obstacle_height`
4. **检测远距离障碍物**：增大 `x_filter_max`
5. **调整椭圆大小**：修改 `ellipse_scale_factor`（增大使椭圆更保守地包络障碍物）

## 轨迹生成参数说明

编辑 `config/trajectory_generator_params.yaml` 来调整轨迹生成行为：

### 关键参数

- **velocity_topic**: 输入速度话题名称
- **source_frame**: 速度所在的源坐标系（默认camera2）
- **target_frame**: 目标坐标系（默认camera1）
- **transform_x/y/z**: 坐标系变换平移参数（米）
- **transform_roll/pitch/yaw**: 坐标系变换旋转参数（弧度）
- **trajectory_length**: 轨迹长度（米）
- **trajectory_points**: 轨迹点数
- **default_velocity**: 当速度为零时的默认速度（米/秒）

### 重要说明

⚠️ **坐标变换参数必须与点云融合配置一致**：`trajectory_generator_params.yaml` 中的 transform 参数应该与 `fusion_params.yaml` 中的参数保持一致，以确保速度转换到正确的坐标系。

### 使用场景

1. **路径规划**：生成的轨迹可用于预测未来位置
2. **避障系统**：与障碍物检测结合，判断路径是否安全
3. **可视化**：在RViz中实时查看运动方向和预期路径

## MPC控制器参数说明

编辑 `config/mpc_controller_params.yaml` 来调整MPC行为：

### 拖车模型参数

- **front_radius**: 前车半径（米）
- **rear_radius**: 后车半径（米）
- **hitch_length**: 挂接长度（米）

### MPC参数

- **prediction_horizon**: 预测时域（步数），值越大计算量越大
- **dt**: 采样时间（秒），影响控制精度

### 代价函数权重

- **weight_position**: 位置跟踪权重，增大提高跟踪精度
- **weight_orientation**: 方向跟踪权重
- **weight_obstacle**: 障碍物避障权重，增大增强避障能力
- **weight_velocity/angular_vel**: 控制平滑权重

### 详细说明

查看 `MPC_CONTROLLER_GUIDE.md` 获取完整的MPC控制器使用指南。

## 开发者信息

### 代码结构

```
pointcloud_fusion/
├── CMakeLists.txt                        # 构建配置
├── package.xml                           # 包元数据
├── README.md                             # 本文件
├── QUICKSTART.md                         # 快速开始指南
├── MPC_CONTROLLER_GUIDE.md               # MPC控制器详细指南
├── ELLIPSE_OBSTACLE_USAGE.md             # 椭圆障碍物使用指南
├── msg/                                  # 自定义消息定义
│   ├── EllipseObstacle.msg               # 椭圆障碍物消息
│   └── EllipseObstacleArray.msg          # 椭圆障碍物数组消息
├── config/
│   ├── fusion_params.yaml                # 点云融合参数
│   ├── obstacle_detection_params.yaml    # 障碍物检测参数
│   ├── trajectory_generator_params.yaml  # 轨迹生成参数
│   └── mpc_controller_params.yaml        # MPC控制器参数
├── launch/
│   ├── pointcloud_fusion.launch.py       # 点云融合启动文件
│   ├── obstacle_detection.launch.py      # 障碍物检测启动文件
│   ├── trajectory_generator.launch.py    # 轨迹生成启动文件
│   ├── mpc_controller.launch.py          # MPC控制器启动文件
│   └── full_pipeline.launch.py           # 完整流程启动文件
├── scripts/                              # 测试脚本
│   ├── test_velocity_publisher.py        # 速度发布测试脚本
│   └── README_TEST.md                    # 测试说明
└── src/
    ├── pointcloud_fusion_node.cpp        # 点云融合节点
    ├── obstacle_detection_node.cpp       # 障碍物检测节点（椭圆简化）
    ├── trajectory_generator_node.cpp     # 轨迹生成节点
    └── mpc_controller_node.cpp           # MPC控制器节点（拖车模型）
```

### 处理流程

```
点云1 ─┐
       ├─→ 点云融合 ─→ 融合点云 ─→ 障碍物检测 ─→ 椭圆障碍物 ─┐
点云2 ─┘                              ↓                    │
                              ┌──────┴──────┐             │
                              │ 1. 降采样   │             ↓
速度   ─→ 轨迹生成 ─→ 轨迹路径 │ 2. ROI滤波  │        MPC控制器
(camera2)     ↓              │ 3. 地面移除 │        (拖车模型)
         坐标转换             │ 4. 聚类分析 │             ↓
        (→camera1)           │ 5. PCA椭圆  │        速度命令
              ↓              └─────────────┘       /cmd_vel_mpc
         轨迹可视化                 ↓
                                椭圆可视化
```

### 扩展功能建议

**点云融合：**
- 支持动态TF变换
- 支持三个或更多点云融合
- 支持不同点云类型（PointXYZRGB、PointXYZI等）
- 添加时间戳同步

**障碍物检测：**
- ✅ 已实现椭圆障碍物简化和自定义消息
- 添加障碍物跟踪功能（基于椭圆匹配）
- 支持障碍物分类（行人、车辆等）
- 集成深度学习模型
- 与轨迹生成集成实现碰撞预测
- 生成代价地图用于路径规划
- 优化椭圆拟合算法

**轨迹生成：**
- 支持动态速度更新
- 添加曲线轨迹生成（非直线）
- 支持多条轨迹预测（不同速度假设）
- 与障碍物检测集成实现碰撞检测
- 添加轨迹平滑算法
- 支持不同运动模型（匀加速、转弯等）

**MPC控制器：**
- ✅ 已实现基于CasADi的MPC
- ✅ 已实现拖车运动学模型
- ✅ 已实现椭圆归一化距离避障
- 添加自适应权重调整
- 支持更复杂的车辆模型（阿克曼模型）
- 集成状态估计器
- 添加多目标优化
- 实现分布式MPC

## 许可证

Apache-2.0

## 贡献

欢迎提交问题和拉取请求！

