# MPC控制器使用指南

## 概述

本项目实现了基于**CasADi**的模型预测控制器（MPC），用于拖车（trailer）系统的轨迹跟踪和避障控制。

## 系统模型

### 拖车模型

系统采用简化的拖车运动学模型：

```
状态变量: [x_front, y_front, theta_front, v, omega]
- x_front, y_front: 前车位置
- theta_front: 前车朝向角
- v: 线速度
- omega: 角速度

控制输入: [a, alpha]
- a: 线加速度
- alpha: 角加速度

后车位置:
x_rear = x_front - L * cos(theta_front)
y_rear = y_front - L * sin(theta_front)
其中 L 为挂接长度
```

### 本体表示

- **前车**: 圆形，半径 `front_radius`（默认0.5米）
- **后车**: 圆形，半径 `rear_radius`（默认0.5米）
- **挂接长度**: `hitch_length`（默认2.0米）

## 功能特性

### 1. 轨迹跟踪

- 订阅参考轨迹（`nav_msgs/Path`）
- 通过MPC优化跟踪误差
- 考虑位置和方向误差

### 2. 障碍物避障

- 订阅椭圆障碍物（`pointcloud_fusion/EllipseObstacleArray`）
- 使用**椭圆归一化距离**计算障碍物与车辆的距离
- 在代价函数中添加避障惩罚

### 3. 约束处理

- 速度和加速度限制
- 平滑的控制输入

### 4. 实时控制

- 10Hz控制频率
- 发布速度命令（`geometry_msgs/Twist`）

## 安装依赖

### CasADi库

MPC控制器需要CasADi优化库。

#### Ubuntu/Debian安装

```bash
# 方法1: 从源码编译（推荐）
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install

# 方法2: 使用pip安装Python版本（可选）
pip3 install casadi

# 方法3: 使用预编译包
# 从 https://github.com/casadi/casadi/releases 下载对应版本
```

#### 验证安装

```bash
# 测试C++库
pkg-config --libs casadi

# 或者
find /usr -name "*casadi*" 2>/dev/null
```

## 配置参数

编辑 `config/mpc_controller_params.yaml`:

### 拖车模型参数

```yaml
front_radius: 0.5          # 前车半径（米）
rear_radius: 0.5           # 后车半径（米）
hitch_length: 2.0          # 挂接长度（米）
```

### MPC参数

```yaml
prediction_horizon: 20     # 预测时域（步数）
dt: 0.1                    # 采样时间（秒）
```

- **prediction_horizon**: 预测未来多少步，值越大计算量越大但效果越好
- **dt**: 离散化时间步长，影响控制精度

### 约束参数

```yaml
max_velocity: 2.0          # 最大线速度（米/秒）
max_angular_vel: 1.0       # 最大角速度（弧度/秒）
max_acceleration: 1.0      # 最大加速度（米/秒²）
max_angular_acc: 0.5       # 最大角加速度（弧度/秒²）
```

### 代价函数权重

```yaml
weight_position: 10.0      # 位置跟踪权重
weight_orientation: 5.0    # 方向跟踪权重
weight_velocity: 0.1       # 速度平滑权重
weight_angular_vel: 0.1    # 角速度平滑权重
weight_obstacle: 100.0     # 障碍物避障权重
```

**调优建议**:
- 增大 `weight_position` 提高位置跟踪精度
- 增大 `weight_obstacle` 增强避障能力
- 增大 `weight_velocity/angular_vel` 获得更平滑的控制

### 安全参数

```yaml
safety_distance: 0.5       # 安全距离（米）
```

## 使用方法

### 方法1: 启动完整流程（推荐）

包含点云融合、障碍物检测、轨迹生成和MPC控制：

```bash
ros2 launch pointcloud_fusion full_pipeline.launch.py
```

### 方法2: 单独启动MPC控制器

```bash
ros2 launch pointcloud_fusion mpc_controller.launch.py
```

### 方法3: 直接运行节点

```bash
ros2 run pointcloud_fusion mpc_controller_node --ros-args \
  --params-file config/mpc_controller_params.yaml
```

## 话题说明

### 订阅的话题

- **`/predicted_trajectory`** (nav_msgs/Path)
  - 参考轨迹
  - 通常由轨迹生成节点提供

- **`/obstacles`** (pointcloud_fusion/EllipseObstacleArray)
  - 椭圆障碍物数组
  - 由障碍物检测节点提供

### 发布的话题

- **`/cmd_vel_mpc`** (geometry_msgs/Twist)
  - MPC计算的速度命令
  - 包含线速度和角速度

- **`/mpc_markers`** (visualization_msgs/MarkerArray)
  - 可视化标记
  - 显示前车和后车的位置

## 椭圆-圆距离计算

MPC使用**椭圆归一化距离**来评估障碍物威胁：

### 计算步骤

1. 将圆心转换到椭圆局部坐标系
2. 计算归一化距离: `d_norm = sqrt((x/a)² + (y/b)²)`
3. 实际距离: `d = (d_norm - 1) * max(a,b) - r`

其中:
- `(x, y)`: 圆心在椭圆坐标系中的位置
- `a, b`: 椭圆长短半轴
- `r`: 圆半径

### 避障策略

MPC在代价函数中添加避障项：

```
cost_obstacle = weight_obstacle / (distance + epsilon)
```

- 距离越小，惩罚越大
- 对前车和后车分别计算
- 所有障碍物的惩罚累加

## MPC优化问题

### 目标函数

```
minimize: Σ[
  w_pos * ||position - reference||²        # 位置跟踪
  + w_ori * ||orientation - reference||²   # 方向跟踪
  + w_vel * ||acceleration||²              # 控制平滑
  + w_ang * ||angular_acc||²               # 角加速度平滑
  + w_obs * Σ(1 / (distance + ε))          # 避障惩罚
]
```

### 约束条件

1. **系统动力学**: `x[k+1] = x[k] + dt * f(x[k], u[k])`
2. **初始状态**: `x[0] = x_current`
3. **控制约束**: 
   - `-max_acc ≤ a ≤ max_acc`
   - `-max_ang_acc ≤ alpha ≤ max_ang_acc`
4. **状态约束**:
   - `-max_vel ≤ v ≤ max_vel`
   - `-max_omega ≤ omega ≤ max_omega`

## 可视化

### 在RViz中查看

1. 添加 **MarkerArray** 显示
2. 订阅 `/mpc_markers` 话题
3. 显示内容：
   - 绿色圆柱体: 前车
   - 蓝色圆柱体: 后车

### 监控话题

```bash
# 查看速度命令
ros2 topic echo /cmd_vel_mpc

# 查看MPC频率
ros2 topic hz /cmd_vel_mpc

# 查看参考轨迹
ros2 topic echo /predicted_trajectory

# 查看障碍物
ros2 topic echo /obstacles
```

## 性能优化

### 1. 调整预测时域

- **减小N**: 更快但可能不够前瞻
- **增大N**: 更好的性能但计算量大

```yaml
prediction_horizon: 15  # 降低到15可能更实时
```

### 2. 调整采样时间

```yaml
dt: 0.15  # 增大dt减少计算量
```

### 3. 求解器选项

在代码中可以调整IPOPT求解器参数：

```cpp
opts["ipopt.max_iter"] = 50;    # 减少迭代次数
opts["ipopt.tol"] = 1e-3;       # 放宽收敛条件
```

### 4. 简化障碍物

只考虑最近的N个障碍物：

```cpp
// 只处理距离最近的5个障碍物
std::sort(obstacles_.begin(), obstacles_.end(), 
          [](const auto& a, const auto& b) { 
            return a.distance < b.distance; 
          });
if (obstacles_.size() > 5) {
  obstacles_.resize(5);
}
```

## 故障排除

### 1. CasADi未找到

**错误**: `Could not find casadi`

**解决**:
```bash
# 设置环境变量
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# 或者在 ~/.bashrc 中添加
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 2. MPC求解失败

**症状**: 日志显示 "MPC求解失败"

**可能原因**:
- 初始状态不可行
- 约束过于严格
- 障碍物太近无法避开

**解决**:
- 放宽速度/加速度限制
- 调整权重参数
- 检查障碍物数据是否合理

### 3. 控制不稳定

**症状**: 车辆运动不平滑或震荡

**解决**:
- 增大 `weight_velocity` 和 `weight_angular_vel`
- 减小 `dt` 提高控制精度
- 增大 `prediction_horizon`

### 4. 避障失效

**症状**: 车辆没有避开障碍物

**解决**:
- 增大 `weight_obstacle`
- 检查障碍物话题是否正常
- 确认椭圆-圆距离计算正确

## 扩展功能

### 1. 添加速度跟踪

在目标函数中添加速度跟踪项：

```cpp
MX vel_error = X(3, k) - ref_velocity;
cost += weight_vel_tracking * vel_error * vel_error;
```

### 2. 动态障碍物

考虑障碍物的速度信息（需要扩展EllipseObstacle消息）。

### 3. 多目标优化

使用多目标优化平衡跟踪和避障。

### 4. 自适应权重

根据场景动态调整权重参数。

## 参考资料

- **CasADi文档**: https://web.casadi.org/
- **MPC理论**: Rawlings & Mayne, "Model Predictive Control: Theory and Design"
- **拖车模型**: https://en.wikipedia.org/wiki/Trailer_(vehicle)
- **IPOPT求解器**: https://coin-or.github.io/Ipopt/

## 性能指标

典型性能（在Intel i7处理器上）：

- **控制频率**: 10 Hz
- **求解时间**: 20-50 ms
- **预测时域**: 20步（2秒）
- **内存使用**: ~50 MB

## 许可证

Apache-2.0

