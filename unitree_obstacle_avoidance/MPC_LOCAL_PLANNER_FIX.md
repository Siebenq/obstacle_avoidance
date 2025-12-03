# MPC局部规划器避障修复

## 🔍 问题描述

**现象**：
- MPC可以接收到障碍物信息和目标位置
- 但遇到障碍物会**直接撞上去**，不绕行
- 机器人坐标系：本体**始终位于原点(0, 0)**（局部规划器模式）

## 🎯 问题根源

### 致命错误：障碍物代价与预测位置无关

```cpp
// ❌ 错误的代码
MX computeObstacleCost() {  // 没有位置参数！
    for (const auto& obs : obstacles_) {
        MX dist = ellipseCircleDistance(
            robot_radius_,  // 只有半径
            obs.center.x, obs.center.y, ...);
    }
}

MX ellipseCircleDistance(double radius, ...) {
    MX dx = -ex;  // 假设机器人永远在(0,0)！
    MX dy = -ey;
}
```

### 为什么会撞上去？

```
场景：机器人在(0,0)，障碍物在(2,0)，目标在(5,0)

MPC优化过程（错误）：
  方案A（直行）：
    步骤0: (0, 0) → 障碍物代价 = f(0,0) = 某个值
    步骤1: (0.5, 0) → 障碍物代价 = f(0,0) = 相同值！❌
    步骤2: (1.0, 0) → 障碍物代价 = f(0,0) = 相同值！❌
    步骤3: (1.5, 0) → 障碍物代价 = f(0,0) = 相同值！❌
    步骤4: (2.0, 0) → 障碍物代价 = f(0,0) = 相同值！❌ (撞了！)
    总代价 = 跟踪误差(小) + 障碍物代价(固定)
  
  方案B（绕行）：
    步骤0: (0, 0)
    步骤1: (0.4, 0.2) → 障碍物代价 = f(0,0) = 相同值！❌
    步骤2: (0.8, 0.5)
    步骤3: (1.2, 0.8)
    步骤4: (1.6, 1.0)
    总代价 = 跟踪误差(大) + 障碍物代价(固定)

MPC选择：方案A（跟踪误差更小）
结果：撞上障碍物！❌
```

**核心问题**：
- 障碍物代价始终基于(0,0)计算
- 不管预测轨迹是直行还是绕行，障碍物代价都一样
- MPC只看到跟踪误差，选择最短路径（直线）
- 直接撞上去！

## ✅ 正确的解决方案

### 核心原理：障碍物代价必须依赖预测位置

```cpp
// ✅ 正确的代码
MX computeObstacleCost(const MX& robot_x, const MX& robot_y) {  // 接收位置参数
    for (const auto& obs : obstacles_) {
        MX dist = ellipseCircleDistance(
            robot_x, robot_y,  // 使用预测位置
            robot_radius_,
            obs.center.x, obs.center.y, ...);
    }
}

MX ellipseCircleDistance(const MX& robot_x, const MX& robot_y, ...) {
    MX dx = robot_x - ex;  // 预测位置到障碍物的距离
    MX dy = robot_y - ey;
}
```

### 修复后的MPC优化

```
场景：机器人在(0,0)，障碍物在(2,0)，目标在(5,0)

MPC优化过程（正确）：
  方案A（直行）：
    步骤0: (0, 0) → 障碍物代价 = f(0, 0) = 小
    步骤1: (0.5, 0) → 障碍物代价 = f(0.5, 0) = 小
    步骤2: (1.0, 0) → 障碍物代价 = f(1.0, 0) = 中等
    步骤3: (1.5, 0) → 障碍物代价 = f(1.5, 0) = 大！✓
    步骤4: (2.0, 0) → 障碍物代价 = f(2.0, 0) = 极大！✓
    总代价 = 跟踪误差(小) + 障碍物代价(极大)
  
  方案B（绕行）：
    步骤0: (0, 0) → 障碍物代价 = f(0, 0) = 小
    步骤1: (0.4, 0.2) → 障碍物代价 = f(0.4, 0.2) = 小
    步骤2: (0.8, 0.5) → 障碍物代价 = f(0.8, 0.5) = 小
    步骤3: (1.2, 0.8) → 障碍物代价 = f(1.2, 0.8) = 小
    步骤4: (1.6, 1.0) → 障碍物代价 = f(1.6, 1.0) = 小
    总代价 = 跟踪误差(大) + 障碍物代价(小)

MPC选择：方案B（总代价更小）
结果：成功绕行！✅
```

## 🔧 详细修改

### 修改1：ellipseCircleDistance函数

```cpp
// 修复前（错误）
MX ellipseCircleDistance(double radius, double ex, double ey, ...) {
    MX dx = -ex;  // 假设机器人在(0,0)
    MX dy = -ey;
    // ...
}

// 修复后（正确）
MX ellipseCircleDistance(const MX& robot_x, const MX& robot_y,
                         double radius, double ex, double ey, ...) {
    MX dx = robot_x - ex;  // 使用预测位置
    MX dy = robot_y - ey;
    // ...
}
```

**关键**：
- `robot_x`, `robot_y` 是MX类型（符号变量）
- 在MPC第k步，可能是 (0.5, 0.1), (1.0, 0.2) 等
- 不是固定的(0, 0)

### 修改2：computeObstacleCost函数

```cpp
// 修复前（错误）
MX computeObstacleCost() {  // 无参数
    MX dist = ellipseCircleDistance(
        robot_radius_,  // 缺少机器人位置
        obs.center.x, obs.center.y, ...);
}

// 修复后（正确）
MX computeObstacleCost(const MX& robot_x, const MX& robot_y) {
    MX dist = ellipseCircleDistance(
        robot_x, robot_y,  // 传入预测位置
        robot_radius_,
        obs.center.x, obs.center.y, ...);
}
```

### 修改3：MPC优化循环中的调用

```cpp
// 修复前（错误）
for (int k = 0; k < N_; ++k) {
    // ...
    if (has_obstacles_) {
        cost += weight_obs_ * computeObstacleCost();  // 无参数
    }
}

// 修复后（正确）
for (int k = 0; k < N_; ++k) {
    // ...
    if (has_obstacles_) {
        cost += weight_obs_ * computeObstacleCost(X(0, k), X(1, k));  // 传入第k步位置
    }
}
```

**解释**：
- `X(0, k)` = MPC第k步预测的x坐标
- `X(1, k)` = MPC第k步预测的y坐标
- 即使k=0时在(0,0)，k>0时会移动

## 📊 技术原理

### 局部规划器vs全局规划器

| 类型 | 坐标系 | 机器人位置 | MPC初始状态 |
|-----|--------|----------|-----------|
| 全局规划器 | 固定世界坐标系 | 实时更新(x, y) | current_state_ |
| 局部规划器 | 跟随机器人移动 | 始终在(0, 0) | [0, 0, θ] |

**您的场景是局部规划器**。

### 局部规划器的MPC预测

```
初始状态: X(0) = [0, 0, θ]  (当前在原点)

预测轨迹:
  X(1) = [0.1, 0.0, θ1]  ← k=1步后的位置
  X(2) = [0.2, 0.05, θ2] ← k=2步后的位置
  X(3) = [0.3, 0.10, θ3]
  ...
  X(N) = [vt*N, ..., θN]
```

**关键**：虽然起点是(0,0)，但预测的未来位置会移动！

### 障碍物代价计算

```
对于MPC第k步：
  预测位置: (X(0,k), X(1,k))
  障碍物位置: (obs.x, obs.y)
  
  相对位置:
    dx = X(0,k) - obs.x
    dy = X(1,k) - obs.y
  
  距离:
    dist = sqrt(dx²/A² + dy²/B²)  (椭圆归一化)
  
  代价:
    cost = exp(-λ * (dist - 1))
```

**错误做法**：`dx = -obs.x` (假设机器人永远在0)
**正确做法**：`dx = X(0,k) - obs.x` (使用预测位置)

## 🎯 为什么这样修复有效？

### 修复前的问题

```
MPC看到的代价函数：
  cost = tracking_error(X) + constant
                              ^^^^^^^^
                              障碍物代价与X无关！

优化器思考：
  "障碍物代价是常数，我只需要最小化tracking_error"
  "最小tracking_error = 走直线"
  → 撞上障碍物
```

### 修复后的效果

```
MPC看到的代价函数：
  cost = tracking_error(X) + obstacle_cost(X)
                              ^^^^^^^^^^^^^^^^
                              障碍物代价与X相关！

优化器思考：
  "如果我走直线，tracking_error小，但obstacle_cost大"
  "如果我绕行，tracking_error大，但obstacle_cost小"
  "我选择总代价最小的方案"
  → 绕行成功！
```

## ⚙️ 配置参数

`config/mpc_controller_params.yaml`:

```yaml
# 障碍物避障权重
weight_obstacle: 1000.0  # 越大越重视避障

# 衰减率
decay_rate: 10.0  # 越大惩罚衰减越快

# 机器人半径
robot_radius: 0.5  # 机器人尺寸

# 预测时域
prediction_horizon: 10  # 预测步数
```

**调整建议**：
- 如果避障不够：增大 `weight_obstacle` (1000 → 5000)
- 如果过于保守：减小 `weight_obstacle` (1000 → 500)
- 如果反应太慢：增大 `prediction_horizon` (10 → 15)

## 🚀 使用方法

```bash
# 编译
cd /home/zjq/thesis
colcon build --packages-select unitree_obstacle_avoidance
source install/setup.bash

# 启动
ros2 launch unitree_obstacle_avoidance full_pipeline.launch.py

# 发布目标
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}" --once

# 观察：机器人应该绕过障碍物
```

## 🔍 验证方法

### 在RViz中验证

```bash
rviz2
```

添加：
1. **MarkerArray** → `/obstacles_markers` (障碍物椭圆)
2. **MarkerArray** → `/mpc_markers` (MPC预测轨迹)
3. **Path** → `/cmd_vel` (实际路径)

**预期行为**：
- ✅ MPC预测轨迹应该绕过障碍物椭圆
- ✅ 机器人实际路径应该避开障碍物
- ❌ 不应该直线穿过障碍物

### 日志检查

```bash
# 查看障碍物信息
ros2 topic echo /obstacles

# 查看控制输出
ros2 topic echo /cmd_vel

# 应该看到：
# - linear.x 不是固定值
# - angular.z 有变化（在避障时转向）
```

## 🐛 如果仍然不避障

### 检查清单

1. ✅ 障碍物数据正常？
   ```bash
   ros2 topic hz /obstacles  # 应该有输出
   ```

2. ✅ 权重足够大？
   ```yaml
   weight_obstacle: 1000.0  # 尝试增大到5000
   ```

3. ✅ 障碍物位置合理？
   ```bash
   ros2 topic echo /obstacles --once
   # 检查 center.x, center.y 是否在路径上
   ```

4. ✅ 预测时域足够？
   ```yaml
   prediction_horizon: 10  # 尝试增大到15
   ```

## 📝 关键要点总结

### ❌ 错误理解

> "机器人在原点，所以障碍物代价应该基于原点计算"

### ✅ 正确理解

> "机器人当前在原点，但MPC预测未来会移动到其他位置，必须检查每个预测位置是否会撞障碍物"

### 核心修改

```cpp
// 3个关键修改：

// 1. 调用时传入预测位置
cost += weight_obs_ * computeObstacleCost(X(0, k), X(1, k));
//                                         ^^^^^^^^  ^^^^^^^^
//                                      MPC第k步的预测位置

// 2. 函数接收预测位置
MX computeObstacleCost(const MX& robot_x, const MX& robot_y) {
//                                ^^^^^^^       ^^^^^^^
//                              预测位置参数

// 3. 使用预测位置计算距离
MX dx = robot_x - obs.x;  // 预测位置到障碍物
MX dy = robot_y - obs.y;
```

## 📚 相关文档

- MPC控制器指南: `MPC_CONTROLLER_GUIDE.md`
- 完整README: `README.md`

---

**总结**：局部规划器场景下，即使机器人本体始终在原点，MPC的预测轨迹会移动，**必须用预测位置计算障碍物代价**，否则MPC无法区分"直行"和"绕行"的差别，会选择最短路径直接撞上去！🎯

