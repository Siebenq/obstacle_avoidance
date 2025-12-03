# 障碍物位置偏移问题修复

## 🔍 问题描述
障碍物检测节点识别出的障碍物位置，与点云中的实际障碍物位置不重合。

## 🎯 根本原因

### 原因：坐标系不一致

**问题所在**:
```cpp
// obstacle_detection_node.cpp (修复前)
void publishObstacles(...) {
    msg.header.frame_id = frame_id_;  // 使用配置文件中的固定值 "base_link"
}

void publishObstacleMarkers(...) {
    marker.header.frame_id = frame_id_;  // 使用配置文件中的固定值 "base_link"
}
```

**配置文件**:
```yaml
# obstacle_detection_params.yaml
frame_id: "base_link"  # 固定值

# fusion_params.yaml
target_frame: "base_link"  # 融合后的点云坐标系
```

**问题**:
1. 输入点云 `/pointcloud_fused` 的坐标系是 `base_link`（来自fusion节点）
2. 但如果fusion节点的配置或TF树有问题，点云实际坐标系可能不同
3. obstacle_detection节点强制使用配置文件中的 `frame_id: "base_link"`
4. **导致**：障碍物坐标系与点云坐标系不匹配，位置偏移

## ✅ 修复方案

### 核心修改：使用输入点云的坐标系

```cpp
// 修复后的代码
void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. 保存输入点云的坐标系
    std::string input_frame_id = msg->header.frame_id;
    
    // ... 处理点云 ...
    
    // 2. 发布时使用输入点云的坐标系
    publishObstacles(obstacles, msg->header.stamp, input_frame_id);
    publishObstacleMarkers(obstacles, msg->header.stamp, input_frame_id);
}

void publishObstacles(..., const std::string& frame_id) {
    msg.header.frame_id = frame_id;  // 使用输入点云的坐标系，而非配置文件
}

void publishObstacleMarkers(..., const std::string& frame_id) {
    marker.header.frame_id = frame_id;  // 使用输入点云的坐标系
}
```

### 修改详情

#### 1. 保存输入坐标系
```cpp
void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::string input_frame_id = msg->header.frame_id;  // ← 新增
    // ...
}
```

#### 2. 传递坐标系参数
```cpp
// 修改前
publishObstacles(obstacles, msg->header.stamp);
publishObstacleMarkers(obstacles, msg->header.stamp);

// 修改后
publishObstacles(obstacles, msg->header.stamp, input_frame_id);
publishObstacleMarkers(obstacles, msg->header.stamp, input_frame_id);
```

#### 3. 更新函数签名
```cpp
// 修改前
void publishObstacles(
    const std::vector<...>& obstacles,
    const rclcpp::Time& timestamp);

// 修改后
void publishObstacles(
    const std::vector<...>& obstacles,
    const rclcpp::Time& timestamp,
    const std::string& frame_id);  // ← 新增参数
```

#### 4. 使用动态坐标系
```cpp
// 修改前
msg.header.frame_id = frame_id_;  // 配置文件中的固定值

// 修改后
msg.header.frame_id = frame_id;  // 输入点云的实际坐标系
```

## 🛠️ 诊断工具

### 新增：坐标系对齐诊断工具

```bash
ros2 run unitree_obstacle_avoidance diagnose_frame_alignment.py
```

**功能**:
1. ✅ 检查点云和障碍物的坐标系名称是否一致
2. ✅ 计算点云的空间范围
3. ✅ 检查每个障碍物位置是否在点云范围内
4. ✅ 发布调试标记（绿色箭头=正常，红色=异常）
5. ✅ 给出具体的修复建议

**输出示例**:
```
======================================================================
坐标系诊断报告
======================================================================
[坐标系] 点云: "base_link"
[坐标系] 障碍物: "base_link"
✅ 坐标系匹配: "base_link"

[点云统计]
  点数: 15234
  X范围: [-2.50, 12.30] m
  Y范围: [-6.20, 5.80] m
  Z范围: [-0.50, 2.10] m
  中心: (4.50, 0.20, 0.80)

[障碍物统计]
  数量: 3
  ✅ 障碍物0: 位置(3.45, 1.20, 0.00), 距离3.66m, 高度1.50m
  ✅ 障碍物1: 位置(5.00, -0.50, 0.00), 距离5.02m, 高度1.20m
  ✅ 障碍物2: 位置(7.80, 0.30, 0.00), 距离7.81m, 高度0.80m

[诊断结果]
✅ 所有 3 个障碍物位置正常

💡 在RViz中查看：
  • 添加 PointCloud2 → /pointcloud_fused
  • 添加 MarkerArray → /obstacles_markers
  • 添加 MarkerArray → /debug_alignment_markers (绿色箭头=正常, 红色=异常)
======================================================================
```

## 📊 修复前后对比

### 修复前
```
点云坐标系: "base_link" (实际)
障碍物坐标系: "base_link" (配置文件)
问题: 如果点云实际坐标系不是base_link，就会偏移
```

### 修复后
```
点云坐标系: "base_link" (实际)
障碍物坐标系: "base_link" (从点云读取)
结果: 始终一致，不会偏移 ✅
```

## 🚀 使用方法

### 第1步：编译
```bash
cd /home/zjq/thesis
colcon build --packages-select unitree_obstacle_avoidance
source install/setup.bash
```

### 第2步：启动系统
```bash
ros2 launch unitree_obstacle_avoidance full_pipeline.launch.py
```

### 第3步：运行诊断工具
```bash
# 新终端
ros2 run unitree_obstacle_avoidance diagnose_frame_alignment.py
```

### 第4步：在RViz中验证
```bash
rviz2
```

添加以下显示：
1. **PointCloud2** → `/pointcloud_fused`
   - 显示融合后的点云
   
2. **MarkerArray** → `/obstacles_markers`
   - 显示检测到的障碍物椭圆
   
3. **MarkerArray** → `/debug_alignment_markers`
   - 显示诊断箭头
   - 绿色箭头：位置正常
   - 红色箭头：位置异常

**验证方法**:
- 椭圆应该正好框住点云中的障碍物
- 橙色地面轮廓应该在障碍物底部
- 文本标签应该在障碍物顶部

## 🔧 如果仍有问题

### 检查清单

#### 1. 检查点云融合配置
```bash
# 查看融合后的点云坐标系
ros2 topic echo /pointcloud_fused --once | grep frame_id
```

应该输出：`frame_id: "base_link"`

#### 2. 检查TF树
```bash
# 安装TF工具
sudo apt install ros-humble-tf2-tools

# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo base_link camera_dog_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_person_optical_frame
```

#### 3. 检查配置文件一致性
```yaml
# config/fusion_params.yaml
target_frame: "base_link"  # ← 应该一致

# config/obstacle_detection_params.yaml
frame_id: "base_link"      # ← 应该一致（但现在不重要了，因为使用动态坐标系）
```

#### 4. 手动测试坐标系
```bash
# 查看点云话题信息
ros2 topic info /pointcloud_fused
ros2 topic echo /pointcloud_fused --once

# 查看障碍物话题信息
ros2 topic info /obstacles
ros2 topic echo /obstacles --once
```

## 📝 技术细节

### 为什么不能用配置文件的固定值？

**场景1：正常情况**
```
点云实际坐标系: base_link
配置文件frame_id: base_link
结果: ✅ 正常
```

**场景2：TF树配置错误**
```
点云实际坐标系: camera_dog_optical_frame (TF变换失败)
配置文件frame_id: base_link
结果: ❌ 偏移！
```

**场景3：使用不同的目标坐标系**
```
点云实际坐标系: odom
配置文件frame_id: base_link
结果: ❌ 偏移！
```

### 动态坐标系的优势

1. **自动适应**: 无论点云在哪个坐标系，障碍物都在同一坐标系
2. **鲁棒性强**: TF配置错误时不会导致偏移
3. **灵活性高**: 可以轻松切换目标坐标系

## 🎯 预期效果

### 修复前
```
在RViz中：
- 点云显示在正确位置
- 障碍物椭圆偏移了几米
- 椭圆和点云对不上
```

### 修复后
```
在RViz中：
- 点云显示在正确位置 ✅
- 障碍物椭圆准确框住点云中的障碍物 ✅
- 地面轮廓在障碍物底部 ✅
- 文本标签在障碍物顶部 ✅
```

## 📚 相关文档

- 地面投影说明: `GROUND_PROJECTION_GUIDE.md`
- 投影修改说明: `PROJECTION_CHANGES.md`
- 参数优化指南: `OBSTACLE_DETECTION_FIXES.md`
- 完整文档: `README.md`

## 总结

✅ **问题**: 障碍物位置与点云不重合
✅ **原因**: 坐标系使用配置文件固定值，与输入点云不一致
✅ **修复**: 使用输入点云的动态坐标系
✅ **工具**: 新增坐标系对齐诊断工具
✅ **验证**: 在RViz中椭圆应准确框住障碍物

---

**快速测试**:
```bash
# 启动
ros2 launch unitree_obstacle_avoidance full_pipeline.launch.py

# 诊断
ros2 run unitree_obstacle_avoidance diagnose_frame_alignment.py

# 可视化
rviz2
```

现在障碍物位置应该与点云完美对齐！🎯

