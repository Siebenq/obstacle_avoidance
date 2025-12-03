# 椭圆障碍物使用指南

## 概述

障碍物检测节点已升级，现在将检测到的障碍物简化为**椭圆**形式，并通过自定义消息类型发布。这种表示方法更适合避障算法使用。

## 椭圆障碍物的优势

### 1. 数据高效
- 每个障碍物只需几个参数（中心、长短半轴、旋转角）
- 相比原始点云，大幅减少数据量
- 降低通信和计算开销

### 2. 几何直观
- 椭圆准确描述障碍物在地面的投影
- 保留主要方向信息（长轴方向）
- 便于计算距离和碰撞检测

### 3. 算法友好
- 椭圆-点距离计算简单高效
- 支持旋转变换
- 易于集成到路径规划算法

## 自定义消息定义

### EllipseObstacle.msg

```msg
# 障碍物ID
int32 id

# 椭圆中心位置（3D坐标）
geometry_msgs/Point center

# 椭圆参数（在XY平面上）
float64 semi_major_axis    # 长半轴（米）
float64 semi_minor_axis    # 短半轴（米）
float64 rotation_angle     # 旋转角度（弧度，相对于X轴）

# 高度信息
float64 height             # 障碍物高度（米）
float64 z_min              # 最低点Z坐标（米）
float64 z_max              # 最高点Z坐标（米）

# 附加信息
int32 point_count          # 点云数量
float64 distance           # 到原点的距离（米）
```

### EllipseObstacleArray.msg

```msg
# 消息头
std_msgs/Header header

# 障碍物数组
EllipseObstacle[] obstacles
```

## 如何订阅椭圆障碍物

### C++ 示例

```cpp
#include "rclcpp/rclcpp.hpp"
#include "pointcloud_fusion/msg/ellipse_obstacle_array.hpp"

class ObstacleSubscriber : public rclcpp::Node
{
public:
  ObstacleSubscriber() : Node("obstacle_subscriber")
  {
    subscription_ = this->create_subscription<
      pointcloud_fusion::msg::EllipseObstacleArray>(
      "/obstacles", 10,
      std::bind(&ObstacleSubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const pointcloud_fusion::msg::EllipseObstacleArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "接收到 %zu 个障碍物", msg->obstacles.size());
    
    for (const auto& obs : msg->obstacles) {
      RCLCPP_INFO(this->get_logger(),
        "障碍物 %d: 中心(%.2f, %.2f), 长轴=%.2f, 短轴=%.2f, 角度=%.2f°",
        obs.id,
        obs.center.x, obs.center.y,
        obs.semi_major_axis, obs.semi_minor_axis,
        obs.rotation_angle * 180.0 / M_PI);
    }
  }

  rclcpp::Subscription<pointcloud_fusion::msg::EllipseObstacleArray>::SharedPtr subscription_;
};
```

### Python 示例

```python
import rclpy
from rclpy.node import Node
from pointcloud_fusion.msg import EllipseObstacleArray
import math

class ObstacleSubscriber(Node):
    def __init__(self):
        super().__init__('obstacle_subscriber')
        self.subscription = self.create_subscription(
            EllipseObstacleArray,
            '/obstacles',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(f'接收到 {len(msg.obstacles)} 个障碍物')
        
        for obs in msg.obstacles:
            self.get_logger().info(
                f'障碍物 {obs.id}: '
                f'中心({obs.center.x:.2f}, {obs.center.y:.2f}), '
                f'长轴={obs.semi_major_axis:.2f}, '
                f'短轴={obs.semi_minor_axis:.2f}, '
                f'角度={math.degrees(obs.rotation_angle):.2f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 椭圆参数说明

### 坐标系统

- 所有坐标在 `frame_id` 指定的坐标系下（默认 `base_link`）
- X轴通常指向前方，Y轴指向左侧，Z轴向上

### 椭圆参数

1. **center (geometry_msgs/Point)**
   - 椭圆中心的3D坐标
   - `center.z` 通常是障碍物的质心高度

2. **semi_major_axis (float64)**
   - 长半轴长度（米）
   - 障碍物在主方向上的尺寸
   - 通过PCA计算得出

3. **semi_minor_axis (float64)**
   - 短半轴长度（米）
   - 障碍物在次方向上的尺寸

4. **rotation_angle (float64)**
   - 椭圆长轴相对于X轴的旋转角（弧度）
   - 范围: [-π, π]
   - 正值表示逆时针旋转

5. **height (float64)**
   - 障碍物总高度 = z_max - z_min

## 碰撞检测示例

### 点到椭圆的距离

```cpp
double distanceToEllipse(double px, double py, const EllipseObstacle& obs)
{
  // 将点转换到椭圆局部坐标系
  double dx = px - obs.center.x;
  double dy = py - obs.center.y;
  
  // 旋转到椭圆坐标系
  double cos_a = std::cos(-obs.rotation_angle);
  double sin_a = std::sin(-obs.rotation_angle);
  double lx = dx * cos_a - dy * sin_a;
  double ly = dx * sin_a + dy * cos_a;
  
  // 归一化
  double nx = lx / obs.semi_major_axis;
  double ny = ly / obs.semi_minor_axis;
  
  // 到椭圆边界的归一化距离
  double dist_norm = std::sqrt(nx*nx + ny*ny);
  
  // 实际距离（近似）
  if (dist_norm <= 1.0) {
    return 0.0;  // 点在椭圆内
  } else {
    // 点在椭圆外，计算到边界的距离
    double scale = std::max(obs.semi_major_axis, obs.semi_minor_axis);
    return (dist_norm - 1.0) * scale;
  }
}
```

## 可视化

### 在RViz中查看

1. 添加 **MarkerArray** 显示
2. 订阅 `/obstacles_markers` 话题
3. 障碍物将显示为椭圆柱体（圆柱体近似）

### 查看消息内容

```bash
# 查看障碍物话题
ros2 topic echo /obstacles

# 查看消息结构
ros2 interface show pointcloud_fusion/msg/EllipseObstacle
ros2 interface show pointcloud_fusion/msg/EllipseObstacleArray
```

## PCA椭圆拟合原理

障碍物检测使用**主成分分析（PCA）**来计算椭圆参数：

1. 将障碍物点云投影到XY平面
2. 计算点云的协方差矩阵
3. 特征值分解得到主方向（特征向量）和方差（特征值）
4. 长半轴 = √(最大特征值) × 2 × scale_factor
5. 短半轴 = √(最小特征值) × 2 × scale_factor
6. 旋转角 = 最大特征向量的方向角

默认 `scale_factor = 1.2`，可通过参数调整。

## 常见问题

### 1. 椭圆太小/太大？

调整 `config/obstacle_detection_params.yaml` 中的 `ellipse_scale_factor`:

```yaml
ellipse_scale_factor: 1.5  # 增大以获得更保守的椭圆
```

### 2. 如何访问自定义消息？

确保：
- 项目已正确编译: `colcon build --packages-select pointcloud_fusion`
- 已加载环境: `source install/setup.bash`
- Python路径正确设置

### 3. 椭圆方向不对？

- 检查 `rotation_angle` 的单位（弧度，不是角度）
- 确认坐标系方向（X轴向前，Y轴向左）
- 长轴指向障碍物的主延伸方向

## 后续应用

椭圆障碍物可用于：

- ✅ **避障算法**: 快速碰撞检测
- ✅ **路径规划**: 代价地图生成
- ✅ **碰撞预测**: 与轨迹生成结合
- ✅ **障碍物跟踪**: 基于椭圆参数匹配
- ✅ **空间分析**: 可通行区域计算

## 参考资料

- PCL文档: https://pointclouds.org/
- ROS2自定义消息: https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html
- 主成分分析: https://en.wikipedia.org/wiki/Principal_component_analysis

