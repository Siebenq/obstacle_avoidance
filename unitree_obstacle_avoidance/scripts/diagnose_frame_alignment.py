#!/usr/bin/env python3
"""
坐标系对齐诊断工具
检查点云和障碍物是否在同一坐标系中
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from unitree_obstacle_avoidance.msg import EllipseObstacleArray
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class FrameAlignmentDiagnostic(Node):
    def __init__(self):
        super().__init__('frame_alignment_diagnostic')
        
        self.cloud_data = None
        self.obstacles_data = None
        
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/pointcloud_fused', self.cloud_callback, 10)
        self.sub_obstacles = self.create_subscription(
            EllipseObstacleArray, '/obstacles', self.obstacles_callback, 10)
        
        # 发布调试标记
        self.debug_marker_pub = self.create_publisher(
            MarkerArray, '/debug_alignment_markers', 10)
        
        self.timer = self.create_timer(2.0, self.diagnose)
        
        self.get_logger().info('='*70)
        self.get_logger().info('坐标系对齐诊断工具已启动')
        self.get_logger().info('='*70)
    
    def cloud_callback(self, msg):
        self.cloud_data = msg
    
    def obstacles_callback(self, msg):
        self.obstacles_data = msg
    
    def diagnose(self):
        if self.cloud_data is None:
            self.get_logger().warn('⚠️  等待点云数据...')
            return
        
        if self.obstacles_data is None:
            self.get_logger().warn('⚠️  等待障碍物数据...')
            return
        
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('坐标系诊断报告')
        self.get_logger().info('='*70)
        
        # 1. 检查坐标系名称
        cloud_frame = self.cloud_data.header.frame_id
        obstacles_frame = self.obstacles_data.header.frame_id
        
        self.get_logger().info(f'[坐标系] 点云: "{cloud_frame}"')
        self.get_logger().info(f'[坐标系] 障碍物: "{obstacles_frame}"')
        
        if cloud_frame != obstacles_frame:
            self.get_logger().error(f'❌ 坐标系不匹配！')
            self.get_logger().error(f'   点云在 "{cloud_frame}" 坐标系')
            self.get_logger().error(f'   障碍物在 "{obstacles_frame}" 坐标系')
            self.get_logger().error('解决方案：')
            self.get_logger().error('  1. 检查 config/fusion_params.yaml 中的 target_frame')
            self.get_logger().error('  2. 检查 config/obstacle_detection_params.yaml 中的 frame_id')
            self.get_logger().error('  3. 确保两者一致！')
        else:
            self.get_logger().info(f'✅ 坐标系匹配: "{cloud_frame}"')
        
        # 2. 提取点云数据
        try:
            points = []
            for p in pc2.read_points(self.cloud_data, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if not points:
                self.get_logger().warn('⚠️  点云为空')
                return
            
            points = np.array(points)
            
            # 3. 计算点云范围
            cloud_min = points.min(axis=0)
            cloud_max = points.max(axis=0)
            cloud_center = points.mean(axis=0)
            
            self.get_logger().info('\n[点云统计]')
            self.get_logger().info(f'  点数: {len(points)}')
            self.get_logger().info(f'  X范围: [{cloud_min[0]:.2f}, {cloud_max[0]:.2f}] m')
            self.get_logger().info(f'  Y范围: [{cloud_min[1]:.2f}, {cloud_max[1]:.2f}] m')
            self.get_logger().info(f'  Z范围: [{cloud_min[2]:.2f}, {cloud_max[2]:.2f}] m')
            self.get_logger().info(f'  中心: ({cloud_center[0]:.2f}, {cloud_center[1]:.2f}, {cloud_center[2]:.2f})')
            
            # 4. 检查障碍物位置
            n_obstacles = len(self.obstacles_data.obstacles)
            self.get_logger().info(f'\n[障碍物统计]')
            self.get_logger().info(f'  数量: {n_obstacles}')
            
            if n_obstacles == 0:
                self.get_logger().warn('  ⚠️  未检测到障碍物')
                return
            
            # 5. 逐个检查障碍物位置是否在点云范围内
            markers = MarkerArray()
            alignment_ok = 0
            alignment_bad = 0
            
            for i, obs in enumerate(self.obstacles_data.obstacles):
                x, y, z = obs.center.x, obs.center.y, obs.center.z
                
                # 检查XY是否在点云范围内（Z=0是投影，不检查）
                x_in_range = cloud_min[0] <= x <= cloud_max[0]
                y_in_range = cloud_min[1] <= y <= cloud_max[1]
                
                # 扩展范围5m（因为点云可能不包含整个障碍物）
                x_reasonable = (cloud_min[0] - 5.0) <= x <= (cloud_max[0] + 5.0)
                y_reasonable = (cloud_min[1] - 5.0) <= y <= (cloud_max[1] + 5.0)
                
                status = '✅' if (x_reasonable and y_reasonable) else '❌'
                
                if x_reasonable and y_reasonable:
                    alignment_ok += 1
                else:
                    alignment_bad += 1
                
                self.get_logger().info(
                    f'  {status} 障碍物{i}: 位置({x:.2f}, {y:.2f}, {z:.2f}), '
                    f'距离{obs.distance:.2f}m, 高度{obs.height:.2f}m')
                
                if not (x_reasonable and y_reasonable):
                    self.get_logger().warn(
                        f'      ⚠️  位置异常！不在点云范围内')
                    self.get_logger().warn(
                        f'      点云X范围: [{cloud_min[0]:.2f}, {cloud_max[0]:.2f}]')
                    self.get_logger().warn(
                        f'      点云Y范围: [{cloud_min[1]:.2f}, {cloud_max[1]:.2f}]')
                
                # 创建调试标记：在障碍物位置放一个箭头指向上方
                marker = Marker()
                marker.header.frame_id = obstacles_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "debug_arrows"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 0.707
                marker.pose.orientation.x = 0.707
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                
                marker.scale.x = 0.5  # 箭头长度
                marker.scale.y = 0.1  # 箭杆宽度
                marker.scale.z = 0.1  # 箭头宽度
                
                if x_reasonable and y_reasonable:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                marker.color.a = 1.0
                
                marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
                markers.markers.append(marker)
            
            # 发布调试标记
            self.debug_marker_pub.publish(markers)
            
            # 6. 总结
            self.get_logger().info('\n[诊断结果]')
            if alignment_bad == 0:
                self.get_logger().info(f'✅ 所有 {n_obstacles} 个障碍物位置正常')
            else:
                self.get_logger().error(f'❌ {alignment_bad}/{n_obstacles} 个障碍物位置异常！')
                self.get_logger().error('\n可能的原因：')
                self.get_logger().error('  1. 坐标系设置错误')
                self.get_logger().error('  2. 点云融合的坐标变换有误')
                self.get_logger().error('  3. 障碍物检测使用了错误的坐标系')
                self.get_logger().error('\n建议检查：')
                self.get_logger().error('  • config/fusion_params.yaml → target_frame')
                self.get_logger().error('  • config/obstacle_detection_params.yaml → frame_id')
                self.get_logger().error('  • 确保点云融合的坐标变换正确')
            
            self.get_logger().info('\n💡 在RViz中查看：')
            self.get_logger().info('  • 添加 PointCloud2 → /pointcloud_fused')
            self.get_logger().info('  • 添加 MarkerArray → /obstacles_markers')
            self.get_logger().info('  • 添加 MarkerArray → /debug_alignment_markers (绿色箭头=正常, 红色=异常)')
            self.get_logger().info('='*70 + '\n')
            
        except Exception as e:
            self.get_logger().error(f'诊断失败: {str(e)}')

def main():
    rclpy.init()
    node = FrameAlignmentDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

