#!/usr/bin/env python3
"""
坐标变换诊断工具
用于诊断点云融合中的坐标变换问题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation

class TransformDiagnoser(Node):
    def __init__(self):
        super().__init__('transform_diagnoser')
        
        self.cloud1_data = None
        self.cloud2_data = None
        self.fused_data = None
        
        self.sub1 = self.create_subscription(
            PointCloud2, '/camera_dog/points', self.cloud1_callback, 10)
        self.sub2 = self.create_subscription(
            PointCloud2, '/camera_person/points', self.cloud2_callback, 10)
        self.sub_fused = self.create_subscription(
            PointCloud2, '/pointcloud_fused', self.fused_callback, 10)
        
        self.timer = self.create_timer(3.0, self.diagnose)
        
        self.get_logger().info('诊断工具已启动')
        self.get_logger().info('请确保障碍物同时出现在两个相机视野中...')
    
    def cloud1_callback(self, msg):
        self.cloud1_data = msg
    
    def cloud2_callback(self, msg):
        self.cloud2_data = msg
    
    def fused_callback(self, msg):
        self.fused_data = msg
    
    def extract_points(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points) if points else np.array([]).reshape(0, 3)
    
    def diagnose(self):
        if self.cloud1_data is None or self.cloud2_data is None:
            self.get_logger().warn('等待点云数据...')
            return
        
        try:
            points1 = self.extract_points(self.cloud1_data)
            points2 = self.extract_points(self.cloud2_data)
            
            if len(points1) == 0 or len(points2) == 0:
                return
            
            # 计算点云的统计信息
            center1 = np.mean(points1, axis=0)
            center2 = np.mean(points2, axis=0)
            std1 = np.std(points1, axis=0)
            std2 = np.std(points2, axis=0)
            
            self.get_logger().info('\n' + '='*70)
            self.get_logger().info('坐标变换诊断结果:')
            self.get_logger().info('-'*70)
            
            self.get_logger().info(f'点云1 ({self.cloud1_data.header.frame_id}):')
            self.get_logger().info(f'  点数: {len(points1)}')
            self.get_logger().info(f'  中心: X={center1[0]:.3f}, Y={center1[1]:.3f}, Z={center1[2]:.3f}')
            self.get_logger().info(f'  标准差: X={std1[0]:.3f}, Y={std1[1]:.3f}, Z={std1[2]:.3f}')
            
            self.get_logger().info(f'\n点云2 ({self.cloud2_data.header.frame_id}):')
            self.get_logger().info(f'  点数: {len(points2)}')
            self.get_logger().info(f'  中心: X={center2[0]:.3f}, Y={center2[1]:.3f}, Z={center2[2]:.3f}')
            self.get_logger().info(f'  标准差: X={std2[0]:.3f}, Y={std2[1]:.3f}, Z={std2[2]:.3f}')
            
            # 计算粗略的变换（假设看到同一物体）
            translation = center1 - center2
            self.get_logger().info(f'\n如果两个相机看到同一物体，需要的平移变换:')
            self.get_logger().info(f'  transform_x: {translation[0]:.3f}')
            self.get_logger().info(f'  transform_y: {translation[1]:.3f}')
            self.get_logger().info(f'  transform_z: {translation[2]:.3f}')
            
            # 分析融合结果
            if self.fused_data is not None:
                points_fused = self.extract_points(self.fused_data)
                if len(points_fused) > 0:
                    # 检查是否有重复点（两层现象）
                    # 使用聚类检测两层
                    from sklearn.cluster import DBSCAN
                    
                    clustering = DBSCAN(eps=0.05, min_samples=10).fit(points_fused)
                    n_clusters = len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)
                    
                    self.get_logger().info(f'\n融合点云分析:')
                    self.get_logger().info(f'  总点数: {len(points_fused)}')
                    self.get_logger().info(f'  检测到的聚类数: {n_clusters}')
                    
                    if n_clusters > len(points1) / 50 + len(points2) / 50:
                        self.get_logger().warn('  ⚠️  检测到异常多的聚类，可能存在两层现象！')
                        self.get_logger().warn('  建议：调整fusion_params.yaml中的变换参数')
                    
                    # 计算融合点云的密度
                    if len(points_fused) > len(points1) + len(points2) * 0.9:
                        density_ratio = len(points_fused) / (len(points1) + len(points2))
                        if density_ratio > 1.5:
                            self.get_logger().warn(f'  ⚠️  点云数量异常({density_ratio:.1f}x)，可能有重叠！')
            
            # 给出具体建议
            self.get_logger().info('\n建议的配置（config/fusion_params.yaml）:')
            self.get_logger().info('```yaml')
            self.get_logger().info(f'transform_x: {translation[0]:.3f}')
            self.get_logger().info(f'transform_y: {translation[1]:.3f}')
            self.get_logger().info(f'transform_z: {translation[2]:.3f}')
            self.get_logger().info('transform_roll: 0.0')
            self.get_logger().info('transform_pitch: 0.0')
            self.get_logger().info('transform_yaw: 0.0  # 如果物体旋转了，调整此值')
            self.get_logger().info('```')
            self.get_logger().info('='*70 + '\n')
            
        except Exception as e:
            self.get_logger().error(f'诊断失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main():
    rclpy.init()
    node = TransformDiagnoser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

