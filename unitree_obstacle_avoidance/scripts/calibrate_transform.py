#!/usr/bin/env python3

"""
点云坐标变换校准工具

使用方法：
1. 在两个相机前放置同一个物体
2. 运行此脚本
3. 根据输出调整 fusion_params.yaml 中的变换参数
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class TransformCalibrator(Node):
    def __init__(self):
        super().__init__('transform_calibrator')
        
        self.declare_parameter('topic1', '/camera_dog/points')
        self.declare_parameter('topic2', '/camera_person/points')
        
        topic1 = self.get_parameter('topic1').value
        topic2 = self.get_parameter('topic2').value
        
        self.cloud1_data = None
        self.cloud2_data = None
        
        self.sub1 = self.create_subscription(
            PointCloud2, topic1, self.cloud1_callback, 10)
        self.sub2 = self.create_subscription(
            PointCloud2, topic2, self.cloud2_callback, 10)
        
        self.timer = self.create_timer(2.0, self.analyze_callback)
        
        self.get_logger().info('校准工具已启动')
        self.get_logger().info(f'订阅话题1: {topic1}')
        self.get_logger().info(f'订阅话题2: {topic2}')
        self.get_logger().info('\n请在两个相机前放置同一个物体...')
    
    def cloud1_callback(self, msg):
        self.cloud1_data = msg
    
    def cloud2_callback(self, msg):
        self.cloud2_data = msg
    
    def analyze_callback(self):
        if self.cloud1_data is None or self.cloud2_data is None:
            self.get_logger().warn('等待点云数据...')
            return
        
        try:
            # 转换点云数据
            points1 = self.extract_points(self.cloud1_data)
            points2 = self.extract_points(self.cloud2_data)
            
            if len(points1) == 0 or len(points2) == 0:
                self.get_logger().warn('点云为空')
                return
            
            # 计算中心点
            center1 = np.mean(points1, axis=0)
            center2 = np.mean(points2, axis=0)
            
            # 计算边界
            min1 = np.min(points1, axis=0)
            max1 = np.max(points1, axis=0)
            min2 = np.min(points2, axis=0)
            max2 = np.max(points2, axis=0)
            
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('点云分析结果:')
            self.get_logger().info(f'点云1: {len(points1)} 点')
            self.get_logger().info(f'  中心: ({center1[0]:.3f}, {center1[1]:.3f}, {center1[2]:.3f})')
            self.get_logger().info(f'  范围: X[{min1[0]:.3f}, {max1[0]:.3f}], '
                                 f'Y[{min1[1]:.3f}, {max1[1]:.3f}], '
                                 f'Z[{min1[2]:.3f}, {max1[2]:.3f}]')
            
            self.get_logger().info(f'\n点云2: {len(points2)} 点')
            self.get_logger().info(f'  中心: ({center2[0]:.3f}, {center2[1]:.3f}, {center2[2]:.3f})')
            self.get_logger().info(f'  范围: X[{min2[0]:.3f}, {max2[0]:.3f}], '
                                 f'Y[{min2[1]:.3f}, {max2[1]:.3f}], '
                                 f'Z[{min2[2]:.3f}, {max2[2]:.3f}]')
            
            # 计算粗略的平移估计
            # 假设两个相机看到的是同一个物体，中心应该对齐
            translation = center1 - center2
            
            self.get_logger().info('\n建议的变换参数（粗略估计）:')
            self.get_logger().info(f'  transform_x: {translation[0]:.3f}')
            self.get_logger().info(f'  transform_y: {translation[1]:.3f}')
            self.get_logger().info(f'  transform_z: {translation[2]:.3f}')
            
            # 计算可能的旋转（基于物体方向）
            if len(points1) > 10 and len(points2) > 10:
                # 使用PCA估计主方向
                _, _, vh1 = np.linalg.svd(points1 - center1)
                _, _, vh2 = np.linalg.svd(points2 - center2)
                
                # 主方向向量
                dir1 = vh1[0]
                dir2 = vh2[0]
                
                # 计算旋转角度（只考虑yaw）
                angle1 = np.arctan2(dir1[1], dir1[0])
                angle2 = np.arctan2(dir2[1], dir2[0])
                yaw = angle1 - angle2
                
                # 归一化到[-pi, pi]
                while yaw > np.pi:
                    yaw -= 2 * np.pi
                while yaw < -np.pi:
                    yaw += 2 * np.pi
                
                self.get_logger().info(f'  transform_yaw: {yaw:.3f} ({np.degrees(yaw):.1f}°)')
            
            self.get_logger().info('\n将这些值复制到 config/fusion_params.yaml')
            self.get_logger().info('='*60 + '\n')
            
        except Exception as e:
            self.get_logger().error(f'分析失败: {str(e)}')
    
    def extract_points(self, msg):
        """从PointCloud2消息中提取点坐标"""
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

def main(args=None):
    rclpy.init(args=args)
    node = TransformCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

