#!/usr/bin/env python3
"""
障碍物检测参数测试工具
实时显示点云处理各步骤的统计信息，帮助调整参数
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from unitree_obstacle_avoidance.msg import EllipseObstacleArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class ObstacleParamTester(Node):
    def __init__(self):
        super().__init__('obstacle_param_tester')
        
        self.cloud_data = None
        self.obstacles_data = None
        self.last_update = self.get_clock().now()
        
        self.sub_input = self.create_subscription(
            PointCloud2, '/pointcloud_fused', self.input_callback, 10)
        self.sub_obstacles = self.create_subscription(
            EllipseObstacleArray, '/obstacles', self.obstacles_callback, 10)
        
        self.timer = self.create_timer(2.0, self.analyze)
        
        self.get_logger().info('参数测试工具已启动')
        self.get_logger().info('建议：在RViz中同时查看 /pointcloud_fused 和 /obstacles_markers')
    
    def input_callback(self, msg):
        self.cloud_data = msg
    
    def obstacles_callback(self, msg):
        self.obstacles_data = msg
    
    def analyze(self):
        if self.cloud_data is None:
            self.get_logger().warn('等待点云数据... 请确保 pointcloud_fusion 节点正在运行')
            return
        
        try:
            # 提取点云
            points = []
            for p in pc2.read_points(self.cloud_data, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if not points:
                self.get_logger().warn('点云为空')
                return
            
            points = np.array(points)
            
            # 模拟各处理步骤
            self.get_logger().info('\n' + '='*70)
            self.get_logger().info('点云处理分析')
            self.get_logger().info('-'*70)
            
            # 原始点云
            self.get_logger().info(f'[输入] 原始点云: {len(points)} 点')
            
            # 模拟降采样 (0.2m网格)
            voxel_size = 0.2
            voxel_indices = np.floor(points / voxel_size).astype(int)
            unique_voxels = np.unique(voxel_indices, axis=0)
            downsampled_count = len(unique_voxels)
            downsample_ratio = downsampled_count / len(points) * 100
            self.get_logger().info(f'[步骤1] 降采样后: ~{downsampled_count} 点 ({downsample_ratio:.1f}%)')
            if downsample_ratio < 10:
                self.get_logger().warn('  ⚠️  降采样过于粗糙！建议减小 voxel_leaf_size 到 0.1')
            
            # 模拟ROI滤波
            roi_mask = (points[:, 0] >= 0.0) & (points[:, 0] <= 10.0) & \
                       (points[:, 1] >= -5.0) & (points[:, 1] <= 5.0) & \
                       (points[:, 2] >= -1.5) & (points[:, 2] <= 2.0)
            roi_points = points[roi_mask]
            roi_ratio = len(roi_points) / len(points) * 100
            self.get_logger().info(f'[步骤2] ROI滤波后: {len(roi_points)} 点 ({roi_ratio:.1f}%)')
            if roi_ratio < 30:
                self.get_logger().warn('  ⚠️  ROI过滤太多！建议放宽 x/y/z_filter 范围')
            
            # 分析地面点
            ground_mask = np.abs(roi_points[:, 2]) < 0.2
            ground_points = roi_points[ground_mask]
            above_ground = roi_points[~ground_mask]
            ground_ratio = len(ground_points) / len(roi_points) * 100 if len(roi_points) > 0 else 0
            self.get_logger().info(f'[步骤3] 地面点: {len(ground_points)} ({ground_ratio:.1f}%), ' + 
                                 f'非地面: {len(above_ground)} ({100-ground_ratio:.1f}%)')
            if ground_ratio > 80:
                self.get_logger().warn('  ⚠️  大部分是地面点！障碍物可能被误移除')
                self.get_logger().warn('  建议: 增大 ground_threshold 或设置 remove_ground: false')
            
            # 估计聚类数量（简化版）
            if len(above_ground) > 0:
                # 使用简单的空间分组估计
                cluster_tolerance = 0.5
                # 计算点之间的距离，估计可能的聚类数
                from scipy.spatial.distance import pdist
                if len(above_ground) < 1000:
                    distances = pdist(above_ground)
                    close_pairs = np.sum(distances < cluster_tolerance)
                    estimated_clusters = max(1, len(above_ground) // 20)  # 粗略估计
                else:
                    estimated_clusters = len(above_ground) // 50
                self.get_logger().info(f'[步骤4] 估计聚类数: ~{estimated_clusters}')
                
                # 检查点密度
                if len(above_ground) < 50:
                    self.get_logger().warn('  ⚠️  非地面点太少！可能无法形成有效聚类')
                    self.get_logger().warn('  建议: 减小 voxel_leaf_size 或 min_cluster_size')
            
            # 实际检测结果
            if self.obstacles_data:
                n_obstacles = len(self.obstacles_data.obstacles)
                self.get_logger().info(f'[结果] 实际检测到: {n_obstacles} 个障碍物')
                
                if n_obstacles == 0:
                    self.get_logger().error('  ❌ 未检测到任何障碍物!')
                    self.suggest_fixes(points, roi_points, above_ground)
                else:
                    self.get_logger().info('  ✅ 检测到障碍物:')
                    for i, obs in enumerate(self.obstacles_data.obstacles[:5]):  # 只显示前5个
                        self.get_logger().info(
                            f'    {i}: 位置({obs.center.x:.2f}, {obs.center.y:.2f}), ' +
                            f'尺寸{obs.semi_major_axis*2:.2f}×{obs.semi_minor_axis*2:.2f}m, ' +
                            f'高{obs.height:.2f}m, {obs.point_count}点')
                    if n_obstacles > 5:
                        self.get_logger().info(f'    ... 还有 {n_obstacles-5} 个')
            else:
                self.get_logger().warn('[结果] 未接收到障碍物数据')
            
            self.get_logger().info('='*70 + '\n')
            
        except Exception as e:
            self.get_logger().error(f'分析失败: {str(e)}')
    
    def suggest_fixes(self, points, roi_points, above_ground):
        """根据数据特征给出参数调整建议"""
        self.get_logger().info('\n  💡 参数调整建议:')
        
        # 检查降采样
        if len(points) > 10000:
            self.get_logger().info('  1. 降采样: voxel_leaf_size: 0.15  # 适中')
        else:
            self.get_logger().info('  1. 降采样: voxel_leaf_size: 0.1   # 保留更多细节')
        
        # 检查ROI
        if len(roi_points) < len(points) * 0.3:
            self.get_logger().info('  2. ROI范围: x_filter_max: 15.0, y_filter_min: -8.0, y_filter_max: 8.0')
        
        # 检查地面移除
        if len(above_ground) < 100:
            self.get_logger().info('  3. 地面移除: remove_ground: false  # 暂时禁用测试')
            self.get_logger().info('     或: ground_threshold: 0.3  # 放宽阈值')
        
        # 检查聚类
        if len(above_ground) < 50:
            self.get_logger().info('  4. 聚类: min_cluster_size: 5, cluster_tolerance: 0.8')
        
        # 检查尺寸过滤
        self.get_logger().info('  5. 尺寸过滤: min_obstacle_height: 0.05, min_obstacle_width: 0.02')

def main():
    rclpy.init()
    node = ObstacleParamTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

