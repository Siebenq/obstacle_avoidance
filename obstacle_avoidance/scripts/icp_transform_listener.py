#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ICP变换矩阵监听脚本
订阅TransformMatrix话题并解析显示
"""

import rclpy
from rclpy.node import Node
from obstacle_avoidance.msg import TransformMatrix
import numpy as np


class ICPTransformListener(Node):
    def __init__(self):
        super().__init__('icp_transform_listener')
        
        self.declare_parameter('input_topic', '/icp_transform')
        
        topic = self.get_parameter('input_topic').as_string()
        
        self.subscription = self.create_subscription(
            TransformMatrix,
            topic,
            self.transform_callback,
            10)
        
        self.get_logger().info(f'📡 监听变换矩阵话题: {topic}')
        self.count = 0
    
    def transform_callback(self, msg):
        self.count += 1
        
        # 解析矩阵
        matrix = np.array(msg.matrix).reshape(4, 4)
        
        # 提取旋转和平移
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]
        
        # 计算欧拉角
        from scipy.spatial.transform import Rotation
        r = Rotation.from_matrix(rotation)
        euler = r.as_euler('xyz', degrees=True)
        
        print("\n" + "="*60)
        print(f"📊 ICP标定结果 #{self.count}")
        print("="*60)
        
        print(f"\n✅ 收敛状态: {'✓ 已收敛' if msg.converged else '✗ 未收敛'}")
        print(f"📏 适配度分数: {msg.fitness_score:.6f}")
        print(f"📐 RMSE: {msg.rmse:.6f} m")
        print(f"🔄 迭代次数: {msg.num_iterations}")
        
        print(f"\n📍 坐标系信息:")
        print(f"  源坐标系: {msg.source_frame_id} ({msg.source_points} 点)")
        print(f"  目标坐标系: {msg.target_frame_id} ({msg.target_points} 点)")
        
        print(f"\n🎯 变换矩阵 (4x4):")
        for i in range(4):
            print(f"  [{matrix[i,0]:8.5f} {matrix[i,1]:8.5f} {matrix[i,2]:8.5f} {matrix[i,3]:8.5f}]")
        
        print(f"\n📦 平移向量 (x, y, z):")
        print(f"  [{translation[0]:8.5f}, {translation[1]:8.5f}, {translation[2]:8.5f}] m")
        
        print(f"\n🔄 旋转矩阵 (3x3):")
        for i in range(3):
            print(f"  [{rotation[i,0]:8.5f} {rotation[i,1]:8.5f} {rotation[i,2]:8.5f}]")
        
        print(f"\n📐 欧拉角 (roll, pitch, yaw):")
        print(f"  [{euler[0]:7.3f}°, {euler[1]:7.3f}°, {euler[2]:7.3f}°]")
        
        # 质量评估
        print(f"\n🎯 标定质量评估:")
        if msg.fitness_score < 0.001:
            quality = "优秀 ⭐⭐⭐⭐⭐"
        elif msg.fitness_score < 0.01:
            quality = "良好 ⭐⭐⭐⭐"
        elif msg.fitness_score < 0.1:
            quality = "可接受 ⭐⭐⭐"
        else:
            quality = "较差 ⭐"
        
        print(f"  {quality}")
        
        if not msg.converged:
            print(f"\n⚠️  警告: ICP未收敛，结果可能不准确")
        
        if msg.fitness_score > 0.1:
            print(f"\n💡 建议: 适配度分数较高，请检查:")
            print(f"   - 两个点云是否有足够重叠")
            print(f"   - 点云质量是否良好")
            print(f"   - ICP参数是否需要调整")
        
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    node = ICPTransformListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

