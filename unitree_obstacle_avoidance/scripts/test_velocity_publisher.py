#!/usr/bin/env python3
"""
测试速度发布器 - 用于测试轨迹生成节点
发布模拟的速度命令到 /cmd_vel 话题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class TestVelocityPublisher(Node):
    def __init__(self):
        super().__init__('test_velocity_publisher')
        
        # 声明参数
        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('pattern', 'constant')  # constant, sine, circle
        self.declare_parameter('linear_x', 1.0)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('angular_z', 0.0)
        
        # 获取参数
        topic = self.get_parameter('topic').value
        frequency = self.get_parameter('frequency').value
        self.pattern = self.get_parameter('pattern').value
        self.linear_x = self.get_parameter('linear_x').value
        self.linear_y = self.get_parameter('linear_y').value
        self.angular_z = self.get_parameter('angular_z').value
        
        # 创建发布者
        self.publisher = self.create_publisher(Twist, topic, 10)
        
        # 创建定时器
        timer_period = 1.0 / frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.counter = 0
        
        self.get_logger().info(f'测试速度发布器已启动')
        self.get_logger().info(f'发布话题: {topic}')
        self.get_logger().info(f'频率: {frequency} Hz')
        self.get_logger().info(f'模式: {self.pattern}')
        self.get_logger().info(f'速度: linear_x={self.linear_x}, linear_y={self.linear_y}, angular_z={self.angular_z}')
        
    def timer_callback(self):
        msg = Twist()
        
        if self.pattern == 'constant':
            # 恒定速度
            msg.linear.x = self.linear_x
            msg.linear.y = self.linear_y
            msg.angular.z = self.angular_z
            
        elif self.pattern == 'sine':
            # 正弦波速度（X方向）
            t = self.counter * 0.1
            msg.linear.x = self.linear_x * math.sin(t)
            msg.linear.y = self.linear_y
            msg.angular.z = self.angular_z
            
        elif self.pattern == 'circle':
            # 圆周运动
            t = self.counter * 0.1
            msg.linear.x = self.linear_x * math.cos(t)
            msg.linear.y = self.linear_x * math.sin(t)
            msg.angular.z = self.angular_z
        
        self.publisher.publish(msg)
        self.counter += 1
        
        if self.counter % 50 == 0:
            self.get_logger().info(
                f'发布速度: linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}), '
                f'angular=({msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestVelocityPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

