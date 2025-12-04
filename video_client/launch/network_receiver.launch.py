#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('video_client'),
        'config',
        'network_params.yaml'
    )
    
    # 网络接收节点
    network_receiver_node = Node(
        package='video_client',
        executable='network_receiver_node',
        name='network_receiver_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )
    
    return LaunchDescription([
        network_receiver_node
    ])

