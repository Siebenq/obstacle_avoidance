#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('obstacle_avoidance'),
        'config',
        'icp_calibration_params.yaml'
    )
    
    # ICP标定节点
    icp_calibration_node = Node(
        package='obstacle_avoidance',
        executable='icp_calibration_node',
        name='icp_calibration_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )
    
    return LaunchDescription([
        icp_calibration_node
    ])

