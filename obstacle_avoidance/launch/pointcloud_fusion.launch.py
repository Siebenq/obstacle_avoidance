#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('obstacle_avoidance')
    
    # 获取默认参数文件路径
    default_params_file = os.path.join(pkg_share, 'config', 'fusion_params.yaml')
    
    # 声明启动参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='参数文件的完整路径'
    )
    
    # 创建节点
    pointcloud_fusion_node = Node(
        package='obstacle_avoidance',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        params_file_arg,
        pointcloud_fusion_node
    ])

