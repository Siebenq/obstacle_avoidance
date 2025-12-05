#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('obstacle_avoidance')
    
    # 获取默认参数文件路径
    fusion_params_file = os.path.join(pkg_share, 'config', 'fusion_params.yaml')
    obstacle_params_file = os.path.join(pkg_share, 'config', 'obstacle_detection_params.yaml')
    mpc_params_file = os.path.join(pkg_share, 'config', 'mpc_controller_params.yaml')
    
    # 声明启动参数
    fusion_params_arg = DeclareLaunchArgument(
        'fusion_params_file',
        default_value=fusion_params_file,
        description='点云融合参数文件的完整路径'
    )
    
    obstacle_params_arg = DeclareLaunchArgument(
        'obstacle_params_file',
        default_value=obstacle_params_file,
        description='障碍物检测参数文件的完整路径'
    )
    

    mpc_params_arg = DeclareLaunchArgument(
        'mpc_params_file',
        default_value=mpc_params_file,
        description='MPC控制器参数文件的完整路径'
    )
    
    # 创建点云融合节点
    pointcloud_fusion_node = Node(
        package='obstacle_avoidance',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        parameters=[LaunchConfiguration('fusion_params_file')],
        emulate_tty=True,
    )
    
    # 创建障碍物检测节点
    obstacle_detection_node = Node(
        package='obstacle_avoidance',
        executable='obstacle_detection_node',
        name='obstacle_detection_node',
        output='screen',
        parameters=[LaunchConfiguration('obstacle_params_file')],
        emulate_tty=True,
    )
    
    
    # 创建MPC控制器节点
    mpc_controller_node = Node(
        package='bstacle_avoidance',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('mpc_params_file')],
        emulate_tty=True,
    )


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
        fusion_params_arg,
        obstacle_params_arg,
        mpc_params_arg,
        icp_calibration_node,
        TimerAction(period=5.0,actions=[pointcloud_fusion_node]),
        TimerAction(period=2.0, actions=[obstacle_detection_node]),
        TimerAction(period=4.0, actions=[mpc_controller_node])
    ])

