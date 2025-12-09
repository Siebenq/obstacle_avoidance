#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('obstacle_avoidance')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'apf_controller_params.yaml')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # APF控制器节点
    apf_controller_node = Node(
        package='obstacle_avoidance',
        executable='apf_controller_node',
        name='apf_controller_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        apf_controller_node,
    ])

