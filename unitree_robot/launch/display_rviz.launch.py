import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 获取包路径
    pkg_path = get_package_share_directory('unitree_robot')
    model_path = os.path.join(pkg_path, 'urdf', 'ubot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'display.rviz')

    # 声明launch参数
    declare_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(model_path),
        description='Absolute path to robot urdf file'
    )

    declare_rviz_config_path = launch.actions.DeclareLaunchArgument(
        name='rviz_config',
        default_value=str(rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 处理xacro文件
    robot_description_content = launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('model')
    ])
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content,
        value_type=str
    )

    # 机器人状态发布器节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 关节状态发布器节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz2节点（带配置文件）
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', launch.substitutions.LaunchConfiguration('rviz_config')]
    )

    return launch.LaunchDescription([
        declare_model_path,
        declare_rviz_config_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])