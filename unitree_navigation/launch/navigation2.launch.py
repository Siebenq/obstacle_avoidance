import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    nav2_dir = get_package_share_directory('unitree_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true'
    )
    map_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(nav2_dir, 'maps', 'room.yaml')
    )
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(nav2_dir, 'config', 'nav2_params.yaml')
    )
    # 点云话题（可配置）
    point_cloud_topic = launch.substitutions.LaunchConfiguration(
        'point_cloud_topic', default='/camera_person/points'
    )

    # ============================================================
    # pointcloud_to_laserscan 节点
    # 将深度相机的 PointCloud2 转换为虚拟的 LaserScan，供 AMCL 定位使用
    # ============================================================
    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', point_cloud_topic),     # 输入：深度相机点云（可配置）
            ('scan', '/scan'),                    # 输出：虚拟激光扫描
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            # 目标坐标系：将点云转换到 base_link 后再切片
            'target_frame': 'base_link',
            'transform_tolerance': 1.0,
            # ⚠ 高度切片范围（相对于 base_link 的Z轴）
            # base_link 在地面以上约 0.091m 处
            # min_height=0.1 → 地面以上约 0.19m（过滤地面噪声）
            # max_height=1.5 → 地面以上约 1.59m（原1.0太窄，会丢失很多墙面点，
            #   导致AMCL可用的扫描点太少，定位质量差）
            'min_height': 0.1,
            'max_height': 1.5,
            # 角度范围：覆盖深度相机的86°水平FOV
            'angle_min': -0.7854,       # -45° (弧度)
            'angle_max': 0.7854,        #  45° (弧度)
            'angle_increment': 0.00872, # ~0.5° 分辨率
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 10.0,
            'use_inf': True,
            'concurrency_level': 0,
        }],
        output='screen',
    )

    # ============================================================
    # Nav2 导航栈
    # ============================================================
    nav2_bringup_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
        }.items(),
    )

    # ============================================================
    # RViz 可视化
    # ============================================================
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        launch.actions.DeclareLaunchArgument(
            'map', default_value=os.path.join(nav2_dir, 'maps', 'room.yaml'),
            description='Full path to map yaml file'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_dir, 'config', 'nav2_params.yaml'),
            description='Full path to nav2 params yaml file'
        ),
        launch.actions.DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/camera_person/points',
            description='PointCloud2 topic for depth camera'
        ),

        # 1. 先启动 pointcloud_to_laserscan（AMCL 需要 /scan）
        pointcloud_to_laserscan_node,
        # 2. 启动 Nav2 导航栈
        nav2_bringup_node,
        # 3. 启动 RViz
        rviz_node,
    ])
