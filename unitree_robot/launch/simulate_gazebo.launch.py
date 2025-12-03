import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    urdf_path=get_package_share_directory('unitree_robot')
    model_path=urdf_path+'/urdf/ubot.urdf.xacro'
    # model2_path=urdf_path+'/urdf2/ubot2.urdf.xacro'
    world_path=urdf_path+'/world/obstacle.world'

    declare_path=launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(model_path),
        description='the path of model'
    )
    
    # declare_path2=launch.actions.DeclareLaunchArgument(
    #     name='model2',
    #     default_value=str(model2_path),
    #     description='the path of model2'
    # )

    substitutions_result=launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('model')
    ])
    robot_description=launch_ros.parameter_descriptions.ParameterValue(
        substitutions_result,
        value_type=str
    )

    # substitutions_result2=launch.substitutions.Command([
    #     'xacro ',
    #     launch.substitutions.LaunchConfiguration('model2')
    # ])
    # robot_description2=launch_ros.parameter_descriptions.ParameterValue(
    #     substitutions_result2,
    #     value_type=str
    # )

    robot_state_publisher_node=launch_ros.actions.Node(
     package='robot_state_publisher',
     executable='robot_state_publisher',
     parameters=[{'robot_description':robot_description}],
     namespace='r1',  # 为第1个机器人指定命名空间
    )

    # robot_state_publisher_node2=launch_ros.actions.Node(
    #  package='robot_state_publisher',
    #  executable='robot_state_publisher',
    #  parameters=[{'robot_description':robot_description2}],
    #  namespace='r2',  # 为第2个机器人指定命名空间
    # )

    launch_gazebo=launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py']),
        launch_arguments=[('world',world_path),('verbose','true')]
    )

    spawn_entity_node=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/r1/robot_description', '-entity', 'model',
                    '-x', '-1.0', '-y', '0.0', '-z', '0.0', 
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0']
    )

    # spawn_entity_node2=launch_ros.actions.Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-topic', '/r2/robot_description', '-entity', 'model2',
    #                 '-x', '0.0', '-y', '0.0', '-z', '0.0', 
    #                 '-R', '0.0', '-P', '0.0', '-Y', '0.0']
    # )

    return launch.LaunchDescription([

        launch_gazebo,

        declare_path,
        robot_state_publisher_node,
        spawn_entity_node,

        # declare_path2,
        # robot_state_publisher_node2,
        # spawn_entity_node2,

    ])
