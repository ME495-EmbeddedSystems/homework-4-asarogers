import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
import os
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Locate package files
    pkg_share = FindPackageShare(package='nubot_nav').find('nubot_nav')
    default_model_path = os.path.join(pkg_share, 'urdf/nubot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/nubot_urdf.rviz')
    world_config = os.path.join(pkg_share, 'worlds/nubot_simple.sdf')
    nav2_params_config = os.path.join(pkg_share, 'config/nav2_params.yaml')

    robot_desc = xacro.process_file(default_model_path).toxml()
    robot_description = {'robot_description': robot_desc}

    # # Validate paths
    # assert os.path.exists(default_model_path), f'URDF file not found: {default_model_path}'
    # assert os.path.exists(default_rviz_config_path), f'RViz config file not found: {default_rviz_config_path}'
    # assert os.path.exists(world_config), f'World file not found: {world_config}'

    # Define nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     arguments=[default_model_path]
    # )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'nubot',
            '-topic', '/robot_description',
            '-x', '15', '-z', '0.3'
            ],
        output='screen'
    )

    # Gazebo simulator
    gz_sim = launch.actions.ExecuteProcess(
        cmd=[
            'gz', 'sim', '--verbose', '-r', world_config, 
        ],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch description
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        robot_localization_node,
        rviz_node,
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/nubot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/nubot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/nubot/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/world/nubot_world/model/nubot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            remappings=[
                ('/model/nubot/odometry', '/odom'),
                ('/model/nubot/cmd_vel', '/cmd_vel'),
                ('/model/nubot/tf', '/tf'),
                ('/world/nubot_world/model/nubot/joint_state', '/joint_states')
            ]
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                ]),
            launch_arguments={
                'params_file': nav2_params_config,
                'use_sim_time': 'true'
                }.items()
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py',
                ]
            ),
        ),

    ])
