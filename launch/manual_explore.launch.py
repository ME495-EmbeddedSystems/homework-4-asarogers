"""
Launch File for NuBot Navigation in Simulation.

This launch file configures and initializes a simulated NuBot robot within the Gazebo simulator.
It sets up the robot's URDF model, RViz, and localization, and Nav2 stack

Authors:
   Asa Rogers Date: 2024-11-24
"""
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    """Define the launch file."""
    # Locate package files
    pkg_share = FindPackageShare(package='nubot_nav').find('nubot_nav')
    default_model_path = os.path.join(pkg_share, 'urdf/nubot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/nubot_urdf.rviz')
    world_config = os.path.join(pkg_share, 'worlds/nubot_world.sdf')
    nav2_params_config = os.path.join(pkg_share, 'config/nav2_params.yaml')

    robot_desc = xacro.process_file(default_model_path).toxml()
    robot_description = {'robot_description': robot_desc}

    # Define nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'nubot',
            '-topic', '/robot_description',
            '-x', '15', '-z', '0.3'],
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
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch description
    return launch.LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path,
                              description='Path to robot URDF file'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path,
                              description='Path to RViz config file'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Use sim time if true'),
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
                '/model/nubot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/world/nubot_world/model/nubot/joint_state@sensor_msgs/msg/JointState['
                'gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            remappings=[
                ('/model/nubot/odometry', '/odom'),
                ('/model/nubot/cmd_vel', '/cmd_vel'),
                ('/model/nubot/tf', '/tf'),
                ('/world/nubot_world/model/nubot/joint_state', '/joint_states')
            ]
        ),
        Node(
            package='nubot_nav',
            executable='explore_node',
            name='explore_node',
            output='screen'
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
                'use_sim_time': 'true'}.items()
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
