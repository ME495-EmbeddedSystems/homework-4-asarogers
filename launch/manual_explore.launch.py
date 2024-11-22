"""Launch franka demo, rviz2, and moveitapi."""
# Copyright 2024 Asa, Grayson, Tony, Sharwin
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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    """
    Run franka demo, rviz2, and moveitapi.

    If launch argument demo is set to False, the franka demo will not launch.
    """
    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     'demo',
            #     default_value='False',
            #     description='Demo Only? (True will launch Franka demo.launch.py)',
            # ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare('nubot'),
                        'launch',
                        'simulate.launch.xml',
                    ]
                ),
                # condition=IfCondition(
                #     EqualsSubstitution(LaunchConfiguration('demo'), 'True')
                # ),
            ),
            # Node(
            #     condition=IfCondition(
            #         EqualsSubstitution(LaunchConfiguration('demo'), 'False')
            #     ),
            #     package='rviz2',
            #     executable='rviz2'
            # ),
            # Node(
            #     package='moveitapi',
            #     executable='pick_node'
            # )
        ]
    )
