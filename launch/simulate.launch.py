"""
@file simulate.launch.py
@brief
    Launch file for visualizing Hesai lidar URDF with a Gazebo simulation
@author
    Tobit Flatscher <tobit@robots.ox.ac.uk>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('hesai_description'),
                    'launch', 'visualize.launch.py'
                )
            ]
        ),
        launch_arguments={
            'simulation': 'true'
        }.items(),
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]
        )
    )
    gazebo_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='gazebo_spawner',
        arguments=['-entity', 'magnet', '-topic', 'robot_description'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(visualize_launch)
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawner_node)
    return ld
