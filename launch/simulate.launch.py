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
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ]
        ),
        launch_arguments={
            'gz_args': " -r -v 3 fuel.sdf"
        }.items()
    )
    gazebo_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "hesai_qt64",
            "-allow_renaming",
            "true",
        ]
    )
    gazebo_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('hesai_description'),
                    'launch', 'gz_bridge.launch.py'
                )
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(visualize_launch)
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawner_node)
    ld.add_action(gazebo_bridge_launch)
    return ld
