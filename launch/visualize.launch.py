"""
@file visualize.launch.py
@brief
    Launch file for visualizing Hesai lidar URDF
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
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('hesai_description'),
                    'launch', 'description.launch.py'
                )
            ]
        ),
        launch_arguments={
            'simulation': 'true'
        }.items(),
    )

    rviz_file = os.path.join(
        get_package_share_directory('hesai_description'),
        'rviz',
        'model.rviz'
    )
    rviz_node = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file]
    )

    ld = LaunchDescription()
    ld.add_action(description_launch)
    ld.add_action(rviz_node)
    return ld