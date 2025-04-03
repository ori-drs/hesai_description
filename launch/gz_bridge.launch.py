"""
@file gz_bridge.launch.py
@brief
    Launch file for Gazebo Ignition bridge
@author
    Tobit Flatscher <tobit@robots.ox.ac.uk>
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [
            get_package_share_directory('hesai_description'),
            'config', 'gz_bridge.yaml'
        ]
    )
    gz_bridge = Node(
        name='hesai_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        output='screen',
        parameters= [
            {
                'use_sim_time': True,
                'config_file': default_config_file
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(gz_bridge)
    return ld