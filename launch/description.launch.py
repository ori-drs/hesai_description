"""
@file description.launch.py
@brief
    Launch file for loading Hesai lidar description
@author
    Tobit Flatscher <tobit@robots.ox.ac.uk>
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    simulation_parameter_name = 'simulation'
    simulation = LaunchConfiguration(simulation_parameter_name)
    xacro_file_parameter_name = 'xacro_file'
    xacro_file = LaunchConfiguration(xacro_file_parameter_name)

    simulation_parameter_arg = DeclareLaunchArgument(
        simulation_parameter_name,
        default_value='true',
        description='Simulated or real hardware interface'
    )
    default_xacro_file = PathJoinSubstitution(
        [
            get_package_share_directory('hesai_description'),
            'urdf', 'hesai_xt32_standalone.urdf.xacro'
        ]
    )
    xacro_file_parameter_arg = DeclareLaunchArgument(
        xacro_file_parameter_name,
        default_value=default_xacro_file,
        description='Xacro URDF description to be used'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([xacro_file]), ' ',
            'simulation:=', simulation, ' ',
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ld = LaunchDescription()
    ld.add_action(simulation_parameter_arg)
    ld.add_action(xacro_file_parameter_arg)
    ld.add_action(robot_state_publisher_node)
    return ld