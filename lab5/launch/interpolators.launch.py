import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='lab5',
            executable='interpolator_point',
            name='interpolator_point'),
        Node(
            package='lab5',
            executable='inv_kin',
            name='inv_kin'),
        Node(
            package='lab5',
            executable='no_kdl',
            name='no_kdl'),
        Node(
            package='lab5',
            executable='interpolator',
            name='interpolator'),
        Node(
            package='lab5',
            executable='marker_array_drawer',
            name='marker_array_drawer')
    ])
