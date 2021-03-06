from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_controller',
            namespace='lab1',
            executable='controller',
            name='control',
            parameters=[
                {'forward': 'w',
                 'backwards': 's',
                 'left': 'a',
                 'right': 'd'
                }
            ]
        ),
        Node(
            package='turtlesim',
            namespace='lab1',
            executable='turtlesim_node',
            name='sim'
        ),
    ])
