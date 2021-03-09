from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_controller',
            namespace='turtlesim1',
            executable='controller',
            name='control',
            parameters=[
                {'forward': 'x',
                 'backwards': '<Down>',
                 'left': '<Left>',
                 'right': '<Right>'
                }
            ]
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
    ])
