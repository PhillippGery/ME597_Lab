from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='task2',
            executable='talker', # This is the alias from setup.py
            name='publisher',
            output='screen'
        ),
        Node(
            package='task2',
            executable='service', # This is the alias from setup.py
            name='service',
            output='screen'
        ),
    ])