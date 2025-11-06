

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define the package names
    turtlebot3_gazebo_pkg = 'turtlebot3_gazebo'
    sim_utils_pkg = 'sim_utils'
    task_6_pkg = 'task_6'
    turtlebot4_navigation_pkg = 'turtlebot4_navigation'

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(turtlebot3_gazebo_pkg),
                'launch',
                'task_6.launch.py'  
            )
        )
    )

    map_path = os.path.join(
        get_package_share_directory(task_6_pkg),
        'maps',
        'sync_classroom_map.yaml'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(turtlebot4_navigation_pkg),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'map': map_path,
            'namespace': '',         
            'x_pose': '-5.4',
            'y_pose': '-6.18',
            'z_pose': '0.0',
            'yaw': '1.0',
        }.items()
    )

    # red ball comntroller in external terminal
    controller_node = Node(
        package=sim_utils_pkg,
        executable='red_ball_controller',
        name='red_ball_controller',
        output='screen',
        prefix='xterm -e'
    )

    red_ball_tracker_node = Node(
        package='task_6',
        executable='red_ball_tracker',  
        name='red_ball_tracker',
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        controller_node,
        localization_launch,
        red_ball_tracker_node
        
    ])


