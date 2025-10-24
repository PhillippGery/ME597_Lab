import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define package names
    task_4_pkg = 'task_4'
    turtlebot4_navigation_pkg = 'turtlebot4_navigation'

    # Get the absolute path to the map file
    # Using get_package_share_directory is more robust than a relative path
    map_path = os.path.join(
        get_package_share_directory(task_4_pkg),
        'maps',
        'classroom_map.yaml'
    )

    # 1. Launch the localization from turtlebot4_navigation
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
            'namespace': '/robot'
        }.items()
    )

    # 2. Launch RViz2 viewer from your task_4 package
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(task_4_pkg),
                'launch',
                'view_robot.launch.py'
            )
        ),
        launch_arguments={
            'map': map_path,
            'namespace': '/robot'
        }.items()
    )

    # 3. Run your auto_navigator node
    auto_navigator_node = Node(
        package=task_4_pkg,
        executable='auto_navigator', # The name of the entry point in your setup.py
        name='auto_navigator',
        output='screen',
        parameters=[{
            'map_yaml_path': map_path
        }],
        # Remap topics to be under the /robot namespace
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel'),
            ('/amcl_pose', '/robot/amcl_pose'),
            ('/global_plan', '/robot/global_plan'),
        ]
    )

    return LaunchDescription([
        # localization_launch,
        # rviz_launch,
        auto_navigator_node
    ])