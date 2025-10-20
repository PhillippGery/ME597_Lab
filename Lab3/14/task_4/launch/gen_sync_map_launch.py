import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument( 'namespace',
        default_value='/robot',
        description='Namespace for the robot'
    )

    # Path to the turtlebot4_navigation slam.launch.py file
    slam_launch_path = os.path.join(
        get_package_share_directory('turtlebot4_navigation'),
        'launch',
        'slam.launch.py'
    )

    # Path to the turtlebot4_viz view_robot.launch.py file
    viz_launch_path = os.path.join(
        get_package_share_directory('turtlebot4_viz'),
        'launch',
        'view_robot.launch.py'
    )

    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    # Include the RViz launch file
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(viz_launch_path),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    return LaunchDescription([
        namespace_arg,
        slam_launch,
        viz_launch
    ])





