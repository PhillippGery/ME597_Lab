from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )

    auto_navigator_node = Node(
        package='task_4',
        executable='auto_navigator',  
        name='auto_navigator',
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )

    return LaunchDescription([
        namespace_arg,
        auto_navigator_node,
    ])