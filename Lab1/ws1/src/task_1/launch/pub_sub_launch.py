from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    
    talker1 = Node(
        package='task_1',
        executable='publisher',
        name='publisher1',

    )

    listener1 = Node(
        package='task_1',
        executable='subscriber',
        name='subscriber1',

    )
    
    launch_description = LaunchDescription([
        talker1,
        listener1,
    ])

    return launch_description
