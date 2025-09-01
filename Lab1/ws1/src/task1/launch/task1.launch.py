from launch import LaunchDescription
from launch_ros.actions import Node

# run this launch file with the following command:
# ros2 launch chatterbot.launch.py

def generate_launch_description():

    
    talker1 = Node(
        package='task1',
        executable='publisher',
        name='publisher1',
        #remappings=[('chatter', 'chatter1')]
    )

    listener1 = Node(
        package='task1',
        executable='subscriber',
        name='subscriber1',
        #remappings=[('chatter', 'chatter1')]
    )
    
    launch_description = LaunchDescription([
        talker1,
        listener1,
    ])

    return launch_description