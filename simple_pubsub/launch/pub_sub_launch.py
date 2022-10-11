from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='simple_pubsub',
            executable='publisher',
            output='screen'
        ),
        Node(
            package='simple_pubsub',
            executable='subscriber',
            output='screen'
        )
    ])
