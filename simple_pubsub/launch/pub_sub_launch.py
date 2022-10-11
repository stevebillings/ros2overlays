from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='simple_pubsub',
            executable='sbPublisher',
            output='screen'
        ),
        Node(
            package='simple_pubsub',
            executable='sbSubscriber',
            output='screen'
        )
    ])
