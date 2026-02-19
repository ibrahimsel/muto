from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='/demo',
            output='screen',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            namespace='/demo',
            output='screen',
        ),
    ])
