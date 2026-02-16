# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'graph_observer_interval',
            default_value='5.0',
            description='Interval in seconds between graph observation cycles',
        ),
        Node(
            package='muto_daemon',
            executable='muto_daemon',
            name='muto_daemon',
            parameters=[{
                'graph_observer_interval':
                    LaunchConfiguration('graph_observer_interval'),
            }],
            output='screen',
        ),
    ])
