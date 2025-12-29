# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='saying_sender',
            executable='quotes_publisher',
            name='publisher_node'
        ),
        launch_ros.actions.Node(
            package='saying_sender',
            executable='quotes_subscriber',
            name='subscriber_node'
        ),
    ])
