# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

import os
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class QuotesPublisher(Node):
    def __init__(self):
        super().__init__('quotes_publisher')
        self.publisher_ = self.create_publisher(String, 'quote_topic', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        pkg_share = get_package_share_directory('saying_sender')
        path = os.path.join(pkg_share, 'words.txt')

        try:
            with open(path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                valid_lines = [line.strip() for line in lines if line.strip()]
                msg.data = random.choice(valid_lines) if valid_lines else 'File is empty'
        except FileNotFoundError:
            msg.data = 'File not found'
            self.get_logger().error(f'File not found at: {path}')

        self.publisher_.publish(msg)
        self.get_logger().info('Publish: "%s"' % msg.data)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = QuotesPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
