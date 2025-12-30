# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

import os
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QuotesPublisher(Node):
    def __init__(self):
        super().__init__('quotes_publisher')
        self.publisher_ = self.create_publisher(String, 'quote_topic', 10)
        self.send_quote()

    def send_quote(self):
        msg = String()
        path = os.path.expanduser('~/ros2_ws/src/saying_sender/words.txt')

        try:
            with open(path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                msg.data = random.choice(lines).strip() if lines else 'File is empty'
        except FileNotFoundError:
            msg.data = 'File not found'

        self.publisher_.publish(msg)
        self.get_logger().info('Publish: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = QuotesPublisher()

    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
