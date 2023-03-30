import struct

from blue_ros2_bridge.util import BaseNode

from diff_drive_interfaces.msg import WheelTicks

import rclpy
from rclpy.node import Node

import zmq


class MobilityCommander(BaseNode):

    def __init__(self, node):
        super().__init__(node)

        self.server_host = self.get_parameter('server_host')
        self.server_port = self.get_parameter('server_port', default=10000)

        context = zmq.Context()
        self.sock = context.socket(zmq.PUB)
        self.sock.connect(f'tcp://{self.server_host}:{self.server_port}')

        self.node.create_subscription(WheelTicks, 'wheel_desired_rates',
                                      self.on_rates, 10)

    def on_rates(self, msg):
        msg = struct.pack('<ii', msg.left, msg.right)
        self.sock.send(msg)


def main():
    rclpy.init()
    node = MobilityCommander(Node('mobility_commander'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
