import struct
from threading import Thread

from blue_ros2_bridge.util import BaseNode

from diff_drive_interfaces.msg import WheelTicks

import rclpy
from rclpy.node import Node

import zmq


class RobotMonitor(BaseNode):

    def __init__(self, node):
        super().__init__(node)

        self.server_host = self.get_parameter('server_host')
        self.server_port = self.get_parameter('server_port', default=10001)
        self.frame_id = self.get_parameter('frame_id', default='base_link')

        context = zmq.Context()
        self.sock = context.socket(zmq.SUB)
        self.sock.set_string(zmq.SUBSCRIBE, '')
        self.sock.connect(f'tcp://{self.server_host}:{self.server_port}')

        self.tick_pub = self.node.create_publisher(WheelTicks, 'wheel_ticks',
                                                   10)
        Thread(target=self.monitor).start()

    def monitor(self):
        while True:
            robot_msg = self.sock.recv_json();
            msg = WheelTicks()
            msg.header.stamp = self.get_stamp()
            msg.header.frame_id = self.frame_id
            msg.left = robot_msg['left']
            msg.right = robot_msg['right']
            self.tick_pub.publish(msg)
        

def main():
    rclpy.init()
    node = RobotMonitor(Node('robot_monitor'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
