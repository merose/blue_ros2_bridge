import json

from blue_ros2_bridge.util import BaseNode

from std_msgs.msg import Float64

import rclpy
from rclpy.node import Node

import zmq


class ServoCommander(BaseNode):

    def __init__(self, node):
        super().__init__(node)

        servo_count = self.get_parameter('servos_count', default=8)
        server_host = self.get_parameter('server_host', default='localhost')
        server_port = self.get_parameter('server_port', default=10003)
        update_interval = self.get_parameter('update_interval', default=0.1)

        self.servo_values = [None for i in range(servo_count+1)]
        

        context = zmq.Context()
        self.sock = context.socket(zmq.PUB)
        self.log_info(f'Connecting to: tcp://{server_host}:{server_port}')
        self.sock.connect(f'tcp://{server_host}:{server_port}')

        for i in range(1, servo_count+1):
            self.node.create_subscription(Float64, f'servo_{i}',
                                          self.get_callback(i), 10)

        self.node.create_timer(update_interval, self.command_servos)

    def get_callback(self, i):
        return lambda msg: self.on_servo(msg, i)

    def on_servo(self, msg, index):
        value = max(0.0, min(1.0, msg.data))
        self.servo_values[index] = value
        self.log_info(f'Setting servo {index} to {value}')
        self.log_info(f'Servo values: {self.servo_values}')

    def command_servos(self):
        command = {f'servo{i}': self.servo_values[i]
                   for i in range(len(self.servo_values))
                   if self.servo_values[i] is not None}
        self.log_info(f'Sending servo command: {command}')
        self.sock.send_json(command)
        

def main():
    rclpy.init()
    node = ServoCommander(Node('servo_commander'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
