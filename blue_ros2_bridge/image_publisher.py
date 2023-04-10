from threading import Thread

from blue_ros2_bridge.util import BaseNode

import cv2 as cv

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import zmq


class ImagePublisher(BaseNode):

    def __init__(self, node):
        super().__init__(node)

        self.server_host = self.get_parameter('server_host')
        self.server_port = self.get_parameter('server_port', default=10002)
        self.jpeg_quality = self.get_parameter('jpeg_quality', default=95)
        self.frame_rate = self.get_parameter('frame_rate', default=2)
        self.camera_frame = self.get_parameter('camera_frame',
                                               default='camera_link')

        self.image_pub = self.node.create_publisher(
            CompressedImage, 'image/compressed', 10)

        context = zmq.Context()
        self.sock = context.socket(zmq.SUB)
        self.sock.set_string(zmq.SUBSCRIBE, '')
        self.sock.connect(f'tcp://{self.server_host}:{self.server_port}')

        Thread(target=self.receive_images).start()

    def receive_images(self):
        while True:
            msg = self.sock.recv_multipart()
            self.publish_image(msg[0])

    def publish_image(self, data):
        msg = CompressedImage()
        msg.header.stamp = self.get_stamp() #self.stamp_from_time(image['time'])
        msg.header.frame_id = self.camera_frame
        msg.data = data
        self.image_pub.publish(msg)
        delay = 0 #image["time"] - self.get_time()
        self.log_info(f'Published image: time={msg.header.stamp} delay={delay}')


def main():
    rclpy.init()
    node = ImagePublisher(Node('image_publisher'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
