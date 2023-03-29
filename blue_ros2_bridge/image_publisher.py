from blue_ros2_bridge.util import BaseNode

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import zmq


class ImagePublisher(BaseNode):

    def __init__(self, node):
        super().__init__(node)

        self.image_host = self.get_parameter('image_host')
        self.image_port = self.get_parameter('image_port', default=10002)
        self.jpeg_quality = self.get_parameter('jpeg_quality', default=95)
        self.frame_rate = self.get_parameter('frame_rate', default=2)
        self.camera_frame = self.get_parameter('camera_frame',
                                               default='camera_link')

        self.image_pub = self.node.create_publisher(
            CompressedImage, 'camera/image_compressed', 10)

        context = zmq.Context()
        self.sock = context.socket(zmq.REQ)
        self.sock.connect(f'tcp://{self.image_host}:{self.image_port}')

        self.node.create_timer(1/self.frame_rate, self.publish_image)

    def publish_image(self):
        self.log_info('Retrieving image')
        self.sock.send_json({
            'time': self.get_time(),
            'quality': self.jpeg_quality
        })
        image = self.sock.recv_pyobj()

        msg = CompressedImage()
        msg.header.stamp = self.stamp_from_time(image['time'])
        msg.header.frame_id = self.camera_frame
        msg.format = 'jpeg'
        msg.data = image['data']
        self.image_pub.publish(msg)


def main():
    rclpy.init()
    node = ImagePublisher(Node('image_publisher'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()