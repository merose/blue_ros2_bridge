from setuptools import setup

package_name = 'blue_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='merose',
    maintainer_email='markrose@acm.org',
    description='Bridge package between Beaglebone Blue and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobility_commander = blue_ros2_bridge.mobility_commander:main',
            'robot_monitor = blue_ros2_bridge.robot_monitor:main',
            'image_publisher = blue_ros2_bridge.image_publisher:main',
            'image_writer = blue_ros2_bridge.image_writer:main',
        ],
    },
)
