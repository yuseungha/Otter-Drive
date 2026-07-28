from glob import glob

from setuptools import setup


package_name = 'camera_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juwnoo',
    maintainer_email='juwnoo@example.com',
    description='Publishes frames from a recorded driving video as ROS 2 images.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'video_publisher_node = camera_publisher.video_publisher_node:main',
        ],
    },
)
