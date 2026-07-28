from glob import glob

from setuptools import setup


package_name = 'line_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juwnoo',
    maintainer_email='juwnoo@example.com',
    description='YOLO and OpenVINO based line detection for ROS 2 images.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_detect_node = line_detection.yolo_detect_node:main',
            'opencv_line_detect_node = line_detection.opencv_line_detect_node:main',
            'computer_vision_node = line_detection.computer_vision_node:main',
            'debug_node = line_detection.debug_node:main',
        ],
    },

)
