from glob import glob
import os
from setuptools import setup

package_name = 'rc_car_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RC car keyboard teleoperation and Arduino serial bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = rc_car_teleop.keyboard_teleop:main',
            'serial_bridge = rc_car_teleop.serial_bridge:main',
            'tcp_steering_receiver = '
            'rc_car_teleop.tcp_steering_receiver:main',
            'ackermann_to_drive_cmd = '
            'rc_car_teleop.ackermann_to_drive_cmd:main',
        ],
    },
)
