import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'parking_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('*.yaml') + glob('*.pgm')),
        (os.path.join('share', package_name, 'data'), glob('*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sy',
    maintainer_email='sy@todo.todo',
    description='ROS 2 Autonomous Driving and Parking Mission Package for 1/10 RC Car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_waypoint_follower = parking_mission.mission_waypoint_follower:main',
            'map_server_node = parking_mission.map_server_node:main',
            'sim_vehicle_node = parking_mission.sim_vehicle_node:main',
        ],
    },
)
