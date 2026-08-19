from glob import glob
from setuptools import find_packages, setup


package_name = 'kmu_track'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KMU Track Team',
    maintainer_email='team@example.com',
    description='Mission FSM and baseline perception for the KMU track.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = kmu_track.lane_node:main',
            'lane_preprocessor = kmu_track.lane_node:main',
            'lane_control = kmu_track.lane_control_node:main',
            'actuation_monitor = kmu_track.actuation_monitor_node:main',
            'steering_sweep = kmu_track.steering_sweep:main',
            'mission_manager = kmu_track.mission_manager_node:main',
            'scenario_player = kmu_track.scenario_player_node:main',
            'track_visualizer = kmu_track.visualizer_node:main',
            'traffic_light_detector = kmu_track.traffic_light_node:main',
            'traffic_motor_controller = kmu_track.traffic_motor_node:main',
            'traffic_preview_server = kmu_track.traffic_preview_server:main',
            'usb_camera_source = kmu_track.usb_camera_node:main',
            'video_source = kmu_track.video_source_node:main',
            'yolo_lane_detector = kmu_track.yolo_lane_node:main',
            'tcp_steering_sender = kmu_track.tcp_steering_sender:main',
        ],
    },
)
