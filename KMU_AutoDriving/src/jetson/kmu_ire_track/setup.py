from glob import glob
from setuptools import find_packages, setup


package_name = 'kmu_ire_track'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KMU Track Team',
    maintainer_email='team@example.com',
    description=(
        'IRE variant of KMU segmentation lane planning with center-marking '
        'priority.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ire_camera_source = '
            'kmu_ire_track.ire_camera_source_node:main',
            'ire_follow_view = '
            'kmu_ire_track.ire_follow_view_node:main',
            'ire_yolo_seg_lane_detector = '
            'kmu_ire_track.ire_yolo_seg_lane_node:main',
        ],
    },
)
