from setuptools import setup
setup(name='vehicle_control',version='0.1.0',packages=['vehicle_control'],data_files=[('share/ament_index/resource_index/packages',['resource/vehicle_control']),('share/vehicle_control',['package.xml'])],entry_points={'console_scripts':['drive_node=vehicle_control.drive_node:main']})
