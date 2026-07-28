from setuptools import setup


setup(
    name='line_planner',
    version='0.1.0',
    packages=['line_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/line_planner']),
        ('share/line_planner', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juwnoo',
    maintainer_email='juwnoo@example.com',
    description='Builds a tracking centreline path from lane-detection output.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['line_planner_node = line_planner.line_planner_node:main']},
)
