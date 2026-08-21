from glob import glob
import os

from setuptools import find_packages, setup


package_name = "lidar_cone_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "LICENSE", "OPEN_SOURCE_REFERENCES.md"],
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zeus",
    maintainer_email="zeus@example.com",
    description="Fail-closed 2D LiDAR cone planner and Ackermann controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cone_line_planner = lidar_cone_planner.cone_line_planner:main",
            "cone_cv_viewer = lidar_cone_planner.cone_cv_viewer:main",
            "cone_lidar_static_tf = lidar_cone_planner.lidar_static_tf:main",
            "cone_pure_pursuit = lidar_cone_planner.cone_pure_pursuit:main",
            "synthetic_cone_world = "
            "lidar_cone_planner.synthetic_cone_world:main",
            "synthetic_validation = "
            "lidar_cone_planner.synthetic_validation:main",
        ],
    },
)
