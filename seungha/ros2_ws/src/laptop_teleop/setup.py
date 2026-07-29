from glob import glob
import os

from setuptools import setup


package_name = "laptop_teleop"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "web"),
            glob("web/*"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="sandi",
    maintainer_email="sandi@example.com",
    description="Fail-safe laptop browser teleoperation for the RC car",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_teleop = laptop_teleop.web_teleop:main",
        ],
    },
)
