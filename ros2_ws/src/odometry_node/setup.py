from glob import glob

from setuptools import setup

setup(
    name="odometry_node",
    version="0.1.0",
    packages=["odometry_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/odometry_node"]),
        ("share/odometry_node", ["package.xml"]),
        ("share/odometry_node/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "reserve_odometry_node = odometry_node.runtime:main",
        ]
    },
)
