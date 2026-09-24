from setuptools import find_packages, setup

package_name = "odometry_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/estimator.launch.py"]),
        ("share/" + package_name + "/config", ["config/example.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["estimator_node = odometry_node.estimator_node:main"]},
)
