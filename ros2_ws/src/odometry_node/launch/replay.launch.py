from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("source", default_value="/artifacts/demo/events.jsonl"),
            DeclareLaunchArgument("profile", default_value="/workspace/contracts/profiles/events.yaml"),
            DeclareLaunchArgument("run_id", default_value="ros-demo"),
            Node(
                package="odometry_node",
                executable="estimator",
                parameters=[
                    {"input_mode": "batch", "use_sim_time": True, "run_id": LaunchConfiguration("run_id")}
                ],
            ),
            Node(
                package="odometry_node",
                executable="replay",
                parameters=[
                    {"source": LaunchConfiguration("source"), "profile": LaunchConfiguration("profile")}
                ],
            ),
        ]
    )
