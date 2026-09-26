from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("vehicle_id", default_value="default"),
        DeclareLaunchArgument("route_id", default_value=""),
        DeclareLaunchArgument("s0", default_value="0.0"),
        DeclareLaunchArgument("gnss_policy", default_value="disabled"),
        DeclareLaunchArgument("pathgraph_directory"),
        DeclareLaunchArgument("model_config"),
    ]
    node = Node(
        package="odometry_node",
        executable="reserve_odometry_node",
        name="reserve_odometry",
        output="screen",
        parameters=[
            {
                "vehicle_id": LaunchConfiguration("vehicle_id"),
                "route_id": LaunchConfiguration("route_id"),
                "s0": LaunchConfiguration("s0"),
                "gnss_policy": LaunchConfiguration("gnss_policy"),
                "pathgraph_directory": LaunchConfiguration("pathgraph_directory"),
                "model_config": LaunchConfiguration("model_config"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
