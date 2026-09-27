from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                # vehicle_id in particular ("30618") reads as a number; force str so a numeric-
                # looking launch argument is never mistaken for an int override (organizer audit,
                # 2026-09-27 — the node itself now also tolerates this via dynamic_typing, this is
                # belt and suspenders for the launch path specifically).
                "vehicle_id": ParameterValue(LaunchConfiguration("vehicle_id"), value_type=str),
                "route_id": ParameterValue(LaunchConfiguration("route_id"), value_type=str),
                "s0": LaunchConfiguration("s0"),
                "gnss_policy": LaunchConfiguration("gnss_policy"),
                "pathgraph_directory": LaunchConfiguration("pathgraph_directory"),
                "model_config": LaunchConfiguration("model_config"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
