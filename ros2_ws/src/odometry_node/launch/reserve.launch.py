from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("vehicle_id", default_value="default"),
        DeclareLaunchArgument("route_id", default_value=""),
        DeclareLaunchArgument("s0", default_value="0.0"),
        DeclareLaunchArgument("gnss_policy", default_value="disabled"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("initial_v_mps", default_value="-1.0"),
        DeclareLaunchArgument("pathgraph_directory", default_value=PathJoinSubstitution([
            FindPackageShare("odometry_python"), "Pathgraph",
        ])),
        DeclareLaunchArgument("model_config", default_value=PathJoinSubstitution([
            FindPackageShare("odometry_python"), "models",
        ])),
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
                # belt and suspenders for the launch path specifically). fix/runtime-validation
                # extended this to every string-like parameter and added the two this file was
                # missing (initial_v_mps, use_sim_time), each typed explicitly.
                **{name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in (
                    "vehicle_id", "route_id", "gnss_policy", "pathgraph_directory", "model_config",
                )},
                "s0": ParameterValue(LaunchConfiguration("s0"), value_type=float),
                "initial_v_mps": ParameterValue(LaunchConfiguration("initial_v_mps"), value_type=float),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
