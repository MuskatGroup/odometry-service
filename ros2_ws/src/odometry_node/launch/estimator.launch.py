from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="Path to the bridge JSON config (see config/example.json)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="odometry_node", executable="estimator_node", name="estimator_node", output="screen",
             parameters=[{"config": LaunchConfiguration("config"),
                          "use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
