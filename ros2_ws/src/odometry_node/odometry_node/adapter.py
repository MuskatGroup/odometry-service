"""Configured ROS types -> the same normalizer used for files."""

from pathlib import Path

import rclpy
import yaml
from odometry_core import ControlSample
from odometry_io import Normalizer, load_profile
from odometry_msgs.msg import ControlSample as ControlMsg
from odometry_msgs.msg import WheelSample as WheelMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from .conversion import event_message


class SourceAdapter(Node):
    def __init__(self):
        super().__init__("source_adapter")
        self.declare_parameter("config", "")
        path = Path(self.get_parameter("config").value)
        configuration = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.control = self.create_publisher(ControlMsg, "/tram/control", 10)
        self.wheels = self.create_publisher(WheelMsg, "/tram/wheel_speed", 10)
        self.subscriptions_owned = []
        for spec in configuration["topics"]:
            normalizer = Normalizer(load_profile(path.parent / spec["profile"]))

            def callback(msg, n=normalizer, s=spec):
                # Optional wrapper gives a constant event kind without mutating ROS data.
                record = {"message": msg, "kind": s["kind"]} if "kind" in s else msg
                before = n.diagnostics["INVALID_RECORD"]
                for event in n.normalize(record):
                    publisher = self.control if isinstance(event, ControlSample) else self.wheels
                    publisher.publish(event_message(event))
                if n.diagnostics["INVALID_RECORD"] > before:
                    self.get_logger().warning(f"Input rejected: {n.last_error}", throttle_duration_sec=1)

            qos = qos_profile_sensor_data if spec.get("qos", "reliable") == "sensor" else 10
            self.subscriptions_owned.append(
                self.create_subscription(get_message(spec["type"]), spec["name"], callback, qos)
            )


def main():
    rclpy.init()
    node = SourceAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
