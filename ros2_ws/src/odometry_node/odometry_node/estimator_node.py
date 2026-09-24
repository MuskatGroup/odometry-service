"""Thin rclpy shell around odometry_io.ros_bridge.RosBridge. UNTESTED against a real ROS 2 install:
the logic lives in the ROS-free bridge (unit-tested); this file only wires topics, QoS and timers.

Install the core first:  pip install <repo root>   (provides `odometry_io`), then colcon build.
"""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from odometry_io.ros_bridge import RosBridge

try:
    from odometry_msgs.msg import LongitudinalEstimate
except ImportError:  # message package not built: fall back to a JSON string topic
    LongitudinalEstimate = None

MODES = {"INITIALIZING": 0, "FUSED": 1, "DEGRADED": 2, "MODEL_ONLY": 3, "INVALID": 4}


def qos(name):
    reliability = ReliabilityPolicy.BEST_EFFORT if name == "best_effort" else ReliabilityPolicy.RELIABLE
    return QoSProfile(reliability=reliability, history=HistoryPolicy.KEEP_LAST, depth=5)


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")
        path = self.declare_parameter("config", "").value
        if not path:
            raise RuntimeError("Parameter 'config' (path to bridge JSON) is required")
        with open(path, encoding="utf-8-sig") as stream:
            config = json.load(stream)
        self.bridge = RosBridge(config)
        self.subscriptions_ = []
        for item in config["inputs"]:
            msg_type = get_message(item["type"])
            self.subscriptions_.append(self.create_subscription(
                msg_type, item["topic"],
                lambda msg, topic=item["topic"]: self.on_message(topic, msg), qos(item.get("qos", "reliable"))))
        out = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", out)
        if LongitudinalEstimate is not None:
            self.est_pub = self.create_publisher(LongitudinalEstimate, "/odometry/estimate", out)
        else:
            self.get_logger().warn("odometry_msgs not found: publishing JSON on /odometry/estimate_json")
            self.est_pub = self.create_publisher(String, "/odometry/estimate_json", out)
        self.timer = self.create_timer(1.0 / float(self.bridge.output["rate_hz"]), self.on_timer)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def on_message(self, topic, msg):
        try:
            self.bridge.on_message(topic, message_to_ordereddict(msg), self.now_ns())
        except Exception as exc:  # bad input must not kill the estimator; the frame flags stay honest
            self.get_logger().error(f"{topic}: {exc}", throttle_duration_sec=5.0)

    def on_timer(self):
        try:
            frame = self.bridge.tick(self.now_ns())
        except Exception as exc:
            self.get_logger().error(f"tick failed: {exc}", throttle_duration_sec=5.0)
            return
        if frame is None:
            return
        stamp_ns = int(frame["stamp_ns"])
        stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
        self.publish_estimate(frame, stamp)
        odom = self.bridge.odometry(frame)
        if odom is not None:  # Odometry has no valid flag: stop publishing when the estimate is invalid
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id = stamp, odom["frame_id"]
            msg.child_frame_id = odom["child_frame_id"]
            msg.pose.pose.position.x = odom["x"]
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = odom["vx"]
            big = 1e6  # unobserved degrees of freedom: large finite variance, never zero
            msg.pose.covariance = [big if i % 7 == 0 else 0.0 for i in range(36)]
            msg.twist.covariance = [big if i % 7 == 0 else 0.0 for i in range(36)]
            msg.pose.covariance[0], msg.twist.covariance[0] = odom["var_x"], odom["var_vx"]
            self.odom_pub.publish(msg)

    def publish_estimate(self, frame, stamp):
        if LongitudinalEstimate is None:
            self.est_pub.publish(String(data=json.dumps(frame, allow_nan=False)))
            return
        msg = LongitudinalEstimate()
        msg.header.stamp = stamp
        msg.header.frame_id = self.bridge.output["frame_id"]
        msg.seq = frame["seq"]
        msg.has_estimate = frame["s_m"] is not None
        msg.valid = bool(frame["valid"])
        msg.mode = MODES[frame["mode"]]
        if msg.has_estimate:
            msg.s_m, msg.v_mps = float(frame["s_m"]), float(frame["v_mps"])
            msg.a_mps2, msg.disturbance_mps2 = float(frame["a_mps2"]), float(frame["disturbance_mps2"])
            msg.covariance = [float(v) for v in frame["covariance_4x4"]]
            msg.sigma_s_m, msg.sigma_v_mps = float(frame["sigma_s_m"]), float(frame["sigma_v_mps"])
        msg.reason_codes = list(frame["reason_codes"])
        msg.has_control_age = frame["control_age_s"] is not None
        msg.control_age_s = float(frame["control_age_s"] or 0.0)
        msg.has_wheel_age = frame["wheel_age_s"] is not None
        msg.wheel_age_s = float(frame["wheel_age_s"] or 0.0)
        msg.model_only_duration_s = float(frame["model_only_duration_s"])
        msg.accepted_wheel_count = frame["accepted_wheel_count"]
        msg.rejected_wheel_count = frame["rejected_wheel_count"]
        msg.model_version = frame["model_version"]
        self.est_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
