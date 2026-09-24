import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from odometry_core import EstimatorConfig, InitialState, OdometryEstimator
from odometry_msgs.msg import ControlSample, InputBatch, LongitudinalEstimate, WheelSample
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from .conversion import domain_event, estimate_message, ns


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator")
        self.declare_parameter("input_mode", "batch")
        self.declare_parameter("run_id", "ros-demo")
        self.declare_parameter("processing_delay_ms", 20.0)
        self.declare_parameter("initial_s_m", 0.0)
        self.declare_parameter("initial_v_mps", 0.0)
        self.estimator = OdometryEstimator()
        self.epoch = 0
        self.last_stamp = None
        self.last_clock = None
        self.create_subscription(Clock, "/clock", self.clock_changed, 10)
        self.publisher = self.create_publisher(LongitudinalEstimate, "/odometry/estimate", 5)
        self.odom = self.create_publisher(Odometry, "/odometry/filtered", 5)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/odometry/diagnostics", 5)
        if self.get_parameter("input_mode").value not in ("batch", "topics"):
            raise ValueError("input_mode must be batch or topics")
        if self.get_parameter("input_mode").value == "batch":
            self.create_subscription(InputBatch, "/tram/events", self.batch, 10)
        else:
            self.create_subscription(ControlSample, "/tram/control", self.event, 10)
            self.create_subscription(WheelSample, "/tram/wheel_speed", self.event, qos_profile_sensor_data)
            self.create_timer(0.02, self.tick)

    def initialize(self, stamp):
        self.estimator.initialize(
            InitialState(
                stamp, self.get_parameter("initial_s_m").value, self.get_parameter("initial_v_mps").value
            ),
            EstimatorConfig(),
        )

    def event(self, msg):
        event = domain_event(msg)
        if self.estimator.t is None:
            self.initialize(event.stamp_ns)
        if isinstance(msg, ControlSample):
            self.estimator.ingest_control(event)
        else:
            self.estimator.ingest_wheel(event)

    def batch(self, msg):
        stamp = ns(msg.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.get_logger().warning("Discarding late/duplicate input batch", throttle_duration_sec=1)
            return
        if self.estimator.t is None:
            stamps = [ns(m.header.stamp) for m in list(msg.controls) + list(msg.wheels)]
            self.initialize(min(stamps + [stamp]))
        for event in msg.controls:
            self.event(event)
        for event in msg.wheels:
            self.event(event)
        self.publish(stamp)

    def clock_changed(self, msg):
        stamp = ns(msg.clock)
        if self.last_clock is not None and stamp < self.last_clock:
            self.epoch += 1
            self.estimator.reset()
            self.last_stamp = None
        self.last_clock = stamp

    def tick(self):
        stamp = self.get_clock().now().nanoseconds
        if stamp == 0:
            return
        stamp = max(0, stamp - round(self.get_parameter("processing_delay_ms").value * 1e6))
        if self.estimator.t is not None and stamp < self.estimator.t:
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.epoch += 1
                self.estimator.reset()
                self.last_stamp = None
            return
        if stamp != self.last_stamp:
            self.publish(stamp)

    def publish(self, stamp):
        began = time.perf_counter()
        result = self.estimator.advance_to(stamp)
        compute_ms = (time.perf_counter() - began) * 1000
        run_id = self.get_parameter("run_id").value
        if self.epoch:
            run_id += f"-epoch{self.epoch}"
        msg = estimate_message(result, run_id, compute_ms)
        self.publisher.publish(msg)
        if result.valid and result.uncertainty_available and result.covariance_4x4 is not None:
            odom = Odometry()
            odom.header, odom.child_frame_id = msg.header, "tram_1d"
            odom.pose.pose.position.x, odom.twist.twist.linear.x = result.s_m, result.v_mps
            odom.pose.pose.orientation.w = 1.0
            for i in (0, 7, 14, 21, 28, 35):
                odom.pose.covariance[i] = odom.twist.covariance[i] = 1e6
            odom.pose.covariance[0] = result.covariance_4x4[0]
            odom.twist.covariance[0] = result.covariance_4x4[5]
            self.odom.publish(odom)
        array = DiagnosticArray()
        array.header = msg.header
        status = DiagnosticStatus()
        status.name, status.hardware_id = "odometry", "baseline"
        status.level = DiagnosticStatus.WARN if result.valid else DiagnosticStatus.ERROR
        status.message = result.mode
        status.values = [
            KeyValue(key="reasons", value=",".join(result.reason_codes)),
            KeyValue(key="compute_ms", value=str(compute_ms)),
        ]
        array.status = [status]
        self.diagnostics.publish(array)
        self.last_stamp = stamp


def main():
    rclpy.init()
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
