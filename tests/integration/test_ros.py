"""Run inside the Humble image; exercises DDS and rosbag, not mocks."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_TESTS") != "1", reason="Requires built ROS Humble workspace"
)


def test_reserve_node_uses_organizer_contracts_and_si_outputs():
    import rclpy
    from nav_msgs.msg import Odometry
    from odometry_core import ModelConfig
    from odometry_node.runtime import ReserveOdometryNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

    class Pose:
        x_m = 103500.0
        y_m = 85876.0
        z_m = 164.0
        yaw_rad = 0.0

    class Geometry:
        def grade_at(self, route_id, s_m):
            return 0.0

        def body_pose_at(self, route_id, s_m):
            pose = Pose()
            pose.x_m += s_m
            return pose

    rclpy.init()
    source = Node("organizer_source")
    node = ReserveOdometryNode(
        geometry=Geometry(),
        model_config=ModelConfig(control_timeout_s=None),
        parameter_overrides=[
            Parameter("route_id", value="route-a"),
            Parameter("initial_v_mps", value=10.0),
            Parameter("processing_delay_ms", value=120.0),
        ],
    )
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(node)
    controls = source.create_publisher(DriverControllerCommand, "/vehicle/driver_position_cmd", 10)
    front = source.create_publisher(VelocitySensor, "/vehicle/front_bogie_velocity", 10)
    rear = source.create_publisher(VelocitySensor, "/vehicle/rear_bogie_velocity", 10)
    velocities, positions = [], []
    source.create_subscription(VelocitySensor, "/result/velocity", velocities.append, 10)
    source.create_subscription(Odometry, "/result/position", positions.append, 10)
    try:
        deadline = time.monotonic() + 10
        while front.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert front.get_subscription_count() == rear.get_subscription_count() == 1
        now = source.get_clock().now().to_msg()
        control = DriverControllerCommand()
        control.header.stamp = now
        control.position = 0
        wheel = VelocitySensor()
        wheel.header.stamp = now
        wheel.velocity = 36.0
        controls.publish(control)
        front.publish(wheel)
        rear.publish(wheel)
        deadline = time.monotonic() + 5
        while (not velocities or not positions) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert velocities[-1].velocity == pytest.approx(10.0, abs=0.2)
        assert positions[-1].header.frame_id == "map"
        assert positions[-1].child_frame_id == "base_link"

        # Organizer bags deliver wheel headers tens of milliseconds behind /clock. The
        # fixed-lag commit must accept these samples while publishing a current-time preview.
        #
        # `delayed` (80 ms behind "now", computed just below) must land safely after the
        # estimator's own start time (its init stamp is the very first message's header, above) or
        # there is no fixed-lag history left to insert it into and it is correctly rejected as
        # OUT_OF_ORDER -- not a bug, just an earlier-than-init timestamp. On a slow/loaded CI
        # runner the publish calls above can land within a few milliseconds of each other, so
        # "wall clock now minus 80 ms" is not reliably later than init (organizer audit,
        # 2026-09-27: this exact race was observed failing under load). Wait for the invariant
        # itself -- at least 160 ms (double the 80 ms offset, for margin) actually elapsed since
        # the first message's stamp -- rather than a fixed sleep that merely tends to be enough.
        first_stamp = now.sec * 1_000_000_000 + now.nanosec
        deadline = time.monotonic() + 2
        while source.get_clock().now().nanoseconds < first_stamp + 160_000_000:
            executor.spin_once(timeout_sec=0.01)
            assert time.monotonic() < deadline
        delayed = source.get_clock().now().nanoseconds - 80_000_000
        wheel.header.stamp.sec, wheel.header.stamp.nanosec = divmod(delayed, 1_000_000_000)
        front.publish(wheel)
        rear.publish(wheel)
        accepted_before = node.estimator.accepted
        deadline = time.monotonic() + 2
        while node.estimator.accepted <= accepted_before and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert node.estimator.rejected == 0
        # A front/rear pair at one timestamp is fused as one median wheel update.
        assert node.estimator.accepted >= accepted_before + 1
        assert node.too_late_input_count == 0
    finally:
        executor.shutdown()
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
