"""Run inside the Humble image; exercises DDS and rosbag, not mocks."""

import os
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_TESTS") != "1", reason="Requires built ROS Humble workspace"
)


def test_ros_and_bag_match_offline(tmp_path):
    import rclpy
    from odometry_core import ControlSample, WheelSample
    from odometry_lab.runner import run_events
    from odometry_msgs.msg import InputBatch, LongitudinalEstimate
    from odometry_node.conversion import event_message, set_stamp
    from odometry_node.estimator import EstimatorNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    events = []
    for i in range(101):
        stamp = i * 20_000_000
        events += [ControlSample(stamp, i, 0), WheelSample(stamp, i, "left", 10)]
    expected, _ = run_events(events, estimator_name="adaptive-ekf")
    rclpy.init()
    node = Node("integration_source")
    estimator = EstimatorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(estimator)
    publisher = node.create_publisher(InputBatch, "/tram/events", 10)
    received = []
    node.create_subscription(
        LongitudinalEstimate, "/odometry/estimate", lambda msg: received.append(msg), 200
    )
    recorder = subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(tmp_path / "bag"), "/tram/events"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    def spin_until(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "ROS discovery/delivery timeout"

    try:
        spin_until(lambda: publisher.get_subscription_count() >= 2)
        # Give output subscription discovery a bounded opportunity before publishing.
        spin_until(lambda: estimator.publisher.get_subscription_count() >= 1)
        for i in range(101):
            batch = InputBatch()
            set_stamp(batch.header, i * 20_000_000)
            batch.controls = [event_message(events[2 * i])]
            batch.wheels = [event_message(events[2 * i + 1])]
            publisher.publish(batch)
            spin_until(lambda: len(received) >= i + 1)
            # Record at the intended 50 Hz, so playback is not an artificial burst.
            executor.spin_once(timeout_sec=0.02)
        assert received[-1].s_m == pytest.approx(expected[-1]["s_m"])
        assert received[-1].uncertainty_available
        recorder.send_signal(signal.SIGINT)
        recorder.wait(timeout=10)
        executor.remove_node(estimator)
        estimator.destroy_node()
        received.clear()
        estimator = EstimatorNode()
        executor.add_node(estimator)
        spin_until(lambda: publisher.get_subscription_count() >= 1)
        player_log = (tmp_path / "player.log").open("w+")
        player = subprocess.Popen(
            ["ros2", "bag", "play", str(tmp_path / "bag"), "--delay", "1"],
            stdout=player_log, stderr=player_log,
        )
        try:
            try:
                spin_until(lambda: len(received) >= len(expected), timeout=20)
            except AssertionError:
                player_log.flush()
                pytest.fail(f"Received {len(received)}/{len(expected)}; player exit={player.poll()}; "
                            + (tmp_path / "player.log").read_text())
            for actual, reference in zip(received, expected):
                assert actual.s_m == pytest.approx(reference["s_m"])
                assert actual.v_mps == pytest.approx(reference["v_mps"])
            player.wait(timeout=10)
        finally:
            if player.poll() is None:
                player.terminate()
                player.wait(timeout=5)
            player_log.close()
    finally:
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=10)
        executor.shutdown()
        node.destroy_node()
        estimator.destroy_node()
        rclpy.shutdown()


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
        # `delayed` must land safely after the estimator's own start time (its init stamp is the
        # very first message's header, above) or there is no fixed-lag history left to insert it
        # into and it is correctly rejected as OUT_OF_ORDER -- not a bug, just an earlier-than-init
        # timestamp. On a slow/loaded CI runner the two publish calls above can land within a few
        # milliseconds of each other, so "wall clock now minus 80 ms" is not reliably later than
        # init; wait out a fixed buffer well past the 80 ms offset first so the test's own timing
        # never decides whether it passes (organizer audit, 2026-09-27: this exact race was
        # observed failing under load).
        buffer_deadline = time.monotonic() + 0.3
        while time.monotonic() < buffer_deadline:
            executor.spin_once(timeout_sec=0.02)
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
