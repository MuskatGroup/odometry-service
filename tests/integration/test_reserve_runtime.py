"""Runtime regressions; run with RUN_ROS_TESTS=1 in the built Humble workspace."""
import os
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("RUN_ROS_TESTS") != "1", reason="Requires ROS Humble")


@pytest.fixture
def runtime():
    import rclpy
    from odometry_core import ModelConfig
    from odometry_node.runtime import ReserveOdometryNode

    rclpy.init()
    node = ReserveOdometryNode(model_config=ModelConfig(c1=0.0))
    node.timer.cancel()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_relative_position_and_moving_start_without_map(runtime):
    from odometry_node.conversion import ns
    from tram_vehicle_msgs.msg import VelocitySensor

    wheel = VelocitySensor()
    wheel.header.stamp = runtime.get_clock().now().to_msg()
    wheel.velocity = 36.0
    runtime._wheel(wheel, "front")
    result = runtime.estimator.advance_to(ns(wheel.header.stamp) + 20_000_000)
    pose = runtime._odometry(result, None, result.stamp_ns)
    assert pose.header.frame_id == "odom"
    assert pose.twist.twist.linear.x == pytest.approx(10.0)
    assert pose.pose.pose.position.x == pytest.approx(0.2)
    assert pose.pose.covariance[35] == 1e6
    assert not runtime.gnss_subscriptions


def test_outside_map_keeps_velocity_valid_and_publishes_track_coordinate(runtime):
    from odometry_core import ControlSample, InitialState

    class Outside:
        def body_pose_at(self, route, s):
            raise ValueError("OUT_OF_GRAPH")

    runtime.geometry = Outside()
    runtime.route_id = "test-route"
    runtime.estimator.initialize(InitialState(0, 5000.0, 10.0))
    runtime.estimator.ingest_control(ControlSample(0, 0, 0.0))
    result = runtime.estimator.advance_to(20_000_000)
    runtime._publish(result, result.stamp_ns, 0.1)
    assert result.valid
    assert "OUT_OF_GRAPH" in result.reason_codes
    pose = runtime._odometry(result, None, result.stamp_ns)
    assert pose.header.frame_id == "track/test-route"
    assert pose.pose.pose.position.x > 5000.0


def test_latency_includes_time_waiting_for_publication(runtime):
    from odometry_core import InitialState

    runtime.estimator.initialize(InitialState(0))
    result = runtime.estimator.advance_to(20_000_000)
    runtime.pending_input_timing.append((0, time.perf_counter_ns() - 50_000_000))
    runtime._publish(result, result.stamp_ns, 0.1)
    assert runtime.latest_input_to_publication_ms >= 50.0
    assert len(runtime.latency_ms) == 1
    runtime._publish(result, result.stamp_ns + 20_000_000, 0.1)
    assert runtime.latest_input_to_publication_ms is None
    assert len(runtime.latency_ms) == 1


def test_initialization_window_closes_without_accepted_fix(runtime):
    runtime.gnss_policy = "initialization_only"
    runtime.input_origin_stamp = 1_000_000_000
    assert runtime._gnss_allowed(2_000_000_000)
    assert not runtime._gnss_allowed(7_000_000_000)


def test_late_gnss_anchor_does_not_rewind_filter(runtime):
    from odometry_core import ControlSample, InitialState
    from odometry_node.geometry_adapter import GnssMapObservation

    runtime.gnss_policy = "initialization_only"
    runtime.input_origin_stamp = 0
    runtime.estimator.initialize(InitialState(0, 0.0, 10.0))
    runtime.estimator.ingest_control(ControlSample(0, 0, 0.5))
    runtime.estimator.advance_to(300_000_000)
    runtime._apply_observation(
        GnssMapObservation("route", 3000.0, 0, 0, 0, 0, 0, None), 100_000_000, 4.0, "gnss",
    )
    assert runtime.estimator.t == 300_000_000
    assert runtime.estimator.control == 0.5
    assert runtime.route_id is None
    assert "OUT_OF_ORDER" in runtime.estimator.pending_reasons


def test_gnss_close_is_safe_with_inflight_multithreaded_callbacks():
    import threading
    from types import SimpleNamespace

    import rclpy
    from odometry_core import ModelConfig
    from odometry_node.geometry_adapter import GnssMapObservation
    from odometry_node.runtime import ReserveOdometryNode
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import NavSatFix

    class Geometry:
        def observe_dual(self, *args):
            return GnssMapObservation("route-a", 3000.0, 1, 2, 3, 0, 0, 0)

        def observe_single(self, *args):
            return self.observe_dual()

        def grade_at(self, *args):
            return 0.0

        def body_pose_at(self, *args):
            return SimpleNamespace(x_m=1.0, y_m=2.0, z_m=3.0, yaw_rad=0.0)

    rclpy.init()
    node = ReserveOdometryNode(
        geometry=Geometry(), model_config=ModelConfig(),
        parameter_overrides=[Parameter("gnss_policy", value="initialization_only")],
    )
    source = Node("gnss_closing_source")
    publishers = [source.create_publisher(NavSatFix, f"/sensing/gnss/{name}/fix", 100)
                  for name in ("master", "rover")]
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(source)
    failures = []

    def spin():
        try:
            executor.spin()
        except Exception as exc:
            failures.append(exc)

    thread = threading.Thread(target=spin)
    thread.start()
    try:
        deadline = time.monotonic() + 5
        while not all(p.get_subscription_count() for p in publishers):
            assert time.monotonic() < deadline
            time.sleep(0.01)
        # Keep a steady flood of inflight fixes landing on the reentrant GNSS subscriptions
        # while the node closes its GNSS initialization window on the timer thread, instead of
        # a fixed publish count/sleep: the accept-and-close itself is fast (one accepted dual fix
        # is enough), but it shares this tick with the node's *first-ever* logger call (the flat-
        # grade fallback WARN below), and rclpy/rcl's lazy logging init on that first call can
        # itself take the better part of a second in this environment -- a fixed 60 * 10ms = 600ms
        # publish loop can (and did) end and assert before that tick returns. Bound by a generous
        # deadline and wait on the actual condition instead, the same pattern test_ros.py's
        # test_reserve_node_uses_organizer_contracts_and_si_outputs uses for its own clock-based
        # race (waiting on `first_stamp + 160_000_000` rather than a fixed sleep).
        deadline = time.monotonic() + 10
        fix = None
        while not (node.gnss_closed and node.gnss_initialized):
            assert time.monotonic() < deadline, "GNSS initialization window never closed"
            assert not failures
            fix = NavSatFix()
            fix.header.stamp = source.get_clock().now().to_msg()
            fix.status.status = 0
            fix.latitude, fix.longitude, fix.altitude = 55.8, 37.4, 160.0
            for publisher in publishers:
                publisher.publish(fix)
            time.sleep(0.01)
        assert not failures
        assert node.gnss_closed and node.gnss_initialized
        assert node.estimator.accepted_gnss_position >= 1
        accepted = node.estimator.accepted_gnss_position
        node._fix(fix, "master")
        assert node.estimator.accepted_gnss_position == accepted
        assert node.crash_count == 0
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
