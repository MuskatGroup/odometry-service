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
