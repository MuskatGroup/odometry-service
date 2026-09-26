import asyncio
import queue
import threading
import time

import rclpy
from odometry_core import ControlSample
from odometry_io import Normalizer, load_profile, read_file, websocket_records
from odometry_msgs.msg import ControlSample as ControlMsg
from odometry_msgs.msg import InputBatch
from odometry_msgs.msg import WheelSample as WheelMsg
from rclpy.clock import Clock as RclClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from .conversion import event_message, set_stamp


class ReplayNode(Node):
    def __init__(self):
        super().__init__("replay")
        self.declare_parameter("source", "/artifacts/demo/events.jsonl")
        self.declare_parameter("profile", "/workspace/contracts/profiles/events.yaml")
        self.declare_parameter("tail_s", 3.0)
        self.source = self.get_parameter("source").value
        self.normalizer = Normalizer(load_profile(self.get_parameter("profile").value))
        self.batch_pub = self.create_publisher(InputBatch, "/tram/events", 10)
        self.control_pub = self.create_publisher(ControlMsg, "/tram/control", 10)
        self.wheel_pub = self.create_publisher(WheelMsg, "/tram/wheel_speed", 10)
        self.live = self.source.startswith(("ws://", "wss://"))
        self.clock_pub = None if self.live else self.create_publisher(Clock, "/clock", 10)
        self.position = 0
        self.started = time.monotonic()
        self.finished = False
        self.queue = queue.Queue(maxsize=4096)
        self.stop = threading.Event()
        if self.live:
            if self.normalizer.profile["time"]["clock"] != "system":
                raise ValueError("Live WebSocket ROS source requires an explicit system-clock profile")
            threading.Thread(target=self.receive, daemon=True).start()
            self.origin = None
            self.live_start = None
        else:
            self.events = read_file(self.source, self.normalizer)
            if not self.events:
                raise ValueError("No input events")
            self.stamp = self.events[0].stamp_ns
            self.end = self.events[-1].stamp_ns + round(self.get_parameter("tail_s").value * 1e9)
        # File replay advances simulated time from a steady wall-clock timer.
        self.timer = self.create_timer(0.02, self.tick, clock=RclClock(clock_type=ClockType.STEADY_TIME))

    def receive(self):
        async def read():
            try:
                async for event in websocket_records(self.source, self.normalizer):
                    if self.stop.is_set():
                        break
                    try:
                        self.queue.put_nowait(event)
                    except queue.Full:
                        self.normalizer.diagnostics["QUEUE_OVERFLOW"] += 1
            except Exception as exc:
                self.get_logger().error(f"WebSocket source error: {exc}")
            finally:
                self.get_logger().warning("WebSocket disconnected; freshness will expire")

        asyncio.run(read())

    def tick(self):
        if self.finished or time.monotonic() - self.started < 1.0:
            return
        if self.batch_pub.get_subscription_count() == 0:
            return
        batch = InputBatch()
        events = []
        if self.live:
            while not self.queue.empty():
                events.append(self.queue.get_nowait())
            if self.origin is None:
                if not events:
                    return
                # Live source must use ROS system-clock epoch; no inferred offset.
                self.origin = events[0].stamp_ns
                if abs(self.get_clock().now().nanoseconds - self.origin) > 5_000_000_000:
                    self.get_logger().error("Source and ROS clock differ by more than 5 seconds; stopped")
                    self.finished = True
                    return
            stamp = self.get_clock().now().nanoseconds
        else:
            stamp = self.stamp
            while self.position < len(self.events) and self.events[self.position].stamp_ns <= stamp:
                events.append(self.events[self.position])
                self.position += 1
        set_stamp(batch.header, stamp)
        for event in events:
            msg = event_message(event)
            if isinstance(event, ControlSample):
                batch.controls.append(msg)
                self.control_pub.publish(msg)
            else:
                batch.wheels.append(msg)
                self.wheel_pub.publish(msg)
        # Consumers choose either atomic batches OR individual topics, never both.
        self.batch_pub.publish(batch)
        if self.clock_pub:
            clock = Clock()
            clock.clock = batch.header.stamp
            self.clock_pub.publish(clock)
            if stamp >= self.end:
                self.finished = True
                self.get_logger().info(f"Replay complete. Diagnostics: {dict(self.normalizer.diagnostics)}")
            else:
                self.stamp = min(self.end, self.stamp + 20_000_000)


def main():
    rclpy.init()
    node = ReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop.set()
        node.destroy_node()
        rclpy.shutdown()
