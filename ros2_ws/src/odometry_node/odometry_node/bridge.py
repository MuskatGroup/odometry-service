import json
import queue
import threading
from urllib.error import URLError
from urllib.request import Request, urlopen

import rclpy
from odometry_msgs.msg import LongitudinalEstimate
from rclpy.node import Node

from .conversion import frame_dict


def request(base, path, body):
    req = Request(
        base.rstrip("/") + path,
        data=json.dumps(body, allow_nan=False).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(req, timeout=2) as response:
        return response.read()


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__("telemetry_bridge")
        self.declare_parameter("api", "http://console:8080")
        self.frames = queue.Queue(maxsize=100)
        self.stop = threading.Event()
        self.last_sent_stamp = {}
        self.dropped = 0
        self.create_subscription(LongitudinalEstimate, "/odometry/estimate", self.receive, 5)
        threading.Thread(target=self.worker, daemon=True).start()

    def receive(self, msg):
        frame = frame_dict(msg)
        last = self.last_sent_stamp.get(msg.run_id, -100_000_000)
        stamp = int(frame["stamp_ns"])
        if stamp - last < 100_000_000:
            return
        self.last_sent_stamp[msg.run_id] = stamp
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            self.frames.put_nowait(frame)

    def worker(self):
        registered = set()
        api = self.get_parameter("api").value
        while not self.stop.is_set():
            try:
                frame = self.frames.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if frame["run_id"] not in registered:
                    request(
                        api,
                        "/api/runs",
                        {
                            "run_id": frame["run_id"],
                            "status": "running",
                            "model_version": frame["model_version"],
                        },
                    )
                    registered.add(frame["run_id"])
                request(api, f"/api/runs/{frame['run_id']}/telemetry", [frame])
            except (URLError, OSError, TimeoutError) as exc:
                self.dropped += 1
                registered.discard(frame["run_id"])
                self.get_logger().warning(f"Telemetry unavailable; dropped={self.dropped}: {exc}")
                self.stop.wait(1)


def main():
    rclpy.init()
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.stop.set()
        node.destroy_node()
        rclpy.shutdown()
