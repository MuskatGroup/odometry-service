"""Run the unmodified organizer MetricsNode against the built production ROS node.

Run inside the Humble container after sourcing both workspaces. Outputs stay in
artifacts; no estimator parameters are fitted to this bag.
"""

import argparse
import json
import math
import os
import signal
import sqlite3
import subprocess
import time
from collections import Counter
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from hackathon_solution_checker.metrics import MetricsNode
from nav_msgs.msg import Odometry
from odometry_geometry import PathGraph
from odometry_msgs.msg import LongitudinalEstimate
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message

p = argparse.ArgumentParser()
p.add_argument("--name", required=True)
p.add_argument("--policy", default="disabled")
p.add_argument("--offset", type=float, default=230)
p.add_argument("--rate", type=float, default=1)
p.add_argument("--duration", type=float, default=60, help="Simulated seconds; 0 means complete bag")
a = p.parse_args()
if a.rate <= 0 or a.offset < 0 or a.duration < 0:
    p.error("rate must be positive; offset and duration must be non-negative")
root = Path(__file__).resolve().parents[1]
bag = root / "tests/check-code/bags/30618_88aea4d9"
out = root / "artifacts/organizer-audit" / a.name
if out.exists():
    p.error("Output directory already exists; choose a new --name to preserve previous evidence")
out.mkdir(parents=True, exist_ok=True)
seed = None
if a.policy == "disabled":
    with sqlite3.connect(f"file:{next(bag.glob('*.db3'))}?mode=ro", uri=True) as db:
        start = db.execute("select min(timestamp) from messages").fetchone()[0]
        row = db.execute(
            "select timestamp,data from messages where topic_id in "
            '(select id from topics where name="/localization/kinematic_state") '
            "and timestamp>=? order by timestamp limit 1",
            (start + round(a.offset * 1e9),),
        ).fetchone()
    ref = deserialize_message(row[1], Odometry)
    pos, q = ref.pose.pose.position, ref.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    match = PathGraph.from_directory(root / "dataset/Pathgraph").project(pos.x, pos.y, yaw)
    if match.status != "MATCHED":
        raise RuntimeError(f"Cannot initialize strict run: {match}")
    seed = dict(
        route_id=match.route_id,
        s0=match.s_m,
        initial_v_mps=max(0.0, ref.twist.twist.linear.x),
        source="one reference pose and speed at start; no subsequent reference fed to estimator",
        reference_recorded_ns=str(row[0]),
        cross_track_m=match.cross_track_m,
    )

rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
checker = MetricsNode()
counts = Counter()
modes = Counter()
reasons = Counter()
compute = []
stamps = []
last_estimate = {}
last_diag = {}
maxima = {}
first_rss = None


def estimate(msg):
    global last_estimate
    counts["estimate"] += 1
    modes[msg.mode] += 1
    reasons.update(msg.reason_codes)
    counts["valid"] += int(msg.valid)
    counts["map_pose"] += int(msg.has_map_pose)
    compute.append(msg.compute_ms)
    stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
    last_estimate = {
        k: getattr(msg, k)
        for k in [
            "s_m",
            "v_mps",
            "valid",
            "mode",
            "accepted_wheel_count",
            "rejected_wheel_count",
            "accepted_gnss_position_count",
            "accepted_gnss_velocity_count",
        ]
    }


def diagnostics(msg):
    global last_diag, first_rss
    if not msg.status:
        return
    last_diag = {v.key: v.value for v in msg.status[0].values}
    for key in ("compute_ms", "rss_mb", "cpu_cores", "queue_depth", "crash_count", "too_late_input_count"):
        if key in last_diag:
            maxima[key] = max(maxima.get(key, 0), float(last_diag[key]))
    if first_rss is None and "rss_mb" in last_diag:
        first_rss = float(last_diag["rss_mb"])


checker.create_subscription(LongitudinalEstimate, "/odometry/estimate", estimate, qos_profile_sensor_data)
checker.create_subscription(DiagnosticArray, "/odometry/diagnostics", diagnostics, qos_profile_sensor_data)


def reference_seen(msg):
    counts["reference"] += 1


checker.create_subscription(
    Odometry, "/localization/kinematic_state", reference_seen, qos_profile_sensor_data
)
cmd = [
    "ros2",
    "run",
    "odometry_node",
    "reserve_odometry_node",
    "--ros-args",
    "-p",
    "use_sim_time:=true",
    "-p",
    f"gnss_policy:={a.policy}",
    "-p",
    "vehicle_id:='30618'",
    "-p",
    "pathgraph_directory:=/repo/dataset/Pathgraph",
    "-p",
    "model_config:=/repo/configs/models",
]
if seed:
    for key in ("route_id", "s0", "initial_v_mps"):
        cmd.extend(["-p", f"{key}:={seed[key]}"])
logs = []
processes = []
began = time.monotonic()
player = None
runtime_exit = None
player_exit = None
try:
    log = (out / "runtime.log").open("w")
    logs.append(log)
    runtime = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(runtime)
    log = (out / "recorder.log").open("w")
    logs.append(log)
    recorder = subprocess.Popen(
        [
            "ros2",
            "bag",
            "record",
            "-o",
            str(out / "results"),
            "/result/velocity",
            "/result/position",
            "/odometry/estimate",
            "/odometry/diagnostics",
            "/localization/kinematic_state",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    processes.append(recorder)
    for _ in range(30):
        rclpy.spin_once(checker, timeout_sec=0.1)
    log = (out / "player.log").open("w")
    logs.append(log)
    play = [
        "ros2",
        "bag",
        "play",
        str(bag),
        "--clock",
        "100",
        "--rate",
        str(a.rate),
        "--start-offset",
        str(a.offset),
        "--delay",
        "2",
    ]
    player = subprocess.Popen(play, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(player)
    deadline = time.monotonic() + (a.duration / a.rate + 5 if a.duration else 1400 / a.rate + 20)
    while player.poll() is None and runtime.poll() is None and time.monotonic() < deadline:
        rclpy.spin_once(checker, timeout_sec=0.05)
    runtime_exit = runtime.poll()
    player_exit = player.poll()
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    for log in logs:
        log.close()
    checker.report()

    def metric(m):
        return dict(n=m.count, rmse=m.rmse if m.count else None, max=m.maximum_error if m.count else None)

    compute.sort()
    result = dict(
        arguments=vars(a),
        seed=seed,
        wall_seconds=time.monotonic() - began,
        runtime_exit_before_shutdown=runtime_exit,
        player_exit_before_shutdown=player_exit,
        velocity=metric(checker.velocity_metric),
        position={k: metric(v) for k, v in checker.position_metrics.items()},
        counts=dict(counts),
        modes=dict(modes),
        reasons=dict(reasons),
        last_estimate=last_estimate,
        first_output_stamp=stamps[0] if stamps else None,
        last_output_stamp=stamps[-1] if stamps else None,
        last_diagnostics=last_diag,
        maxima=maxima,
        first_rss_mb=first_rss,
        output_hz_simulated=(len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else None,
        compute_p99_ms=compute[min(len(compute) - 1, int(0.99 * len(compute)))] if compute else None,
    )
    (out / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    checker.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)

if runtime_exit is not None or player_exit not in (None, 0) or not counts["estimate"]:
    raise SystemExit("Run failed: inspect summary.json and process logs")
if a.duration == 0 and player_exit is None:
    raise SystemExit("Full playback did not finish before the timeout")
