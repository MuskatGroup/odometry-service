"""Run the actual ROS executable and rosbag player, save outputs and measured diagnostics.

Run inside the built Humble environment. Does not feed the reference topic to the estimator.
The optional /localization/kinematic_state reference is the organizer check-bag reference,
not claimed to be raw GNSS. Missing metrics are null, never zero.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import subprocess
import time
from bisect import bisect_left
from collections import Counter
from pathlib import Path


def percentile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * q))] if ordered else None


def score(output):
    with (output / "reference.csv").open() as handle:
        reference = sorted((int(r["stamp_ns"]), float(r["v_mps"]),
                            float(r["x"]), float(r["y"]), float(r["z"]))
                           for r in csv.DictReader(handle))
    times = [r[0] for r in reference]

    def nearest(stamp):
        index = bisect_left(times, stamp)
        choices = reference[max(0, index - 1):index + 1]
        if not choices:
            return None
        row = min(choices, key=lambda r: abs(r[0] - stamp))
        return row if abs(row[0] - stamp) <= 50_000_000 else None

    errors = []
    with (output / "velocity.csv").open() as handle:
        for row in csv.DictReader(handle):
            ref = nearest(int(row["stamp_ns"]))
            if ref is not None:
                errors.append(float(row["v_mps"]) - ref[1])
    position_errors = []
    with (output / "position.csv").open() as handle:
        for row in csv.DictReader(handle):
            ref = nearest(int(row["stamp_ns"]))
            if ref is not None and row["frame"] == "map":
                position_errors.append(math.sqrt(sum(
                    (float(row[name]) - ref[i]) ** 2 for i, name in enumerate(("x", "y", "z"), 2)
                )))
    return {
        "reference": "/localization/kinematic_state; nearest header within 50 ms; all estimates, including invalid",
        "speed_pairs": len(errors),
        "speed_rmse_mps": math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
        "speed_bias_mps": sum(errors) / len(errors) if errors else None,
        "map_position_pairs": len(position_errors),
        "map_position_rmse_m": math.sqrt(sum(e * e for e in position_errors) / len(position_errors))
        if position_errors else None,
    }


def main():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from odometry_msgs.msg import LongitudinalEstimate
    from odometry_node.conversion import ns
    from rclpy.node import Node
    from tram_vehicle_msgs.msg import VelocitySensor

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle-id", default="30618")
    parser.add_argument("--gnss-policy", default="disabled", choices=("disabled", "initialization_only", "intermittent"))
    parser.add_argument("--route-id", default="")
    parser.add_argument("--s0", type=float, default=0.0)
    parser.add_argument("--initial-v-mps", type=float, default=-1.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-offset", type=float, default=0.0, help="Seconds from bag beginning")
    parser.add_argument("--duration", type=float, help="Limit to this many bag-time seconds; omit for full bag")
    args = parser.parse_args()
    if args.rate <= 0 or args.start_offset < 0 or (args.duration is not None and args.duration <= 0):
        parser.error("rate/duration must be positive and start-offset non-negative")
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("output directory must be empty (existing artifacts are never overwritten)")
    args.output.mkdir(parents=True, exist_ok=True)
    handles = []

    def writer(name, columns):
        handle = (args.output / name).open("w", newline="", encoding="utf-8")
        handles.append(handle)
        result = csv.writer(handle)
        result.writerow(columns)
        return result

    velocity = writer("velocity.csv", ["stamp_ns", "v_mps", "received_monotonic_s"])
    position = writer("position.csv", ["stamp_ns", "frame", "x", "y", "z"])
    estimates = writer("estimates.csv", ["stamp_ns", "s_m", "v_mps", "valid", "mode", "reasons"])
    reference = writer("reference.csv", ["stamp_ns", "v_mps", "x", "y", "z"])
    diagnostics = (args.output / "diagnostics.jsonl").open("w", encoding="utf-8")
    handles.append(diagnostics)
    rclpy.init()
    node = Node("reserve_validation_collector")
    counts, modes, frames = Counter(), Counter(), Counter()
    stamp_ranges, last_wall, gaps = {}, {}, {"velocity": [], "position": []}
    latest_diagnostics, maxima = {}, {}
    last_diagnostic_write = 0.0
    first_estimate = last_estimate = None

    def received(kind, stamp):
        now = time.monotonic()
        counts[kind] += 1
        if kind in last_wall:
            gaps[kind].append(now - last_wall[kind])
        last_wall[kind] = now
        limits = stamp_ranges.setdefault(kind, [stamp, stamp])
        limits[1] = stamp
        return now

    def on_velocity(msg):
        stamp = ns(msg.header.stamp)
        velocity.writerow([stamp, msg.velocity, received("velocity", stamp)])

    def on_position(msg):
        stamp = ns(msg.header.stamp)
        received("position", stamp)
        p = msg.pose.pose.position
        position.writerow([stamp, msg.header.frame_id, p.x, p.y, p.z])
        frames[msg.header.frame_id] += 1

    def on_reference(msg):
        p = msg.pose.pose.position
        reference.writerow([ns(msg.header.stamp), msg.twist.twist.linear.x, p.x, p.y, p.z])

    def on_estimate(msg):
        nonlocal first_estimate, last_estimate
        last_estimate = ns(msg.header.stamp)
        if first_estimate is None:
            first_estimate = last_estimate
        counts["estimate"] += 1
        counts["valid"] += int(msg.valid)
        modes[msg.mode] += 1
        estimates.writerow([last_estimate, msg.s_m, msg.v_mps, int(msg.valid), msg.mode, ",".join(msg.reason_codes)])

    def on_diagnostics(msg):
        nonlocal last_diagnostic_write
        for status in msg.status:
            values = {item.key: item.value for item in status.values}
            latest_diagnostics.update(values)
            for key in ("latency_max_ms", "latency_p99_ms", "rss_mb", "compute_ms", "cpu_cores", "crash_count"):
                try:
                    value = float(values[key])
                except (KeyError, ValueError):
                    continue
                if math.isfinite(value):
                    maxima[key] = max(maxima.get(key, 0), value)
            if time.monotonic() - last_diagnostic_write >= 1:
                diagnostics.write(json.dumps({"stamp_ns": ns(msg.header.stamp), **values}) + "\n")
                last_diagnostic_write = time.monotonic()

    node.create_subscription(VelocitySensor, "/result/velocity", on_velocity, 200)
    node.create_subscription(Odometry, "/result/position", on_position, 200)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_reference, 200)
    node.create_subscription(LongitudinalEstimate, "/odometry/estimate", on_estimate, 200)
    node.create_subscription(DiagnosticArray, "/odometry/diagnostics", on_diagnostics, 200)
    runtime_log = (args.output / "runtime.log").open("w")
    player_log = (args.output / "player.log").open("w")
    processes = []
    started = time.monotonic()
    completed = False
    try:
        runtime = subprocess.Popen([
            "ros2", "launch", "odometry_node", "reserve.launch.py", "use_sim_time:=true",
            f"vehicle_id:={args.vehicle_id}", f"gnss_policy:={args.gnss_policy}",
            *([f"route_id:={args.route_id}"] if args.route_id else []),
            f"s0:={args.s0}", f"initial_v_mps:={args.initial_v_mps}",
        ], stdout=runtime_log, stderr=runtime_log)
        processes.append(runtime)
        discovery_deadline = time.monotonic() + 20
        while node.count_publishers("/result/velocity") == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if runtime.poll() is not None or time.monotonic() > discovery_deadline:
                raise RuntimeError("Runtime did not become ready; inspect runtime.log")
        player = subprocess.Popen([
            "ros2", "bag", "play", str(args.bag), "--clock", "100", "--delay", "2",
            "--rate", str(args.rate), "--disable-keyboard-controls",
            "--start-offset", str(args.start_offset),
        ], stdout=player_log, stderr=player_log)
        processes.append(player)
        last_progress = time.monotonic()
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if runtime.poll() is not None:
                raise RuntimeError("Runtime exited during playback")
            if args.duration and first_estimate is not None and (last_estimate - first_estimate) / 1e9 >= args.duration:
                break
            if first_estimate is None and time.monotonic() - started > 30:
                raise RuntimeError("No outputs within 30 seconds; inspect logs")
            if time.monotonic() - last_progress > 30:
                print(f"outputs={dict(counts)}, modes={dict(modes)}", flush=True)
                last_progress = time.monotonic()
        completed = player.poll() == 0
        drain_until = time.monotonic() + 0.5
        while time.monotonic() < drain_until:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        runtime_log.close()
        player_log.close()
        for handle in handles:
            handle.close()
    rates = {}
    for kind, (first, last) in stamp_ranges.items():
        rates[kind] = {
            "bag_time_hz": (counts[kind] - 1) * 1e9 / (last - first) if last > first else None,
            "wall_gap_p99_ms": percentile([g * 1000 for g in gaps[kind]], 0.99),
            "wall_gap_max_ms": max(gaps[kind], default=0) * 1000,
        }
    report = {
        "bag": str(args.bag), "full_bag_completed": completed and args.start_offset == 0,
        "playback_reached_end": completed, "rate": args.rate, "start_offset_s": args.start_offset,
        "gnss_policy": args.gnss_policy, "route_id": args.route_id or None,
        "wall_seconds": time.monotonic() - started, "counts": dict(counts), "modes": dict(modes),
        "position_frames": dict(frames), "rates": rates, "diagnostic_maxima": maxima,
        "last_diagnostics": latest_diagnostics, "metrics": score(args.output),
        "latency_scope": "Node callback entry through publication; excludes publisher/network/DDS pre-callback time",
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    if counts["velocity"] == 0 or counts["position"] == 0 or maxima.get("crash_count", 0):
        raise SystemExit("Validation failed: missing outputs or runtime exceptions")
    if not completed and args.duration is None:
        raise SystemExit("Validation failed: bag did not complete")


if __name__ == "__main__":
    main()
