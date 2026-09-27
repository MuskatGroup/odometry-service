"""Full-length, single-pass run of the production estimator over one entire real bag.

This is NOT the organizer-audit ROS+Docker stress repro (tools/run_organizer_audit.py) --
this environment has no Docker daemon and no rclpy, so the real ROS transport, DDS,
MultiThreadedExecutor and true 1x wall-clock pacing cannot be exercised here. What this
DOES check, with the exact same production AdaptiveOdometryEstimator class and the same
model-profile loader the ROS node uses: does a full ~30+ minute bag process end to end
without crashing or unbounded memory growth, what does per-tick compute cost look like
over the whole run (not just a 120s excerpt), and what does accuracy/availability look
like against the GNSS-derived reference wherever it has coverage.

    python tools/full_run_check.py --bag 30618_27e994fc --output artifacts/full-run

Writes <output>/<bag_id>.json with the full report.
"""

from __future__ import annotations

import argparse
import json
import resource
import time
from pathlib import Path

from odometry_core import InitialState
from odometry_io import load_model_config
from odometry_lab.realdata import _tram_bag, events_from_bag, load_reference
from odometry_lab.runner import run_events
from odometry_lab.evaluate import evaluate
from odometry_lab.storage import write_json, write_rows

REPO = Path(__file__).resolve().parents[1]
NS = 1_000_000_000


def _percentile(values, q):
    values = sorted(v for v in values if v is not None)
    if not values:
        return None
    return values[min(len(values) - 1, round((len(values) - 1) * q))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Bag id, e.g. 30618_27e994fc")
    parser.add_argument("--data", default="dataset/data")
    parser.add_argument("--derived", default="dataset/derived")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    bag_dir = Path(args.data) / args.bag
    reference_csv = Path(args.derived) / "reference" / f"{args.bag}.csv"

    tram_bag = _tram_bag()
    bag = tram_bag.read_bag(bag_dir)
    events = events_from_bag(bag)
    events.sort(key=lambda e: e.stamp_ns)
    reference = load_reference(reference_csv) if reference_csv.exists() else []

    duration_s = (events[-1].stamp_ns - events[0].stamp_ns) / NS

    profile_path = REPO / "configs" / "models" / f"{bag.vehicle_id}.yaml"
    if not profile_path.exists():
        profile_path = REPO / "configs" / "models" / "default.yaml"
    model_config, _, selected = load_model_config(profile_path, bag.vehicle_id)

    with_v = [r for r in reference if r["v"] is not None]
    if with_v:
        first = with_v[0]
        initial = InitialState(events[0].stamp_ns, 0.0, first["v"])
    else:
        initial = InitialState(events[0].stamp_ns, 0.0, 0.0)

    rss_before_kb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wall_start = time.perf_counter()
    frames, counts = run_events(
        events, initial=initial, config=model_config, hz=args.hz, estimator_name="adaptive-ekf"
    )
    wall_elapsed_s = time.perf_counter() - wall_start
    rss_after_kb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    timings = [f["compute_ms"] for f in frames if f.get("compute_ms") is not None]
    valid = sum(1 for f in frames if f.get("valid"))
    model_only = sum(1 for f in frames if "MODEL_ONLY" in str(f.get("mode", "")).upper())

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    report = {"metrics": {}}
    if with_v:
        with_s = [r for r in reference if r["s"] is not None]
        s0 = with_s[0]["s"] if with_s else 0.0
        truth = [
            {
                "stamp_ns": str(r["stamp_ns"]),
                "v_mps": r["v"],
                "s_m": (r["s"] - s0) if r["s"] is not None else None,
                "route_id": r["route"],
            }
            for r in with_v
        ]
        with tempfile_estimates(output, args.bag, frames, truth) as (estimates_path, truth_path):
            report = evaluate(estimates_path, truth_path, output / f"{args.bag}-report.json", None)

    result = {
        "bag": bag.bag_id,
        "vehicle": bag.vehicle_id,
        "model_profile": str(selected),
        "duration_s": duration_s,
        "n_events": len(events),
        "n_ticks": len(frames),
        "requested_hz": args.hz,
        "achieved_hz": len(frames) / duration_s if duration_s else None,
        "wall_clock_compute_s": wall_elapsed_s,
        "compute_ms": {
            "p50": _percentile(timings, 0.50),
            "p95": _percentile(timings, 0.95),
            "p99": _percentile(timings, 0.99),
            "max": _percentile(timings, 1.0),
        },
        "valid_fraction": valid / len(frames) if frames else None,
        "model_only_fraction": model_only / len(frames) if frames else None,
        "reference_rows_with_v": len(with_v),
        "reference_coverage_of_bag": len(with_v) / len(reference) if reference else None,
        "rss_kb": {"before": rss_before_kb, "after_max": rss_after_kb},
        "counts": counts,
        "metrics_where_reference_available": report.get("metrics", {}),
        "crashed": False,
    }
    write_json(output / f"{args.bag}.json", result)
    print(json.dumps(result, indent=2))


def tempfile_estimates(output, bag_id, frames, truth):
    import contextlib
    import tempfile

    @contextlib.contextmanager
    def _ctx():
        with tempfile.TemporaryDirectory() as folder:
            estimates = Path(folder) / "e.jsonl"
            truth_path = Path(folder) / "t.jsonl"
            write_rows(estimates, frames)
            write_rows(truth_path, truth)
            yield estimates, truth_path

    return _ctx()


if __name__ == "__main__":
    main()
