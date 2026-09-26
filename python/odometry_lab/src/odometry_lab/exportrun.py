"""Export one real-bag run in the console format (schema 0.3): estimates, map, reference, report.

Kept apart from ``realdata`` (scoring) so the console format can evolve without touching benchmarks.
The console only *reads* these files; it never runs the estimator.
"""

from __future__ import annotations

from pathlib import Path

from .benchmark import PRESETS
from .evaluate import evaluate
from .realdata import (
    NS,
    REPO,
    SCENARIOS,
    RealCase,
    _percentile,
    inject,
)
from .runner import run_events
from .storage import write_json, write_rows

SCHEMA_VERSION = "0.3"
MAX_TRACK_POINTS = 1500


def _decimate(rows: list, limit: int) -> list:
    if len(rows) <= limit:
        return rows
    step = len(rows) / limit
    return [rows[int(i * step)] for i in range(limit)] + [rows[-1]]


def map_block(case: RealCase, frames: list[dict], graph=None, step_m: float = 5.0) -> dict:
    """Pathgraph polylines plus reference/estimate tracks in continuous MGRS metres (frame ``map``)."""
    from odometry_geometry import OutOfGraphError, PathGraph

    graph = graph or PathGraph.from_directory(REPO / "dataset/Pathgraph")
    routes = []
    for route_id in graph.route_ids:
        length = graph.length_m(route_id)
        poses = [graph.pose_at(route_id, min(i * step_m, length)) for i in range(int(length // step_m) + 1)]
        routes.append({"route_id": route_id, "xy": [[round(p.x_m, 1), round(p.y_m, 1)] for p in poses]})

    def track(items: list[tuple[str, float]]) -> list[dict]:
        out = []
        for stamp, s_m in items:
            try:
                pose = graph.pose_at(case.route_id, case.s0_m + s_m)
            except OutOfGraphError:
                continue
            out.append({"stamp_ns": stamp, "x_m": round(pose.x_m, 2), "y_m": round(pose.y_m, 2)})
        return out

    has_route = case.route_id is not None
    reference = track([(t["stamp_ns"], t["s_m"]) for t in case.truth]) if has_route else []
    estimate = (
        track([(f["stamp_ns"], f["s_m"]) for f in frames if f["s_m"] is not None and f["valid"]])
        if has_route
        else []
    )
    return {
        "frame": "map: continuous MGRS 37UCB, x east, y north",
        "route_id": case.route_id,
        "routes": routes,
        "reference": _decimate(reference, MAX_TRACK_POINTS),
        "estimate": _decimate(estimate, MAX_TRACK_POINTS),
    }


def export_run(
    case: RealCase,
    scenario: str,
    preset: str,
    output: str | Path,
    run_id: str,
    fault_start_s: float = 50.0,
    fault_len_s: float = 20.0,
) -> dict:
    """Run one case; write run.json / estimates.jsonl / report.json that ``import-report`` uploads."""
    kind, channels = SCENARIOS[scenario]
    start = case.window.start_ns + round(fault_start_s * NS)
    end = start + round(fault_len_s * NS)
    events = inject(case.events, kind, channels, start, end)
    estimator_name, model_config = PRESETS[preset]
    frames, counts = run_events(
        events, initial=case.initial, config=model_config, estimator_name=estimator_name, hz=50
    )
    for frame in frames:
        frame.update(run_id=run_id, schema_version=SCHEMA_VERSION)
    faults = (
        []
        if kind == "none"
        else [{"type": kind, "start_ns": str(start), "end_ns": str(end), "affected_channels": list(channels)}]
    )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    write_rows(output / "estimates.jsonl", frames)
    write_rows(output / "truth.jsonl", case.truth)
    write_rows(output / "faults.jsonl", faults)
    report = evaluate(output / "estimates.jsonl", output / "truth.jsonl", None, output / "faults.jsonl")
    report["schema_version"] = SCHEMA_VERSION
    report["scenario"] = {
        "name": scenario,
        "fault": kind,
        "channels": list(channels),
        "bag": case.bag_id,
        "vehicle_id": case.vehicle_id,
        "preset": preset,
        "window_s": (case.window.end_ns - case.window.start_ns) / NS,
    }
    report["reference"] = {
        "source": "GNSS-derived (two receivers, dual-antenna base_link, Pathgraph)",
        "coverage": case.reference_coverage,
    }
    report["map"] = map_block(case, frames)
    report["corrections"] = {
        "available": False,
        "reason": "GNSS corrections are not implemented in the estimator core yet",
        "accepted": [],
        "rejected": [],
    }
    write_json(output / "report.json", report)
    timings = [f["compute_ms"] for f in frames if f.get("compute_ms") is not None]
    write_json(
        output / "run.json",
        {
            "run_id": run_id,
            "schema_version": SCHEMA_VERSION,
            "status": "completed",
            "source": f"bag:{case.bag_id}",
            "model_version": frames[-1]["model_version"],
            "synthetic": False,
            "scenario": scenario,
            "preset": preset,
            "diagnostics": counts,
            "compute_ms": {"p95": _percentile(timings, 0.95), "max": max(timings, default=None)},
        },
    )
    return report
