"""Failure Lab on real organizer bags: GNSS-derived truth, injected wheel faults, GNSS masks.

Truth is *only* the GNSS-derived reference table written by ``dataset/analysis/build_reference_table.py``
(speed ``v_ref``, along-track ``s_ref``); wheel speed is never a target. Faults are injected into the
real wheel stream and known to the scorer as fault windows. Metrics that cannot be computed stay
``None`` (unavailable) instead of becoming 0.
"""

from __future__ import annotations

import csv
import dataclasses
import json
import sys
import tempfile
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

from odometry_core import (
    AlongTrackPositionCorrection,
    ControlSample,
    InitialState,
    LongitudinalVelocityCorrection,
    WheelSample,
)
from odometry_core.types import event_key

from .benchmark import PRESETS
from .evaluate import evaluate
from .runner import run_events
from .storage import write_json, write_rows

REPO = Path(__file__).resolve().parents[4]
NOTCHES = 15
WHEEL_KMH_TO_MPS = 1.0 / 3.6
NS = 1_000_000_000
GNSS_POSITION_VARIANCE_M2 = 4.0
GNSS_VELOCITY_SIGMA_FLOOR_MPS = 0.1

# name -> (fault kind, affected wheel channels); "none" is the clean run
SCENARIOS: dict[str, tuple[str, tuple[str, ...]]] = {
    "none": ("none", ()),
    "freeze-1": ("freeze", ("front",)),
    "freeze-2": ("freeze", ("front", "rear")),
    "dropout-1": ("dropout", ("front",)),
    "dropout-2": ("dropout", ("front", "rear")),
    "spike-1": ("spike", ("front",)),
    "slip-1": ("slip", ("front",)),
    "lock-1": ("lock", ("front",)),
}
GNSS_MODES = ("never", "initial", "intermittent-30", "intermittent-60", "intermittent-120")


@dataclass(frozen=True)
class Window:
    start_ns: int
    end_ns: int


@dataclass
class RealCase:
    bag_id: str
    vehicle_id: str
    window: Window
    events: list
    truth: list[dict]  # stamp_ns, v_mps, s_m (s relative to the window start)
    initial: InitialState
    reference_coverage: float
    route_id: str | None = None
    s0_m: float = 0.0  # absolute along-track position of the window start on route_id


def _tram_bag():
    sys.path.insert(0, str(REPO / "dataset/analysis"))
    import tram_bag

    return tram_bag


def load_reference(path: str | Path) -> list[dict]:
    rows = []
    with Path(path).open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append(
                {
                    "stamp_ns": int(row["stamp_ns"]),
                    "v": float(row["v_ref"]) if row["v_ref"] else None,
                    "s": float(row["s_ref"]) if row["s_ref"] else None,
                    "route": row.get("route_id") or None,
                    "velocity_sigma": (
                        float(row["reference_uncertainty"])
                        if row.get("reference_uncertainty")
                        else None
                    ),
                }
            )
    return rows


def choose_window(reference: list[dict], length_s: float, max_gap_s: float = 0.5) -> Window | None:
    """Most dynamic stretch of ``length_s`` where both v_ref and s_ref exist without long gaps."""
    good = [r for r in reference if r["v"] is not None and r["s"] is not None]
    spans, begin = [], 0
    for i in range(1, len(good) + 1):
        if i == len(good) or (good[i]["stamp_ns"] - good[i - 1]["stamp_ns"]) / NS > max_gap_s:
            spans.append(good[begin:i])
            begin = i
    best, best_score = None, -1.0
    step_ns = round(length_s / 2 * NS)
    for span in spans:
        if not span or (span[-1]["stamp_ns"] - span[0]["stamp_ns"]) / NS < length_s:
            continue
        start = span[0]["stamp_ns"]
        while start + length_s * NS <= span[-1]["stamp_ns"]:
            inside = [r for r in span if start <= r["stamp_ns"] <= start + length_s * NS]
            moving = sum(abs(r["v"]) > 1.0 for r in inside) / len(inside)
            score = moving * (max(r["v"] for r in inside) - min(r["v"] for r in inside))
            if score > best_score:
                best, best_score = Window(start, start + round(length_s * NS)), score
            start += step_ns
    return best


def events_from_bag(bag) -> list:
    """Organizer topics -> estimator events (km/h -> m/s, notch -> u in [-1, 1]), header-stamp order."""
    events: list = []
    for seq, (stamp, _record, wheel_id, raw) in enumerate(bag.wheels):
        events.append(WheelSample(stamp, seq, wheel_id, raw * WHEEL_KMH_TO_MPS))
    for seq, (stamp, _record, position) in enumerate(bag.controls, start=len(bag.wheels)):
        events.append(ControlSample(stamp, seq, max(-1.0, min(1.0, position / NOTCHES))))
    events.sort(key=event_key)
    return events


def build_case(bag_dir: str | Path, reference_csv: str | Path, window_s: float = 120.0) -> RealCase | None:
    bag = _tram_bag().read_bag(bag_dir)
    reference = load_reference(reference_csv)
    window = choose_window(reference, window_s)
    if window is None:
        return None
    in_window = [r for r in reference if window.start_ns <= r["stamp_ns"] <= window.end_ns]
    rows = [r for r in in_window if r["v"] is not None]
    with_s = [r for r in rows if r["s"] is not None]
    s0 = with_s[0]["s"]
    truth = [
        {
            "stamp_ns": str(r["stamp_ns"]),
            "v_mps": r["v"],
            "s_m": r["s"] - s0,
            "route_id": r["route"],
            "velocity_sigma_mps": r["velocity_sigma"],
        }
        for r in with_s
    ]
    events = [e for e in events_from_bag(bag) if window.start_ns <= e.stamp_ns <= window.end_ns]
    if not events:
        return None
    return RealCase(
        bag_id=bag.bag_id,
        vehicle_id=bag.vehicle_id,
        window=window,
        events=events,
        truth=truth,
        initial=InitialState(window.start_ns, 0.0, rows[0]["v"]),
        reference_coverage=len(rows) / max(1, len(in_window)),
        route_id=with_s[0]["route"],
        s0_m=s0,
    )


# --- fault injection -----------------------------------------------------------------------------


def inject(events: list, kind: str, channels: tuple[str, ...], start_ns: int, end_ns: int) -> list:
    """Return a copy of ``events`` with the fault applied to wheel channels inside [start, end)."""
    if kind == "none":
        return list(events)
    if kind not in {"freeze", "dropout", "spike", "slip", "lock"}:
        raise ValueError(f"Unknown fault: {kind}")
    out, frozen, counter = [], {}, {}
    for event in events:
        inside = start_ns <= event.stamp_ns < end_ns
        if not (isinstance(event, WheelSample) and event.wheel_id in channels):
            out.append(event)
            continue
        if not inside:
            frozen[event.wheel_id] = event.speed_mps
            out.append(event)
            continue
        if kind == "dropout":
            continue
        counter[event.wheel_id] = counter.get(event.wheel_id, 0) + 1
        speed = event.speed_mps
        if kind == "freeze":
            speed = frozen.setdefault(event.wheel_id, speed)
        elif kind == "lock":
            speed = 0.0
        elif kind == "slip":
            speed *= 1.5
        elif kind == "spike" and counter[event.wheel_id] % 5 == 0:
            speed += 6.0
        out.append(dataclasses.replace(event, speed_mps=speed))
    return out


def gnss_availability(stamps_ns: list[int], mode: str) -> list[bool]:
    """Which reference epochs a GNSS receiver would deliver in each test mode (5 s bursts if intermittent)."""
    if mode not in GNSS_MODES:
        raise ValueError(f"Unknown GNSS mode: {mode}")
    if not stamps_ns:
        return []
    origin = stamps_ns[0]
    out = []
    for stamp in stamps_ns:
        t = (stamp - origin) / NS
        if mode == "never":
            out.append(False)
        elif mode == "initial":
            out.append(t < 10.0)
        else:
            period = float(mode.rsplit("-", 1)[1])
            out.append(t < 10.0 or (t % period) < 5.0)
    return out


def correction_events(case: RealCase, mode: str) -> list:
    """Build GNSS-derived correction events without exposing the reference to wheel processing."""
    stamps = [int(row["stamp_ns"]) for row in case.truth]
    available = gnss_availability(stamps, mode)
    events = []
    for seq, (row, enabled) in enumerate(zip(case.truth, available, strict=True)):
        if not enabled:
            continue
        stamp = int(row["stamp_ns"])
        speed = row.get("v_mps")
        if speed is not None:
            sigma = max(
                GNSS_VELOCITY_SIGMA_FLOOR_MPS,
                float(row.get("velocity_sigma_mps") or GNSS_VELOCITY_SIGMA_FLOOR_MPS),
            )
            events.append(
                LongitudinalVelocityCorrection(stamp, seq, float(speed), sigma**2, "gnss-reference")
            )
        route_id = row.get("route_id") or case.route_id
        position = row.get("s_m")
        if route_id and position is not None:
            events.append(
                AlongTrackPositionCorrection(
                    stamp,
                    seq,
                    route_id,
                    float(position),
                    GNSS_POSITION_VARIANCE_M2,
                    "gnss-reference",
                )
            )
    return sorted(events, key=event_key)


def events_for_run(case: RealCase, scenario: str, start_ns: int, end_ns: int, gnss_mode: str) -> list:
    kind, channels = SCENARIOS[scenario]
    events = inject(case.events, kind, channels, start_ns, end_ns)
    events.extend(correction_events(case, gnss_mode))
    return sorted(events, key=event_key)


# --- scoring -----------------------------------------------------------------------------------


def recovery_time_s(frames: list[dict], truth: list[dict], fault_end_ns: int, tolerance=0.5, hold_s=1.0):
    """Seconds after the fault ends until |v - v_ref| stays below tolerance for hold_s; None if never."""
    times = [int(t["stamp_ns"]) for t in truth]
    if not times:
        return None
    ok_since = None
    for frame in frames:
        stamp = int(frame["stamp_ns"])
        if stamp < fault_end_ns or frame["v_mps"] is None:
            continue
        i = bisect_left(times, stamp)
        if i >= len(times):
            break
        good = abs(frame["v_mps"] - truth[i]["v_mps"]) < tolerance and frame["valid"]
        if not good:
            ok_since = None
        elif ok_since is None:
            ok_since = stamp
        if ok_since is not None and (stamp - ok_since) / NS >= hold_s:
            return (ok_since - fault_end_ns) / NS
    return None


def _percentile(values: list[float], q: float) -> float | None:
    values = sorted(v for v in values if v is not None)
    if not values:
        return None
    return values[min(len(values) - 1, round((len(values) - 1) * q))]


def correction_jump_m(frames: list[dict], corrections: list) -> float | None:
    """Largest position discontinuity unexplained by trapezoidal velocity integration."""
    position_stamps = {
        event.stamp_ns for event in corrections if isinstance(event, AlongTrackPositionCorrection)
    }
    if not position_stamps:
        return None
    jumps = []
    previous = None
    for frame in frames:
        stamp = int(frame["stamp_ns"])
        if previous is not None and stamp in position_stamps:
            dt = (stamp - int(previous["stamp_ns"])) / NS
            if (
                dt >= 0
                and frame.get("s_m") is not None
                and previous.get("s_m") is not None
                and frame.get("v_mps") is not None
                and previous.get("v_mps") is not None
            ):
                expected = 0.5 * (previous["v_mps"] + frame["v_mps"]) * dt
                jumps.append(abs((frame["s_m"] - previous["s_m"]) - expected))
        previous = frame
    return max(jumps, default=None)


def peak_rss_mb() -> float | None:
    try:
        import resource  # POSIX only
    except ImportError:
        return None
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def run_case(
    case: RealCase,
    scenario: str,
    preset: str,
    fault_start_s=50.0,
    fault_len_s=20.0,
    gnss_mode="never",
) -> dict:
    kind, channels = SCENARIOS[scenario]
    start = case.window.start_ns + round(fault_start_s * NS)
    end = start + round(fault_len_s * NS)
    estimator_name, model_config = PRESETS[preset]
    if gnss_mode != "never" and estimator_name != "adaptive-ekf":
        raise ValueError("GNSS correction modes require the adaptive-ekf estimator")
    corrections = correction_events(case, gnss_mode)
    events = events_for_run(case, scenario, start, end, gnss_mode)
    frames, counts = run_events(
        events, initial=case.initial, config=model_config, estimator_name=estimator_name, hz=50
    )
    faults = (
        []
        if kind == "none"
        else [
            {
                "type": kind,
                "start_ns": str(start),
                "end_ns": str(end),
                "affected_channels": list(channels),
            }
        ]
    )
    with tempfile.TemporaryDirectory() as folder:
        estimates, truth, fault_file = (Path(folder) / n for n in ("e.jsonl", "t.jsonl", "f.jsonl"))
        write_rows(estimates, frames)
        write_rows(truth, case.truth)
        write_rows(fault_file, faults)
        report = evaluate(estimates, truth, None, fault_file)
    metrics = report["metrics"]
    timings = [f["compute_ms"] for f in frames if f.get("compute_ms") is not None]
    model_only = sum(v for k, v in metrics["mode_fraction"].items() if "MODEL_ONLY" in k.upper())
    return {
        "bag": case.bag_id,
        "vehicle": case.vehicle_id,
        "scenario": scenario,
        "preset": preset,
        "gnss_mode": gnss_mode,
        "speed_rmse_mps": metrics["speed_rmse_mps"],
        "speed_mae_mps": metrics["speed_mae_mps"],
        "speed_bias_mps": metrics["speed_bias_mps"],
        "speed_p95_abs_mps": metrics["speed_p95_abs_mps"],
        "position_rmse_m": metrics["position_rmse_m"],
        "final_position_error_m": metrics["final_position_error_m"],
        "availability": metrics["valid_fraction"],
        "model_only_fraction": model_only,
        "recovery_s": recovery_time_s(frames, case.truth, end) if kind != "none" else None,
        "compute_p95_ms": _percentile(timings, 0.95),
        "compute_p99_ms": _percentile(timings, 0.99),
        "compute_max_ms": max(timings, default=None),
        "correction_jump_m": correction_jump_m(frames, corrections),
        "reference_coverage": case.reference_coverage,
        "counts": counts,
    }


def summarize(rows: list[dict]) -> list[dict]:
    """Mean over bags for each (scenario, preset, GNSS mode); missing metrics stay None."""
    keys = ("speed_rmse_mps", "speed_bias_mps", "speed_p95_abs_mps", "position_rmse_m",
            "final_position_error_m", "availability", "model_only_fraction", "recovery_s")  # fmt: skip
    grouped: dict[tuple[str, str, str], list[dict]] = {}
    for row in rows:
        grouped.setdefault((row["scenario"], row["preset"], row.get("gnss_mode", "never")), []).append(row)
    out = []
    for (scenario, preset, gnss_mode), items in grouped.items():
        entry = {"scenario": scenario, "preset": preset, "gnss_mode": gnss_mode, "bags": len(items)}
        for key in keys:
            values = [i[key] for i in items if i[key] is not None]
            entry[key] = sum(values) / len(values) if values else None
        out.append(entry)
    return out


def _fmt(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


def real_benchmark(
    bags: list[str],
    data_dir: str | Path,
    reference_dir: str | Path,
    output: str | Path,
    scenarios=tuple(SCENARIOS),
    presets=tuple(PRESETS),
    window_s: float = 120.0,
    gnss_modes=("never",),
) -> list[dict]:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows, skipped = [], []
    for bag in bags:
        case = build_case(Path(data_dir) / bag, Path(reference_dir) / f"{bag}.csv", window_s)
        if case is None:
            skipped.append(bag)
            continue
        for scenario in scenarios:
            for preset in presets:
                for gnss_mode in gnss_modes:
                    estimator_name, _ = PRESETS[preset]
                    if gnss_mode != "never" and estimator_name != "adaptive-ekf":
                        continue
                    rows.append(run_case(case, scenario, preset, gnss_mode=gnss_mode))
    summary = summarize(rows)
    write_json(
        output / "real-benchmark.json",
        {
            "window_s": window_s,
            "bags": [b for b in bags if b not in skipped],
            "skipped": skipped,
            "gnss_modes": list(gnss_modes),
            "summary": summary,
            "rows": rows,
        },
    )
    lines = [
        "| Scenario | Preset | GNSS | Bags | Speed RMSE | Bias | p95 | Path RMSE | Availability | Model-only | Recovery s |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for e in sorted(summary, key=lambda r: (r["scenario"], r["preset"], r["gnss_mode"])):
        lines.append(
            f"| {e['scenario']} | {e['preset']} | {e['gnss_mode']} | {e['bags']} | {_fmt(e['speed_rmse_mps'])} | "
            f"{_fmt(e['speed_bias_mps'])} | {_fmt(e['speed_p95_abs_mps'])} | {_fmt(e['position_rmse_m'], 2)} | "
            f"{_fmt(e['availability'])} | {_fmt(e['model_only_fraction'])} | {_fmt(e['recovery_s'], 1)} |"
        )
    (output / "real-benchmark.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return rows


def load_split(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def dedupe(bags: list[str], duplicate_groups: list[list[str]]) -> list[str]:
    """Keep one bag per group of identical recordings so duplicates do not weight the averages."""
    dropped = {bag for group in duplicate_groups for bag in sorted(group)[1:]}
    return [bag for bag in bags if bag not in dropped]
