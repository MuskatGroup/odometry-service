import bisect
import math
from collections import Counter

from .storage import read_rows, write_json


def _percentile(values, quantile):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    left = math.floor(position)
    right = math.ceil(position)
    if left == right:
        return ordered[left]
    return ordered[left] + (ordered[right] - ordered[left]) * (position - left)


def evaluate(estimates_path, truth_path=None, output=None, faults_path=None):
    frames = read_rows(estimates_path)
    truth = read_rows(truth_path) if truth_path else []
    truth.sort(key=lambda row: int(row["stamp_ns"]))
    times = [int(row["stamp_ns"]) for row in truth]
    pairs = []
    for frame in frames:
        t = int(frame["stamp_ns"])
        if not times or t < times[0] or t > times[-1] or frame["v_mps"] is None:
            continue
        index = bisect.bisect_left(times, t)
        if times[index] == t:
            reference = truth[index]
        else:
            left, right = truth[index - 1], truth[index]
            ratio = (t - times[index - 1]) / (times[index] - times[index - 1])
            reference = {key: left[key] + ratio * (right[key] - left[key]) for key in ("v_mps", "s_m")}
        pairs.append((frame, reference))

    # Time-weighted trapezoidal errors; no extrapolation beyond available truth.
    def error_metric(key, power, valid_only=False):
        total = duration = 0.0
        for (a, ar), (b, br) in zip(pairs, pairs[1:]):
            if valid_only and not (a["valid"] and b["valid"]):
                continue
            dt = (int(b["stamp_ns"]) - int(a["stamp_ns"])) / 1e9
            if dt <= 0:
                continue
            total += dt * (abs(a[key] - ar[key]) ** power + abs(b[key] - br[key]) ** power) / 2
            duration += dt
        return (total / duration) ** (1 / power) if duration else None

    speed_errors = [frame["v_mps"] - reference["v_mps"] for frame, reference in pairs]
    position_errors = [frame["s_m"] - reference["s_m"] for frame, reference in pairs]
    # Criterion 2 ("Точность оценки положения") asks for accumulated drift at the end of the run
    # *relative to distance travelled* (%), not just an absolute metre figure: 5 m of error after
    # 50 m travelled is a very different result from 5 m after 5 km. Distance is the reference's
    # own along-track span over the matched window (truth, not our estimate, so a systematic scale
    # error in our output can't shrink its own denominator).
    distance_traveled_m = abs(pairs[-1][1]["s_m"] - pairs[0][1]["s_m"]) if pairs else None
    final_position_error_m = pairs[-1][0]["s_m"] - pairs[-1][1]["s_m"] if pairs else None
    final_drift_pct_of_distance = (
        abs(final_position_error_m) / distance_traveled_m * 100
        if final_position_error_m is not None and distance_traveled_m
        else None
    )
    durations = [(int(b["stamp_ns"]) - int(a["stamp_ns"])) / 1e9 for a, b in zip(frames, frames[1:])]
    duration = sum(durations)
    available = sum(dt for a, dt in zip(frames, durations) if a["valid"])
    incidents = []
    prior = None
    for frame in frames:
        if frame["mode"] != prior:
            incidents.append(
                {
                    "stamp_ns": frame["stamp_ns"],
                    "mode": frame["mode"],
                    "description": f"Режим: {prior or 'START'} → {frame['mode']}. "
                    f"Причины: {', '.join(frame['reason_codes'])}",
                }
            )
            prior = frame["mode"]
    mode_counts = Counter(frame["mode"] for frame in frames)
    sigma_pairs = [
        (abs(frame["v_mps"] - reference["v_mps"]), frame.get("sigma_v_mps"))
        for frame, reference in pairs
        if frame.get("sigma_v_mps") is not None
    ]
    faults = read_rows(faults_path) if faults_path else []
    fault_metrics = []
    for fault in faults:
        start = int(fault.get("start_ns", fault.get("stamp_ns", 0)))
        end = int(fault.get("end_ns", start))
        window = [(frame, reference) for frame, reference in pairs if start <= int(frame["stamp_ns"]) <= end]
        if not window:
            continue
        initial_error = window[0][0]["s_m"] - window[0][1]["s_m"]
        final_error = window[-1][0]["s_m"] - window[-1][1]["s_m"]
        fault_metrics.append(
            {
                "type": fault.get("type", "unknown"),
                "start_ns": str(start),
                "end_ns": str(end),
                "position_drift_m": final_error - initial_error,
                "max_speed_error_mps": max(abs(a["v_mps"] - b["v_mps"]) for a, b in window),
            }
        )
    report = {
        "schema_version": "0.2",
        "model_version": frames[-1].get("model_version", "unknown") if frames else "unknown",
        "metrics": {
            "speed_rmse_mps": error_metric("v_mps", 2),
            "speed_mae_mps": error_metric("v_mps", 1),
            "speed_rmse_valid_mps": error_metric("v_mps", 2, True),
            "speed_p95_abs_mps": _percentile([abs(value) for value in speed_errors], 0.95),
            "speed_max_abs_mps": max(map(abs, speed_errors), default=None),
            "speed_bias_mps": sum(speed_errors) / len(speed_errors) if speed_errors else None,
            "position_mae_m": error_metric("s_m", 1),
            "position_rmse_m": error_metric("s_m", 2),
            "position_p95_abs_m": _percentile([abs(value) for value in position_errors], 0.95),
            "position_max_abs_m": max(map(abs, position_errors), default=None),
            "final_position_error_m": final_position_error_m,
            "distance_traveled_m": distance_traveled_m,
            "final_drift_pct_of_distance": final_drift_pct_of_distance,
            "valid_fraction": available / duration if duration else None,
            "sample_count": len(frames),
            "truth_matched_count": len(pairs),
            "mode_fraction": {key: value / len(frames) for key, value in sorted(mode_counts.items())}
            if frames
            else {},
            "velocity_95pct_coverage": (
                sum(error <= 1.96 * sigma for error, sigma in sigma_pairs) / len(sigma_pairs)
                if sigma_pairs
                else None
            ),
        },
        "unavailable_reason": None if pairs else "NO_INDEPENDENT_TRUTH",
        "uncertainty_available": bool(sigma_pairs),
        "uncertainty_calibrated": False,
        "fault_windows": fault_metrics,
        "incidents": incidents,
        "truth": truth,
        "comparison": {"implemented": ["wheel-hold-v1", "ekf-robust-v1"]},
    }
    if output:
        write_json(output, report)
    return report
