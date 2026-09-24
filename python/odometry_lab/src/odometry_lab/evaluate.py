import bisect

from .storage import read_rows, write_json


def evaluate(estimates_path, truth_path=None, output=None):
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
    report = {
        "schema_version": "0.2",
        "model_version": "wheel-hold-v1",
        "metrics": {
            "speed_rmse_mps": error_metric("v_mps", 2),
            "speed_mae_mps": error_metric("v_mps", 1),
            "speed_rmse_valid_mps": error_metric("v_mps", 2, True),
            "position_mae_m": error_metric("s_m", 1),
            "final_position_error_m": pairs[-1][0]["s_m"] - pairs[-1][1]["s_m"] if pairs else None,
            "valid_fraction": available / duration if duration else None,
            "sample_count": len(frames),
            "truth_matched_count": len(pairs),
        },
        "unavailable_reason": None if pairs else "NO_INDEPENDENT_TRUTH",
        "uncertainty_available": False,
        "incidents": incidents,
        "truth": truth,
        "comparison": {"implemented": ["wheel-hold-v1"], "adaptive_model": "not_implemented"},
    }
    if output:
        write_json(output, report)
    return report
