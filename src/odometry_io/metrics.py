"""Metrics from docs/05: interpolated reference, drift on faults, horizons, availability,
detector quality and sigma coverage. Reference and fault windows never reach the estimator.

Alignment: reference is linearly interpolated at estimate stamps (estimates sit on one uniform
grid, so plain means are time-weighted); frames outside the reference range are skipped and
counted. The path origin is aligned ONCE at the first finite estimate, identically for every
compared algorithm, and never re-anchored per fault.
"""

import bisect
import json
import math
from pathlib import Path
from statistics import median
from .core import InputError, integer, number

HORIZONS_S = (1, 5, 15, 30)
MEASUREMENT_FAULTS = {"slip", "scale", "lock", "freeze", "dropout"}
FLAG_REASONS = {"WHEEL_REJECTED", "WHEEL_DOWNWEIGHTED", "WHEEL_STALE", "WHEEL_FROZEN"}


def load_rows(path: Path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        return [json.loads(line) for line in stream if line.strip()]


class Reference:
    def __init__(self, rows):
        pairs = sorted((integer(r["stamp_ns"], "truth stamp_ns"), number(r["v_mps"], "truth v"),
                        number(r["s_m"], "truth s")) for r in rows)
        stamps = [p[0] for p in pairs]
        if len(set(stamps)) != len(stamps):
            raise InputError("Duplicate reference timestamp")
        self.t, self.v, self.s = stamps, [p[1] for p in pairs], [p[2] for p in pairs]

    def at(self, ns):
        if not self.t or ns < self.t[0] or ns > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, ns)
        if self.t[i] == ns:
            return self.v[i], self.s[i]
        f = (ns - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return (self.v[i - 1] + f * (self.v[i] - self.v[i - 1]), self.s[i - 1] + f * (self.s[i] - self.s[i - 1]))


def _stats(errors):
    if not errors:
        return {"n": 0, "mae": None, "rmse": None, "p95_abs": None, "max_abs": None, "bias": None}
    ordered = sorted(abs(e) for e in errors)
    n = len(errors)
    return {"n": n, "mae": sum(ordered) / n, "rmse": math.sqrt(sum(e * e for e in errors) / n),
            "p95_abs": ordered[math.ceil(0.95 * n) - 1], "max_abs": ordered[-1], "bias": sum(errors) / n}


def _flagged(row):
    # Baselines without a wheel-trust detector (BASELINE_HOLD) never flag anything except staleness.
    return row["mode"] in ("DEGRADED", "MODEL_ONLY", "INVALID") or bool(FLAG_REASONS & set(row.get("reason_codes", [])))


def compute_metrics(estimates, truth, faults=None, horizons=HORIZONS_S):
    ref = Reference(truth)
    frames = []  # (stamp, row, v_err, s_err) for frames with an estimate inside the reference range
    outside = no_estimate = 0
    offset = None
    modes, valid_count, total = {}, 0, 0
    for row in estimates:
        total += 1
        modes[row["mode"]] = modes.get(row["mode"], 0) + 1
        valid_count += bool(row["valid"])
        if row["v_mps"] is None or row["s_m"] is None:
            no_estimate += 1
            continue
        stamp = integer(row["stamp_ns"], "estimate stamp_ns")
        truth_at = ref.at(stamp)
        if truth_at is None:
            outside += 1
            continue
        if offset is None:
            offset = row["s_m"] - truth_at[1]
        frames.append((stamp, row, row["v_mps"] - truth_at[0], row["s_m"] - offset - truth_at[1]))
    result = {
        "alignment": "linear_interpolation_origin_aligned_once",
        "frames": {"total": total, "matched": len(frames), "no_estimate": no_estimate, "outside_reference": outside},
        "origin_offset_m": offset,
        "speed_error_mps": _stats([f[2] for f in frames]),
        "speed_error_valid_mps": _stats([f[2] for f in frames if f[1]["valid"]]),
        "path_error_m": {**_stats([f[3] for f in frames]), "final": frames[-1][3] if frames else None},
        "availability": {"valid_fraction": valid_count / total if total else None,
                         "no_estimate_fraction": no_estimate / total if total else None,
                         "mode_fraction": {k: v / total for k, v in sorted(modes.items())}},
        "coverage_1_96_sigma": _coverage(frames),
    }
    if faults is not None:
        result["faults"] = _fault_metrics(frames, ref, faults, horizons)
    return result


def _coverage(frames):
    def block(rows):
        v = [abs(f[2]) <= 1.96 * f[1]["sigma_v_mps"] for f in rows if f[1].get("sigma_v_mps") is not None]
        s = [abs(f[3]) <= 1.96 * f[1]["sigma_s_m"] for f in rows if f[1].get("sigma_s_m") is not None]
        return {"speed": sum(v) / len(v) if v else None, "path": sum(s) / len(s) if s else None, "n": len(v)}
    by_mode = {}
    for f in frames:
        by_mode.setdefault(f[1]["mode"], []).append(f)
    return {"overall": block(frames), "by_mode": {m: block(rows) for m, rows in sorted(by_mode.items())},
            "note": "nominal 95% under a Gaussian assumption; not a proven interval"}


def _fault_metrics(frames, ref, faults, horizons):
    stamps = [f[0] for f in frames]

    def first_at_or_after(ns):
        i = bisect.bisect_left(stamps, ns)
        return frames[i] if i < len(frames) else None

    def last_at_or_before(ns):
        i = bisect.bisect_right(stamps, ns) - 1
        return frames[i] if i >= 0 else None

    def increment(a, b):
        return (b[1]["s_m"] - a[1]["s_m"]) - (ref.at(b[0])[1] - ref.at(a[0])[1])

    windows = [(f["type"], integer(f["start_ns"], "start_ns"), integer(f["end_ns"], "end_ns")) for f in faults]
    measurement = [(a, b) for kind, a, b in windows if kind in MEASUREMENT_FAULTS]
    entries = []
    for kind, a, b in windows:
        first, last = first_at_or_after(a), last_at_or_before(b)
        entry = {"type": kind, "start_s": a / 1e9, "end_s": b / 1e9, "path_drift_m": None,
                 "rmse_v_mps": None, "detect_delay_s": None, "recovery_s": None}
        if first and last and first[0] < last[0]:
            entry["path_drift_m"] = increment(first, last)
            inside = [f[2] for f in frames if a <= f[0] <= b]
            entry["rmse_v_mps"] = math.sqrt(sum(e * e for e in inside) / len(inside)) if inside else None
        if kind in MEASUREMENT_FAULTS:
            hit = next((f for f in frames if a <= f[0] < b and _flagged(f[1])), None)
            entry["detect_delay_s"] = (hit[0] - a) / 1e9 if hit else None
            fused = next((f for f in frames if f[0] >= b and f[1]["mode"] == "FUSED"), None)
            entry["recovery_s"] = (fused[0] - b) / 1e9 if fused else None
        if kind == "dropout" and first:
            entry["horizons"] = {}
            for h in horizons:
                if a + h * 1_000_000_000 <= b:
                    at = first_at_or_after(a + h * 1_000_000_000)
                    if at and first[0] < at[0]:
                        entry["horizons"][str(h)] = {"speed_abs_error_mps": abs(at[2]),
                                                     "path_drift_m": increment(first, at)}
        entries.append(entry)
    summary = {}
    for kind in sorted({e["type"] for e in entries}):
        group = [e for e in entries if e["type"] == kind]
        drifts = [abs(e["path_drift_m"]) for e in group if e["path_drift_m"] is not None]
        delays = [e["detect_delay_s"] for e in group if e["detect_delay_s"] is not None]
        summary[kind] = {"count": len(group),
                         "median_abs_path_drift_m": median(drifts) if drifts else None,
                         "detected_fraction": len(delays) / len(group) if kind in MEASUREMENT_FAULTS else None,
                         "mean_detect_delay_s": sum(delays) / len(delays) if delays else None}
    flagged = [f for f in frames if _flagged(f[1])]
    in_window = lambda ns: any(a <= ns < b for a, b in measurement)
    positives = [f for f in frames if in_window(f[0])]
    tp = [f for f in flagged if in_window(f[0])]
    detector = {"precision": len(tp) / len(flagged) if flagged else None,
                "recall": len(tp) / len(positives) if positives else None,
                "flagged_frames": len(flagged), "fault_frames": len(positives)}
    return {"entries": entries, "summary": summary, "detector": detector}
