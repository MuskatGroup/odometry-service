"""Offline reference evaluation. Reference values never enter the estimator."""

import json
import math
from pathlib import Path
from .core import InputError, integer, number


def evaluate(estimates: Path, truth: Path):
    reference = {}
    with truth.open(encoding="utf-8-sig") as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            stamp = integer(row["stamp_ns"], "truth stamp_ns")
            if stamp in reference:
                raise InputError("Duplicate reference timestamp")
            reference[stamp] = (number(row["v_mps"], "truth speed"), number(row["s_m"], "truth position"))
    sq_v, sq_s, count, valid_count = 0.0, 0.0, 0, 0
    valid_sq_v = 0.0
    last_error = None
    total = 0
    with estimates.open(encoding="utf-8-sig") as stream:
        for line in stream:
            row = json.loads(line)
            total += 1
            stamp = integer(row["stamp_ns"], "estimate stamp_ns")
            if stamp not in reference or row["v_mps"] is None or row["s_m"] is None:
                continue
            v, s = reference[stamp]
            error_v = number(row["v_mps"], "estimated speed") - v
            last_error = number(row["s_m"], "estimated position") - s
            sq_v += error_v ** 2
            sq_s += last_error ** 2
            count += 1
            if row["valid"]:
                valid_count += 1
                valid_sq_v += error_v ** 2
    return {"alignment": "exact_timestamp_only", "matched_frames": count,
            "estimate_frames": total, "reference_frames": len(reference),
            "valid_matched_frames": valid_count,
            "rmse_v_mps": math.sqrt(sq_v / count) if count else None,
            "rmse_s_m": math.sqrt(sq_s / count) if count else None,
            "rmse_v_valid_mps": math.sqrt(valid_sq_v / valid_count) if valid_count else None,
            "last_matched_position_error_m": last_error,
            "reason": None if count else "No finite predictions at matching reference timestamps"}
