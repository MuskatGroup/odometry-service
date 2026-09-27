"""Score recorded ROS outputs against a GNSS-derived CSV, without feeding truth to the node."""
from __future__ import annotations

import argparse
import csv
import json
import math
from bisect import bisect_left
from pathlib import Path

from odometry_geometry import normalize_route_id


def summarize(errors):
    return {
        "count": len(errors),
        "rmse": math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
        "mae": sum(abs(e) for e in errors) / len(errors) if errors else None,
        "bias": sum(errors) / len(errors) if errors else None,
        "max_abs": max(map(abs, errors), default=None),
        "final_error": errors[-1] if errors else None,
    }


def score(directory, reference_path, route_id=None, max_gap_s=0.5):
    """Interpolate only between valid, same-route reference epochs with a bounded gap."""
    with Path(reference_path).open() as handle:
        rows = list(csv.DictReader(handle))
    route_id = normalize_route_id(route_id)
    for row in rows:
        row["route_id"] = normalize_route_id(row.get("route_id") or None)
    rows.sort(key=lambda r: int(r["stamp_ns"]))
    times = [int(r["stamp_ns"]) for r in rows]

    def interpolate(stamp, field):
        i = bisect_left(times, stamp)
        if i < len(times) and times[i] == stamp:
            row = rows[i]
            if field == "s_ref" and row["route_id"] != route_id:
                return None
            return float(row[field]) if row[field] else None
        if i == 0 or i == len(times) or (times[i] - times[i - 1]) / 1e9 > max_gap_s:
            return None
        a, b = rows[i - 1], rows[i]
        if not a[field] or not b[field]:
            return None
        if field == "s_ref" and (a["route_id"] != route_id or b["route_id"] != route_id):
            return None
        fraction = (stamp - times[i - 1]) / (times[i] - times[i - 1])
        return float(a[field]) + fraction * (float(b[field]) - float(a[field]))

    velocity_errors, position_errors, valid_errors = [], [], []
    with (Path(directory) / "position.csv").open() as handle:
        map_stamps = {int(r["stamp_ns"]) for r in csv.DictReader(handle) if r["frame"] == "map"}
    with (Path(directory) / "estimates.csv").open() as handle:
        estimates = list(csv.DictReader(handle))
    for row in estimates:
        stamp = int(row["stamp_ns"])
        v = interpolate(stamp, "v_ref")
        if v is not None:
            error = float(row["v_mps"]) - v
            velocity_errors.append(error)
            if int(row["valid"]):
                valid_errors.append(error)
        if route_id and stamp in map_stamps:
            s = interpolate(stamp, "s_ref")
            if s is not None:
                position_errors.append(float(row["s_m"]) - s)
    return {
        "reference": str(reference_path), "route_id": route_id,
        "matching": f"linear interpolation, no extrapolation; reference gap <= {max_gap_s} s",
        "speed_all_mps": summarize(velocity_errors),
        "speed_valid_mps": summarize(valid_errors),
        "along_track_m": summarize(position_errors),
        "estimate_count": len(estimates),
        "valid_fraction": sum(int(r["valid"]) for r in estimates) / len(estimates) if estimates else None,
        "position_caveat": "Only matching map-frame outputs are scored; relative/pre-anchor outputs excluded",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--route-id")
    args = parser.parse_args()
    result = score(args.directory, args.reference, args.route_id)
    path = args.directory / "gnss-metrics.json"
    # Mode x protects previously measured artifacts.
    with path.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
