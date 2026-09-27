"""Compare paired real-benchmark reports; refuse silently different cohorts.

This verifies recorded bag/scenario/preset/GNSS keys and window lengths, not the
identity of unrecorded source data or code. Keep the experiment provenance too.
"""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean

KEYS = ("bag", "scenario", "preset", "gnss_mode")
METRICS = ("speed_rmse_mps", "position_rmse_m", "availability")


def _index(document):
    result = {}
    for row in document["rows"]:
        key = tuple(row.get(name, "never" if name == "gnss_mode" else None) for name in KEYS)
        if any(value is None for value in key) or key in result:
            raise ValueError(f"Missing or duplicate experiment key: {key}")
        for metric in METRICS:
            value = row.get(metric)
            if value is not None and (
                isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0
                or (metric == "availability" and value > 1)
            ):
                raise ValueError(f"Invalid {metric} in {key}")
        result[key] = row
    if not result:
        raise ValueError("Empty benchmark")
    return result


def compare(before, after):
    if before.get("window_s") is None or before["window_s"] != after.get("window_s"):
        raise ValueError("Different or missing window lengths")
    left, right = _index(before), _index(after)
    if left.keys() != right.keys():
        raise ValueError("Different experiment cohorts (bag/scenario/preset/GNSS)")
    groups = defaultdict(list)
    for key in left:
        groups[key[1:]].append((left[key], right[key]))
    result = []
    for (scenario, preset, gnss_mode), pairs in sorted(groups.items()):
        row = dict(scenario=scenario, preset=preset, gnss_mode=gnss_mode, bags=len(pairs))
        for metric in METRICS:
            # Never turn missing reference metrics into zeros or average unequal cohorts.
            values = [(a.get(metric), b.get(metric)) for a, b in pairs]
            usable = [(a, b) for a, b in values if a is not None and b is not None]
            row[metric] = {
                "pairs": len(usable),
                "before": mean(a for a, _ in usable) if usable else None,
                "after": mean(b for _, b in usable) if usable else None,
            }
        result.append(row)
    return result


def _fmt(value):
    return "n/a" if value is None else f"{value:.4f}"


def markdown(rows):
    lines = [
        "Mean per-bag RMSE; all finite estimates, including invalid intervals, remain in source metrics.",
        "Cohort keys/window lengths checked; source hashes and initial conditions need separate verification.",
        "",
        "| Scenario | Preset | GNSS | Bags | Speed RMSE m/s: before → after | Path RMSE m: before → after | Availability: before → after |",
        "|---|---|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        cells = []
        for metric in METRICS:
            item = row[metric]
            cell = f"{_fmt(item['before'])} → {_fmt(item['after'])}"
            if item["pairs"] != row["bags"]:
                cell += f" ({item['pairs']}/{row['bags']} pairs)"
            cells.append(cell)
        lines.append(
            f"| {row['scenario']} | {row['preset']} | {row['gnss_mode']} | {row['bags']} | "
            + " | ".join(cells) + " |"
        )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    args = parser.parse_args()
    try:
        before = json.loads(args.before.read_text(encoding="utf-8"))
        after = json.loads(args.after.read_text(encoding="utf-8"))
        print(markdown(compare(before, after)))
    except (ValueError, KeyError, OSError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
