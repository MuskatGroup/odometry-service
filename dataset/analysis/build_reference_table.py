"""Build the GNSS-derived reference table for every bag and split the bags for identification.

    python dataset/analysis/build_reference_table.py [--data dataset/data] [--out dataset/derived]

Writes ``<out>/reference/<bag>.csv`` (one row per 10 Hz GNSS epoch) and ``<out>/split.json``.
Bags are split *whole*; bags that are byte-for-byte the same recording are kept in one group so
identical data never lands in two splits. Wheel speed is stored as a feature column (km/h -> m/s),
never used as a target.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from bisect import bisect_right
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import gnss_reference as gr  # noqa: E402
import tram_bag  # noqa: E402
from odometry_geometry import PathGraph  # noqa: E402

COLUMNS = [
    "stamp_ns", "vehicle_id", "route_id", "s_ref", "v_ref", "a_ref", "controller",
    "front_wheel_mps", "rear_wheel_mps", "grade", "curvature", "reference_available",
    "reference_uncertainty", "reason",
]  # fmt: skip
WHEEL_KMH_TO_MPS = 1.0 / 3.6
HOLD_S = 0.3  # a sample older than this is not "current"


def _hold(stamps: list[int], values: list, t_ns: int, max_age_s: float = HOLD_S):
    i = bisect_right(stamps, t_ns) - 1
    if i < 0 or (t_ns - stamps[i]) / 1e9 > max_age_s:
        return None
    return values[i]


def bag_rows(bag: tram_bag.BagData, graph: PathGraph) -> list[dict]:
    fixes = [gr.GnssFix(*f) for f in bag.gnss_fix]
    velocities = [gr.GnssVelocity(*v) for v in bag.gnss_velocity]
    samples = gr.SpeedReferenceBuilder(graph).build(fixes, velocities)
    accelerations = gr.acceleration(samples)

    controls = sorted((c[0], c[2]) for c in bag.controls)
    ctl_t = [c[0] for c in controls]
    ctl_v = [c[1] for c in controls]
    wheels = {}
    for wheel_id in ("front", "rear"):
        rows = sorted((w[0], w[3] * WHEEL_KMH_TO_MPS) for w in bag.wheels if w[2] == wheel_id)
        wheels[wheel_id] = ([r[0] for r in rows], [r[1] for r in rows])

    out = []
    for sample, a_ref in zip(samples, accelerations, strict=True):
        grade = curvature = None
        if sample.route_id is not None and sample.s_ref_m is not None:
            try:
                pose = graph.pose_at(sample.route_id, sample.s_ref_m)
                grade, curvature = pose.grade, pose.curvature_inv_m
            except ValueError:
                pass
        control = _hold(ctl_t, ctl_v, sample.stamp_ns, max_age_s=1.0)
        out.append(
            {
                "stamp_ns": sample.stamp_ns,
                "vehicle_id": bag.vehicle_id,
                "route_id": sample.route_id,
                "s_ref": sample.s_ref_m,
                "v_ref": sample.v_ref_mps,
                "a_ref": a_ref,
                "controller": control,
                "front_wheel_mps": _hold(*wheels["front"], sample.stamp_ns),
                "rear_wheel_mps": _hold(*wheels["rear"], sample.stamp_ns),
                "grade": grade,
                "curvature": curvature,
                "reference_available": int(sample.v_ref_mps is not None),
                "reference_uncertainty": sample.uncertainty_mps,
                "reason": sample.reason,
            }
        )
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if v is None else v) for k, v in row.items()})


def fingerprint(bag: tram_bag.BagData) -> str:
    """Two bags with the same recording (same messages, different folder) share a fingerprint."""
    digest = hashlib.sha1()
    for stamps in (bag.wheels[:50], bag.controls[:50], bag.gnss_fix[:50]):
        digest.update(repr([row[0] for row in stamps]).encode())
    digest.update(repr(bag.topic_counts).encode())
    return digest.hexdigest()


def assign_splits(groups: dict[str, list[str]], usable: set[str], seed: str = "tram-odometry-v1") -> dict:
    """60/20/20 by group; the order is a stable hash so it does not depend on directory listing."""
    ranked = sorted(groups, key=lambda key: hashlib.sha1((seed + key).encode()).hexdigest())
    ids = [key for key in ranked if any(bag in usable for bag in groups[key])]
    n = len(ids)
    cut_id, cut_val = round(n * 0.6), round(n * 0.8)
    split = {"identification": [], "validation": [], "test": [], "unusable": []}
    for index, key in enumerate(ids):
        name = "identification" if index < cut_id else "validation" if index < cut_val else "test"
        split[name].extend(sorted(bag for bag in groups[key] if bag in usable))
    for key in ranked:
        split["unusable"].extend(sorted(bag for bag in groups[key] if bag not in usable))
    return {name: sorted(bags) for name, bags in split.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--data", type=Path, default=ROOT / "dataset/data")
    parser.add_argument("--out", type=Path, default=ROOT / "dataset/derived")
    parser.add_argument("--min-on-graph", type=float, default=0.3, help="min share of rows on a route")
    parser.add_argument("--min-seconds", type=float, default=120.0)
    args = parser.parse_args()

    graph = PathGraph.from_directory(ROOT / "dataset/Pathgraph")
    reference_dir = args.out / "reference"
    reference_dir.mkdir(parents=True, exist_ok=True)
    groups: dict[str, list[str]] = {}
    usable: set[str] = set()
    report = {}
    for bag_dir in sorted(p for p in args.data.iterdir() if p.is_dir()):
        bag = tram_bag.read_bag(bag_dir)
        groups.setdefault(fingerprint(bag), []).append(bag_dir.name)
        rows = bag_rows(bag, graph)
        write_csv(reference_dir / f"{bag_dir.name}.csv", rows)
        n = len(rows)
        available = sum(r["reference_available"] for r in rows)
        on_graph = sum(r["s_ref"] is not None and r["v_ref"] is not None for r in rows)
        seconds = (rows[-1]["stamp_ns"] - rows[0]["stamp_ns"]) / 1e9 if n > 1 else 0.0
        report[bag_dir.name] = {
            "rows": n,
            "seconds": round(seconds, 1),
            "reference_coverage": round(available / n, 4) if n else 0.0,
            "on_graph_share": round(on_graph / n, 4) if n else 0.0,
        }
        if n and on_graph / n >= args.min_on_graph and seconds >= args.min_seconds:
            usable.add(bag_dir.name)
        print(bag_dir.name, report[bag_dir.name], flush=True)

    split = assign_splits(groups, usable)
    split["duplicates"] = sorted(sorted(bags) for bags in groups.values() if len(bags) > 1)
    split["per_bag"] = report
    (args.out / "split.json").write_text(json.dumps(split, indent=1, ensure_ascii=False), encoding="utf-8")
    print({k: len(v) for k, v in split.items() if k != "per_bag"})


if __name__ == "__main__":
    main()
