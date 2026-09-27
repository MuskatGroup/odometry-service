"""Run odometry_lab.realdata.real_benchmark on real organizer bags, matching each bag to its
own identified vehicle model profile (configs/models/<vehicle_id>.yaml), which the CLI's single
global --model-profile flag cannot do when a subset mixes vehicles.

    python tools/real_fault_report.py --subset validation --scenario dropout-2 freeze-2 slip-1 lock-1 spike-1 --gnss-mode never --output artifacts/real-benchmark/faults

Writes <output>/<vehicle_id>/real-benchmark.json (and .md) per vehicle, same schema
odometry-lab real-benchmark writes, plus a combined <output>/combined.json with all rows.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from odometry_io import load_model_config
from odometry_lab.realdata import PRESETS, dedupe, load_split, real_benchmark

REPO = Path(__file__).resolve().parents[1]


def load_profile_config(path):
    config, _, _ = load_model_config(path, Path(path).stem)
    return config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="dataset/data")
    parser.add_argument("--derived", default="dataset/derived")
    parser.add_argument("--subset", choices=["identification", "validation", "test"], default="validation")
    parser.add_argument("--limit", type=int, default=None, help="Per-vehicle bag limit")
    parser.add_argument("--window", type=float, default=120.0)
    parser.add_argument("--scenario", nargs="+", default=["none"])
    parser.add_argument("--gnss-mode", nargs="+", default=["never"])
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    split = load_split(Path(args.derived) / "split.json")
    bags = dedupe(split[args.subset], split["duplicates"])
    by_vehicle: dict[str, list[str]] = {}
    for bag in bags:
        vehicle_id = bag.split("_", 1)[0]
        by_vehicle.setdefault(vehicle_id, []).append(bag)

    output_root = Path(args.output)
    output_root.mkdir(parents=True, exist_ok=True)
    combined_rows = []
    for vehicle_id, vehicle_bags in sorted(by_vehicle.items()):
        profile_path = REPO / "configs" / "models" / f"{vehicle_id}.yaml"
        if not profile_path.exists():
            print(f"skip {vehicle_id}: no profile at {profile_path}")
            continue
        PRESETS["M1-cal"] = ("adaptive-ekf", load_profile_config(profile_path))
        selected = vehicle_bags[: args.limit] if args.limit else vehicle_bags
        vehicle_output = output_root / vehicle_id
        rows = real_benchmark(
            selected,
            args.data,
            Path(args.derived) / "reference",
            vehicle_output,
            tuple(args.scenario),
            ("M1-cal",),
            args.window,
            tuple(args.gnss_mode),
        )
        combined_rows.extend(rows)
        print(f"{vehicle_id}: {len(selected)} bags -> {vehicle_output}")

    with open(output_root / "combined.json", "w") as handle:
        json.dump(combined_rows, handle, indent=2)
    print(f"combined: {len(combined_rows)} rows -> {output_root / 'combined.json'}")


if __name__ == "__main__":
    main()
