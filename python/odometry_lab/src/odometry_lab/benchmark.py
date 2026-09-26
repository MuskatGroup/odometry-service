"""Reproducible synthetic ablation benchmark for the baseline and EKF variants."""

from __future__ import annotations

from pathlib import Path

from odometry_core import ModelConfig
from odometry_io import Normalizer, load_profile, read_file

from .evaluate import evaluate
from .runner import run_events
from .scenario import generate
from .storage import write_json, write_rows

SCENARIOS = (
    "none",
    "slip",
    "scale",
    "common_slip",
    "lock",
    "freeze",
    "dropout",
    "grade",
    "grade_in_dropout",
    "traction_scale",
)
PRESETS = {
    "B0": ("wheel-hold", None),
    "B1": ("adaptive-ekf", ModelConfig(use_wheels=False)),
    "B2": ("adaptive-ekf", ModelConfig(robust=False)),
    "M1": ("adaptive-ekf", ModelConfig()),
    "M2": ("adaptive-ekf", ModelConfig(adapt_disturbance=True)),
}


def benchmark(output, profile_path, seeds=(17,), scenarios=SCENARIOS, presets=tuple(PRESETS)):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=True)
    profile = load_profile(profile_path)
    rows = []
    for seed in seeds:
        for scenario in scenarios:
            source_dir = root / "sources" / f"{scenario}-seed-{seed}"
            generate(source_dir, seed=seed, duration=20, fault=scenario, start=8, end=13)
            events = read_file(source_dir / "events.jsonl", Normalizer(profile))
            for preset in presets:
                estimator_name, model_config = PRESETS[preset]
                frames, _ = run_events(events, config=model_config, estimator_name=estimator_name)
                run_dir = root / "runs" / f"{scenario}-seed-{seed}-{preset}"
                run_dir.mkdir(parents=True, exist_ok=True)
                estimates = run_dir / "estimates.jsonl"
                write_rows(estimates, frames)
                report = evaluate(
                    estimates,
                    source_dir / "truth.jsonl",
                    run_dir / "report.json",
                    source_dir / "faults.jsonl",
                )
                metrics = report["metrics"]
                rows.append(
                    {
                        "seed": seed,
                        "scenario": scenario,
                        "preset": preset,
                        "speed_rmse_mps": metrics["speed_rmse_mps"],
                        "final_position_error_m": metrics["final_position_error_m"],
                        "valid_fraction": metrics["valid_fraction"],
                    }
                )
    write_json(root / "benchmark.json", {"rows": rows, "presets": list(presets)})
    header = "| Scenario | Seed | Preset | Speed RMSE, m/s | Final path error, m | Availability |\n"
    separator = "|---|---:|---|---:|---:|---:|\n"
    body = "".join(
        f"| {row['scenario']} | {row['seed']} | {row['preset']} | "
        f"{row['speed_rmse_mps']:.3f} | {row['final_position_error_m']:.3f} | "
        f"{row['valid_fraction']:.3f} |\n"
        for row in rows
    )
    (root / "benchmark.md").write_text(header + separator + body, encoding="utf-8")
    return rows
