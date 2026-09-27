"""Calibrate EKF process noise and wheel gates on identification bags with injected faults.

Hand-picked noise made the robust gate reject healthy wheels on real bags (63-116 rejected samples in a
2-minute clean excerpt), so ``q_v``, ``q_a`` and the gate width are chosen by grid search over the
same real-data scenarios the lab scores: mean speed RMSE against the GNSS-derived reference. Only
identification bags are used here; validation and test bags stay untouched.
"""

from __future__ import annotations

import itertools
from pathlib import Path

import yaml
from odometry_core import ModelConfig
from odometry_io import build_model_config, load_model_config

from . import benchmark
from .realdata import build_case, run_case

GRID_Q_V = (0.05, 0.1, 0.3, 1.0)
GRID_Q_A = (0.3, 1.0, 3.0, 10.0)
GRID_GATE = (9.0, 25.0)  # gate_normal; gate_reject is 4x
OBJECTIVE_SCENARIOS = ("none", "freeze-1", "slip-1", "dropout-1", "spike-1")

# Kept as thin aliases over odometry_io's loader (the single source of truth for the model YAML
# schema): the Lab used to parse a subset of the same document by hand, silently dropping the
# traction/braking tables and adapt_disturbance, so a calibrated/benchmarked model was never the
# one runtime actually loads (organizer audit, 2026-09-27 -- fix/runtime-validation independently
# found and fixed the same gap under different names, kept as aliases in odometry_io).
config_from_profile = build_model_config


def load_profile_config(path: str | Path) -> ModelConfig:
    config, _document, _path = load_model_config(path, vehicle_id=Path(path).stem)
    return config


def score(config: ModelConfig, cases: list) -> float:
    benchmark.PRESETS["_calibration"] = ("adaptive-ekf", config)
    try:
        values = [
            run_case(case, scenario, "_calibration")["speed_rmse_mps"]
            for case in cases
            for scenario in OBJECTIVE_SCENARIOS
        ]
    finally:
        benchmark.PRESETS.pop("_calibration", None)
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else float("inf")


def calibrate(
    bags: list[str], data_dir, reference_dir, base: ModelConfig | None = None, window_s: float = 120.0
) -> dict:
    cases = [c for bag in bags if (c := build_case(Path(data_dir) / bag, Path(reference_dir) / f"{bag}.csv", window_s))]
    if not cases:
        raise ValueError("No usable identification bags")
    base = base or ModelConfig()
    results = []
    for q_v, q_a, gate in itertools.product(GRID_Q_V, GRID_Q_A, GRID_GATE):
        config = ModelConfig(
            **{**base.__dict__, "q_v": q_v, "q_a": q_a, "gate_normal": gate, "gate_reject": 4 * gate}
        )
        results.append({"q_v": q_v, "q_a": q_a, "gate_normal": gate, "objective_rmse_mps": score(config, cases)})
    results.sort(key=lambda r: r["objective_rmse_mps"])
    baseline = score(base, cases)
    return {"best": results[0], "baseline_objective_rmse_mps": baseline, "grid": results, "bags": len(cases)}


def apply_to_profile(path: str | Path, result: dict) -> None:
    """Write the chosen noise and gates into a model YAML (structure and other fields untouched)."""
    path = Path(path)
    profile = yaml.safe_load(path.read_text(encoding="utf-8"))
    best = result["best"]
    profile["noise"]["q_v"] = best["q_v"]
    profile["noise"]["q_a"] = best["q_a"]
    profile["wheel_health"]["gate_normal"] = best["gate_normal"]
    profile["wheel_health"]["gate_reject"] = 4 * best["gate_normal"]
    profile["metrics"]["noise_calibration"] = {
        "method": "grid_search_speed_rmse_with_injected_faults",
        "scenarios": list(OBJECTIVE_SCENARIOS),
        "calibration_bags": result["bags"],
        "objective_rmse_mps": round(best["objective_rmse_mps"], 4),
        "handpicked_objective_rmse_mps": round(result["baseline_objective_rmse_mps"], 4),
    }
    path.write_text(yaml.safe_dump(profile, sort_keys=False, allow_unicode=True), encoding="utf-8")
