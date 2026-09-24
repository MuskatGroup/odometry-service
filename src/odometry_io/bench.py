"""Scenario benchmark: docs/05 baselines B0/B1/B2 and M1/M2 on synthetic failure scenarios.

    python -m odometry_io.bench --output work/bench [--seeds 3] [--model-config params.json]

The simulator physics deliberately differs from the model defaults (gain, lag, drag), so the
numbers show robustness to an UNCALIBRATED model. They are synthetic results, not evidence of
accuracy on a real tram, and the criteria below are internal guidance from docs/05.
"""

import argparse
import json
from pathlib import Path
from statistics import median
import tempfile
from .core import InputError, Profile, normalize
from .emulator import fault_intervals, simulate
from .metrics import MEASUREMENT_FAULTS, compute_metrics, load_rows
from .model import ModelConfig, ModelEstimator
from .runner import run

STEP_S = 0.05
STEPS = 2400  # 120 s
CHANNELS = 4
PRESETS = {
    "B0": None,  # wheel + hold
    "B1": {"use_wheels": False, "max_model_only_s": 1e9},
    "B2": {"robust": False, "adapt_disturbance": False, "detect_freeze": False},
    "M1": {"adapt_disturbance": False},
    "M2": {"adapt_disturbance": True},
}


def steps(seconds):
    return round(seconds / STEP_S)


def window(kind, start_s, end_s, **extra):
    return {"type": kind, "start": steps(start_s), "end": steps(end_s), **extra}


# "holdout" changes the plant, timings, amounts and seeds so that parameter choices made on
# "train" can be checked on scenarios they were not tuned on.
VARIANTS = {
    "train": {"shift": 0.0, "strength": 1.0, "seed_offset": 0,
              "plant": {"initial_speed_mps": 3, "noise_mps": 0.05, "traction_gain": 1.5, "brake_gain": 1.3,
                        "drag_mps": 0.03, "actuator_tau_s": 0.4}, "cycle": (160, 280, 400, 0.4, -0.3)},
    "holdout": {"shift": 3.0, "strength": 0.8, "seed_offset": 100,
                "plant": {"initial_speed_mps": 5, "noise_mps": 0.03, "traction_gain": 1.1, "brake_gain": 1.7,
                          "drag_mps": 0.015, "actuator_tau_s": 0.15}, "cycle": (220, 340, 500, 0.6, -0.4)},
}


def scenarios(channels=CHANNELS, variant="train"):
    """name -> faults. Base physics is shared and differs from ModelConfig defaults."""
    v = VARIANTS[variant]
    shift, k = v["shift"], v["strength"]

    def w(kind, start, end, amount=None, channel=None):
        extra = {} if amount is None else {"amount": amount}
        if channel is not None:
            extra["channels"] = [channel % channels]
        return window(kind, start + shift, end + shift, **extra)

    def up(x):  # scale amounts weaken with strength
        return 1 + (x - 1) * k
    return {
        "clean": [],
        "slip_one": [w("scale", 20, 25, up(1.5), 0), w("scale", 60, 63, up(1.6), 1)],
        "slip_two": [w("scale", 20, 25, up(1.5), 0), w("scale", 20, 25, up(1.5), 1)],
        "common_slip": [w("scale", 20, 25, up(1.4))],
        "lock": [w("lock", 20, 23)],
        "dropouts": [w("dropout", 10, 11), w("dropout", 25, 30), w("dropout", 45, 60), w("dropout", 75, 105)],
        "freeze": [w("freeze", 20, 25), w("freeze", 45, 50, channel=0)],
        "load_change": [w("traction_scale", 40, 120, 0.6 if k >= 1 else 0.7)],
        "grade_in_dropout": [w("grade", 45, 65, 0.3 * k), w("dropout", 50, 60)],
        "grade_in_common_slip": [w("grade", 18, 30, 0.3 * k), w("scale", 20, 26, up(1.4))],
    }


def base_config(seed, channels=CHANNELS, variant="train"):
    v = VARIANTS[variant]
    coast_at, brake_at, cycle, drive, brake = v["cycle"]  # accelerate, coast, brake, repeat
    profile = [[0, drive]]
    for n in range(STEPS // cycle + 1):
        profile += [[n * cycle + coast_at, 0.0], [n * cycle + brake_at, brake], [(n + 1) * cycle, drive]]
    profile = sorted({p[0]: p for p in profile if p[0] < STEPS}.values())
    return {"seed": seed + v["seed_offset"], "steps": STEPS, "step_ns": "50000000", "channels": channels,
            "control_profile": profile, **v["plant"]}


def profile(channels=CHANNELS):
    return Profile({"schema_version": "adapter-1", "source_id": "bench", "timestamp": {"field": "stamp_ns", "unit": "ns"},
                    "seq_field": "seq", "control": {"field": "control"},
                    "channels": [{"id": f"w{i}", "field": f"speeds.{i}", "unit": "m/s", "missing": "skip"}
                                 for i in range(channels)]})


def run_one(config, preset, model_settings, workdir):
    prof = profile(config["channels"])
    pairs = list(simulate(config))
    truth = [t for _, t in pairs]
    faults = fault_intervals(config)
    estimator = None
    if PRESETS[preset] is not None:
        estimator = ModelEstimator(prof, ModelConfig.from_dict({**model_settings, **PRESETS[preset]}))
    out = Path(workdir) / "run"
    report = run(normalize((r for r, _ in pairs), prof), prof, out, estimator=estimator, output_hz=20)
    metrics = compute_metrics(load_rows(out / "estimates.jsonl"), truth, faults)
    for name in ("profile.json", "run.json", "normalized.jsonl", "estimates.jsonl", "report.json"):
        (out / name).unlink()
    out.rmdir()
    return {"metrics": metrics, "compute_ms": report["compute_ms"]}


def short_measurement_drifts(results):
    """|path drift| of every measurement fault lasting 1-5 s (docs/05 internal criterion 1)."""
    values = []
    for run_result in results:
        for e in run_result["metrics"]["faults"]["entries"]:
            if e["type"] in MEASUREMENT_FAULTS and 1 <= e["end_s"] - e["start_s"] <= 5 and e["path_drift_m"] is not None:
                values.append(abs(e["path_drift_m"]))
    return values


def summarize(table):
    """table[scenario][preset] -> list of run results across seeds."""
    rows = []
    for scenario, by_preset in table.items():
        for preset, runs in by_preset.items():
            m = [r["metrics"] for r in runs]
            pick = lambda f: [f(x) for x in m if f(x) is not None]
            rows.append({"scenario": scenario, "preset": preset, "seeds": len(runs),
                         "rmse_v_mps": median(pick(lambda x: x["speed_error_mps"]["rmse"])),
                         "rmse_s_m": median(pick(lambda x: x["path_error_m"]["rmse"])),
                         "final_abs_s_m": median(abs(x["path_error_m"]["final"]) for x in m),
                         "valid_fraction": median(pick(lambda x: x["availability"]["valid_fraction"])),
                         "coverage_speed": _median_or_none(pick(lambda x: x["coverage_1_96_sigma"]["overall"]["speed"])),
                         "p99_compute_ms": max(r["compute_ms"].get("p99", 0.0) for r in runs)})
    criteria = {}
    all_runs = {p: [r for by in table.values() for r in by[p]] for p in next(iter(table.values()))}
    for name in ("M1", "M2"):  # M1 (fixed d) is the default configuration, M2 the docs/01 hypothesis
        if "B0" not in all_runs or name not in all_runs:
            continue
        suffix = "" if name == "M2" else f"_{name}"
        b0, mx = short_measurement_drifts(all_runs["B0"]), short_measurement_drifts(all_runs[name])
        if b0 and mx:
            improvement = 1 - median(mx) / median(b0)
            criteria["short_fault_median_path_drift" + suffix] = {
                "B0_m": median(b0), f"{name}_m": median(mx), "improvement": improvement, "target": 0.20,
                "met": improvement >= 0.20, "faults_counted": len(b0)}
        clean = table.get("clean")
        if clean:
            r0 = median(r["metrics"]["speed_error_mps"]["rmse"] for r in clean["B0"])
            rx = median(r["metrics"]["speed_error_mps"]["rmse"] for r in clean[name])
            allowed = max(0.05 * r0, 0.02)
            criteria["clean_rmse_v_not_worse" + suffix] = {"B0": r0, name: rx, "allowed_increase": allowed,
                                                            "met": rx - r0 <= allowed}
    fault_types = {}
    for preset, runs in all_runs.items():
        per_type = {}
        for r in runs:
            for e in r["metrics"]["faults"]["entries"]:
                slot = per_type.setdefault(e["type"], {"drift": [], "detected": [], "delay": [], "recovery": []})
                if e["path_drift_m"] is not None:
                    slot["drift"].append(abs(e["path_drift_m"]))
                if e["type"] in MEASUREMENT_FAULTS:
                    slot["detected"].append(e["detect_delay_s"] is not None)
                    if e["detect_delay_s"] is not None:
                        slot["delay"].append(e["detect_delay_s"])
                    if e["recovery_s"] is not None:
                        slot["recovery"].append(e["recovery_s"])
        fault_types[preset] = {kind: {
            "median_abs_path_drift_m": _median_or_none(s["drift"]),
            "detected_fraction": sum(s["detected"]) / len(s["detected"]) if s["detected"] else None,
            "median_detect_delay_s": _median_or_none(s["delay"]),
            "median_recovery_s": _median_or_none(s["recovery"])} for kind, s in sorted(per_type.items())}
    criteria["by_fault_type"] = fault_types
    return rows, criteria


def _median_or_none(values):
    return median(values) if values else None


def markdown(rows, criteria, seeds, label=""):
    lines = [f"Synthetic scenarios ({label}), {seeds} seed(s), median across seeds. Model parameters are uncalibrated defaults.", "",
             "| scenario | preset | rmse v, m/s | rmse s, m | final |err s|, m | valid | cov v 95% | p99 ms |",
             "|---|---|---|---|---|---|---|---|"]
    fmt = lambda x, d=3: "-" if x is None else f"{x:.{d}f}"
    for r in rows:
        lines.append(f"| {r['scenario']} | {r['preset']} | {fmt(r['rmse_v_mps'])} | {fmt(r['rmse_s_m'])} | "
                     f"{fmt(r['final_abs_s_m'], 2)} | {fmt(r['valid_fraction'], 2)} | {fmt(r['coverage_speed'], 2)} | "
                     f"{fmt(r['p99_compute_ms'], 2)} |")
    types = criteria.get("by_fault_type", {})
    if types:
        kinds = sorted({k for by in types.values() for k in by})
        lines += ["", "Median |path drift| over each fault window, m (detected fraction in brackets for measurement faults):", "",
                  "| fault | " + " | ".join(types) + " |", "|---|" + "---|" * len(types)]
        for kind in kinds:
            cells = []
            for preset in types:
                s = types[preset].get(kind, {})
                d, det = s.get("median_abs_path_drift_m"), s.get("detected_fraction")
                cells.append(fmt(d, 2) + ("" if det is None or preset in ("B0", "B1") else f" ({det:.0%})"))
            lines.append(f"| {kind} | " + " | ".join(cells) + " |")
    lines += ["", "Internal criteria (docs/05, orientation only):"]
    for name, c in criteria.items():
        if name != "by_fault_type":
            lines.append(f"- {name}: {json.dumps(c, default=float)}")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--channels", type=int, default=CHANNELS, help="wheel channels of the simulated source (1..8)")
    parser.add_argument("--variant", choices=sorted(VARIANTS), default="train",
                        help="holdout = different plant, timings and seeds, to check parameter choices")
    parser.add_argument("--model-config", type=Path, help="JSON overrides applied to every model preset")
    parser.add_argument("--scenario", action="append", help="limit to these scenarios")
    parser.add_argument("--preset", action="append", choices=sorted(PRESETS))
    args = parser.parse_args(argv)
    if args.seeds < 1:
        raise InputError("--seeds must be >= 1")
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        raise InputError(f"Output directory must be empty: {args.output}")
    settings = json.loads(args.model_config.read_text(encoding="utf-8-sig")) if args.model_config else {}
    if not 1 <= args.channels <= 8:
        raise InputError("--channels must be between 1 and 8")
    chosen = {k: v for k, v in scenarios(args.channels, args.variant).items() if not args.scenario or k in args.scenario}
    presets = args.preset or list(PRESETS)
    table = {}
    with tempfile.TemporaryDirectory() as tmp:
        for name, faults in chosen.items():
            table[name] = {p: [] for p in presets}
            for seed in range(1, args.seeds + 1):
                config = {**base_config(seed, args.channels, args.variant), "faults": faults}
                for preset in presets:
                    table[name][preset].append(run_one(config, preset, settings, tmp))
    rows, criteria = summarize(table)
    text = markdown(rows, criteria, args.seeds, f"{args.variant}, {args.channels} channel(s)")
    (args.output / "bench.json").write_text(json.dumps({"rows": rows, "criteria": criteria, "seeds": args.seeds,
                                                        "model_settings": settings, "variant": args.variant,
                                                        "channels": args.channels}, indent=2, default=float),
                                            encoding="utf-8")
    (args.output / "bench.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
