"""Small deterministic synthetic source. It does not model a real tram.

Two fault levels (docs/09): measurement faults corrupt only the reported wheel speed;
physics faults (`grade`, `traction_scale`) change the true motion. Truth stays separate.
Legacy keys `dropout_steps` / `slip_steps` still work when no `faults` list is given.
"""

import json
import math
from pathlib import Path
import random
from .core import InputError, integer, number

MEASUREMENT_FAULTS = {"slip", "scale", "lock", "freeze", "dropout"}
PHYSICS_FAULTS = {"grade", "traction_scale"}


def _faults(config, count):
    """Normalized fault list: dicts with type, start, end (steps, end exclusive), channels, amount."""
    if "faults" in config:
        raw = config["faults"]
    else:
        raw = [{"type": "dropout", "start": iv[0], "end": iv[1]} for iv in [config.get("dropout_steps", [120, 150])]]
        raw += [{"type": "slip", "start": iv[0], "end": iv[1], "channels": [0], "amount": 3.0}
                for iv in [config.get("slip_steps", [220, 240])]]
        for interval in (config.get("dropout_steps", [120, 150]), config.get("slip_steps", [220, 240])):
            if not isinstance(interval, list) or len(interval) != 2 or any(type(n) is not int for n in interval) \
                    or not 0 <= interval[0] <= interval[1]:
                raise InputError("Fault intervals must be [first_step, exclusive_last_step]")
    if not isinstance(raw, list):
        raise InputError("faults must be a list")
    result = []
    for item in raw:
        kind = item.get("type")
        if kind not in MEASUREMENT_FAULTS | PHYSICS_FAULTS:
            raise InputError(f"Unknown fault type: {kind!r}")
        start, end = integer(item.get("start"), "fault start"), integer(item.get("end"), "fault end")
        if not 0 <= start <= end:
            raise InputError("Fault interval must satisfy 0 <= start <= end")
        channels = item.get("channels", "all")
        channels = list(range(count)) if channels == "all" else [integer(c, "fault channel") for c in channels]
        if any(not 0 <= c < count for c in channels):
            raise InputError("Fault channel out of range")
        default = {"slip": 3.0, "scale": 1.5, "grade": 0.3, "traction_scale": 0.6}.get(kind, 0.0)
        result.append({"type": kind, "start": start, "end": end, "channels": channels,
                       "amount": number(item.get("amount", default), "fault amount")})
    return result


def fault_intervals(config):
    """Ground-truth fault windows in ns for detector metrics (never given to the estimator)."""
    step_ns = integer(config.get("step_ns", "50000000"), "step_ns")
    count = integer(config.get("channels", 2), "channels")
    return [{"type": f["type"], "start_ns": str(f["start"] * step_ns), "end_ns": str(f["end"] * step_ns),
             "channels": f["channels"], "amount": f["amount"]} for f in _faults(config, count)]


def simulate(config):
    step_ns = integer(config.get("step_ns", "50000000"), "step_ns")
    steps = integer(config.get("steps", 400), "steps")
    count = integer(config.get("channels", 2), "channels")
    if step_ns <= 0 or not 1 <= steps <= 1000000 or not 1 <= count <= 256:
        raise InputError("Invalid emulator step/count/channel limits")
    seed = integer(config.get("seed", 1), "seed")
    rng = random.Random(seed)
    noise = number(config.get("noise_mps", 0.02), "noise_mps")
    speed = number(config.get("initial_speed_mps", 5), "initial_speed_mps")
    if noise < 0 or speed < 0:
        raise InputError("Emulator noise and initial speed must be nonnegative")
    gain = number(config.get("traction_gain", 1.8), "traction_gain")
    brake_gain = number(config.get("brake_gain", gain), "brake_gain")
    drag = number(config.get("drag_mps", 0.02), "drag_mps")
    tau = number(config.get("actuator_tau_s", 0.0), "actuator_tau_s")
    if gain <= 0 or brake_gain <= 0 or drag < 0 or tau < 0:
        raise InputError("Invalid emulator physics parameters")
    faults = _faults(config, count)
    profile = config.get("control_profile")
    if profile is not None:
        valid = isinstance(profile, list) and profile and all(isinstance(i, list) and len(i) == 2 for i in profile)
        if not valid or profile[0][0] != 0 \
                or any(profile[i][0] >= profile[i + 1][0] for i in range(len(profile) - 1)) \
                or any(not -1 <= number(item[1], "control_profile u") <= 1 for item in profile):
            raise InputError("control_profile must be [[0, u], [step, u], ...] with increasing steps and u in [-1, 1]")
    dt = step_ns / 1e9
    position = 0.0
    actuator = 0.0
    last_emitted, frozen = {}, {}
    for index in range(steps):
        stamp = index * step_ns
        if profile is None:
            control = 0.5 if index < steps // 3 else 0 if index < 2 * steps // 3 else -0.4
        else:
            control = [u for start, u in profile if start <= index][-1]
        # Truth is a separate result and is never forwarded to normalize/estimator.
        truth = {"stamp_ns": str(stamp), "v_mps": speed, "s_m": position}
        record = {"stamp_ns": str(stamp), "seq": index, "control": control, "speeds": {}}
        active = [(n, f) for n, f in enumerate(faults) if f["start"] <= index < f["end"]]
        for channel in range(count):
            if any(f["type"] == "dropout" and channel in f["channels"] for _, f in active):
                continue
            value = speed + rng.gauss(0, noise)
            for n, f in active:
                if channel not in f["channels"]:
                    continue
                if f["type"] == "slip":
                    value += f["amount"]
                elif f["type"] == "scale":
                    value *= f["amount"]
                elif f["type"] == "lock":
                    value = 0.0
                elif f["type"] == "freeze":
                    value = frozen.setdefault((n, channel), last_emitted.get(channel, value))
            value = max(0, value)
            last_emitted[channel] = value
            record["speeds"][str(channel)] = value
        yield record, truth
        scale = 1.0
        grade = 0.0
        for _, f in active:
            if f["type"] == "traction_scale":
                scale *= f["amount"]
            elif f["type"] == "grade":
                grade += f["amount"]
        command = control * (gain if control >= 0 else brake_gain) * scale
        actuator = command if tau == 0 else actuator + (command - actuator) * (1 - math.exp(-dt / tau))
        acceleration = actuator - drag * speed + grade
        new_speed = max(0, speed + acceleration * dt)
        position += (speed + new_speed) * dt / 2
        speed = new_speed


def read_emulator(path: Path):
    config = json.loads(path.read_text(encoding="utf-8-sig"))
    for record, _truth in simulate(config):
        yield record


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="Export synthetic inputs and independent truth for offline evaluation")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("Output directory must be empty")
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    with (args.output / "input.jsonl").open("w", encoding="utf-8") as inputs, \
         (args.output / "truth.jsonl").open("w", encoding="utf-8") as truth:
        for record, reference in simulate(config):
            inputs.write(json.dumps(record, allow_nan=False) + "\n")
            truth.write(json.dumps(reference, allow_nan=False) + "\n")
    (args.output / "faults.json").write_text(json.dumps(fault_intervals(config), indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
