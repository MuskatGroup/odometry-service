import random
from pathlib import Path

from .storage import write_json, write_rows


def generate(output, seed=42, duration=20.0, fault="dropout", start=8.0, end=11.0):
    if duration <= 0 or not 0 <= start <= end <= duration:
        raise ValueError("Require 0 <= fault start <= end <= duration")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    events, truth = [], []
    velocity = distance = 0.0
    frozen = None
    dt = 0.02
    for index in range(round(duration / dt) + 1):
        t = index * dt
        u = 0.7 if t < duration * 0.4 else (0.0 if t < duration * 0.65 else -0.6)
        truth.append({"stamp_ns": str(index * 20_000_000), "v_mps": velocity, "s_m": distance})
        events.append(
            {"kind": "control", "stamp_ns": str(index * 20_000_000), "seq": index, "u": u, "valid": True}
        )
        failed = start <= t < end
        if failed and frozen is None:
            frozen = velocity
        for wheel in ("left", "right"):
            if failed and fault == "dropout":
                continue
            speed = max(0, velocity + rng.gauss(0, 0.025))
            if failed:
                if fault == "slip":
                    speed *= 1.5
                elif fault == "slide":
                    speed = 0
                elif fault == "freeze":
                    speed = frozen
            events.append(
                {
                    "kind": "wheel",
                    "stamp_ns": str(index * 20_000_000),
                    "seq": index,
                    "wheel_id": wheel,
                    "speed_mps": speed,
                    "valid": True,
                }
            )
        # Independent synthetic plant; not the wheel-hold estimator.
        acceleration = 1.4 * max(u, 0) - 2.0 * max(-u, 0) - 0.015 * velocity - 0.002 * velocity * velocity
        if failed and fault in ("slip", "slide"):
            acceleration *= 0.35  # Affect physical motion as well as wheel readings.
        next_v = max(0.0, velocity + acceleration * dt)
        distance += (velocity + next_v) * 0.5 * dt
        velocity = next_v
    write_rows(output / "events.jsonl", events)
    write_rows(output / "truth.jsonl", truth)
    write_json(
        output / "scenario.json",
        {
            "schema_version": "0.2",
            "synthetic": True,
            "seed": seed,
            "duration_s": duration,
            "fault": fault,
            "fault_start_s": start,
            "fault_end_s": end,
            "initial": {"stamp_ns": 0, "s_m": 0, "v_mps": 0},
        },
    )
