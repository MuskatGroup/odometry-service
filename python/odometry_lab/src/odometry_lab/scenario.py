import random
from pathlib import Path

from .storage import write_json, write_rows


def generate(output, seed=42, duration=20.0, fault="dropout", start=8.0, end=11.0, wheel_count=2):
    if duration <= 0 or not 0 <= start <= end <= duration:
        raise ValueError("Require 0 <= fault start <= end <= duration")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    events, truth = [], []
    if wheel_count < 1:
        raise ValueError("wheel_count must be positive")
    velocity = distance = actuator_acceleration = 0.0
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
        for wheel_index in range(wheel_count):
            wheel = ("left", "right")[wheel_index] if wheel_index < 2 else f"wheel_{wheel_index + 1}"
            if failed and fault in ("dropout", "grade_in_dropout"):
                continue
            speed = max(0, velocity + rng.gauss(0, 0.025))
            if failed:
                if fault == "slip":
                    # A subset slips so the channel voter has something independent to compare.
                    speed *= 1.5 if wheel_index < max(1, wheel_count // 2) else 1.0
                elif fault == "common_slip":
                    speed *= 1.4
                elif fault in ("slide", "lock"):
                    speed = 0
                elif fault == "freeze":
                    speed = frozen
                elif fault == "scale":
                    speed *= 1.2
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
        # Synthetic plant includes an actuator lag and physical changes not represented by wheel faults.
        traction_scale = 0.6 if failed and fault == "traction_scale" else 1.0
        commanded = traction_scale * (1.4 * max(u, 0) - 2.0 * max(-u, 0))
        actuator_acceleration += (commanded - actuator_acceleration) * (dt / 0.35)
        grade_acceleration = -0.35 if failed and fault in ("grade", "grade_in_dropout") else 0.0
        acceleration = (
            actuator_acceleration + grade_acceleration - 0.015 * velocity - 0.002 * velocity * velocity
        )
        next_v = max(0.0, velocity + acceleration * dt)
        distance += (velocity + next_v) * 0.5 * dt
        velocity = next_v
    write_rows(output / "events.jsonl", events)
    write_rows(output / "truth.jsonl", truth)
    write_rows(
        output / "faults.jsonl",
        []
        if fault == "none"
        else [
            {
                "type": fault,
                "start_ns": str(round(start * 1e9)),
                "end_ns": str(round(end * 1e9)),
                "affected_channels": (
                    [
                        (("left", "right")[index] if index < 2 else f"wheel_{index + 1}")
                        for index in range(max(1, wheel_count // 2))
                    ]
                    if fault == "slip"
                    else ["all"]
                ),
                "physical": fault in ("grade", "grade_in_dropout", "traction_scale"),
            }
        ],
    )
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
            "wheel_count": wheel_count,
            "physics": {"actuator_tau_s": 0.35, "traction_gain": 1.4, "brake_gain": 2.0},
            "initial": {"stamp_ns": 0, "s_m": 0, "v_mps": 0},
        },
    )
