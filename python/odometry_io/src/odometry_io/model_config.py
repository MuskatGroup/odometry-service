"""Strict loader for identified gray-box model parameters."""

from __future__ import annotations

import math
from pathlib import Path

import yaml
from odometry_core import DriveMap, ModelConfig


class ModelProfileError(ValueError):
    pass


TOP_LEVEL = {
    "schema_version",
    "model_version",
    "vehicle_id",
    "identified_at_utc",
    "identification_method",
    "identification_bags",
    "validation_bags",
    "longitudinal",
    "traction",
    "braking",
    "noise",
    "wheel_health",
    "metrics",
}
SECTIONS = {
    "longitudinal": {"tau_s", "c1_inv_s", "c2_inv_m", "disturbance_limit_mps2"},
    "traction": {"controller_u", "speed_mps", "acceleration_mps2"},
    "braking": {"controller_u", "speed_mps", "acceleration_mps2"},
    "noise": {"q_v", "q_a", "q_d", "wheel_variance_floor"},
    "wheel_health": {
        "gate_normal",
        "gate_reject",
        "max_wheel_accel_mps2",
        "freeze_s",
        "reacquire_s",
        "recover_updates",
    },
}


def _strict_section(document, name):
    value = document.get(name)
    if not isinstance(value, dict):
        raise ModelProfileError(f"{name} must be an object")
    unknown = set(value) - SECTIONS[name]
    if unknown:
        raise ModelProfileError(f"Unknown {name} fields: {sorted(unknown)}")
    missing = SECTIONS[name] - set(value)
    if missing:
        raise ModelProfileError(f"Missing {name} fields: {sorted(missing)}")
    return value


def _number(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ModelProfileError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ModelProfileError(f"{name} must be finite")
    return result


def _drive_map(section, name, expected_sign):
    controller = tuple(_number(v, f"{name}.controller_u") for v in section["controller_u"])
    speeds = tuple(_number(v, f"{name}.speed_mps") for v in section["speed_mps"])
    matrix = tuple(
        tuple(_number(v, f"{name}.acceleration_mps2") for v in row)
        for row in section["acceleration_mps2"]
    )
    if any(expected_sign * value <= 0 for value in controller):
        raise ModelProfileError(f"{name} controller axis has the wrong sign")
    if any(speed < 0 for speed in speeds):
        raise ModelProfileError(f"{name} speed axis cannot be negative")
    if any(expected_sign * value < 0 for row in matrix for value in row):
        raise ModelProfileError(f"{name} acceleration values have the wrong sign")
    try:
        return DriveMap(controller, speeds, matrix)
    except ValueError as exc:
        raise ModelProfileError(f"Invalid {name} map: {exc}") from exc


def resolve_model_profile(path, vehicle_id):
    root = Path(path)
    if root.is_file():
        return root
    if not root.is_dir():
        raise FileNotFoundError(f"Model profile path not found: {root}")
    requested = root / f"{vehicle_id}.yaml"
    fallback = root / "default.yaml"
    if requested.is_file():
        return requested
    if fallback.is_file():
        return fallback
    raise FileNotFoundError(f"Neither {requested.name} nor default.yaml exists in {root}")


def load_model_config(path, vehicle_id="default"):
    profile_path = resolve_model_profile(path, vehicle_id)
    document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    return model_config_from_document(document), document, profile_path


def model_config_from_document(document):
    """The single profile-to-model conversion used by runtime and offline evaluation."""
    if not isinstance(document, dict):
        raise ModelProfileError("Model profile must be an object")
    unknown = set(document) - TOP_LEVEL
    if unknown:
        raise ModelProfileError(f"Unknown model profile fields: {sorted(unknown)}")
    required = TOP_LEVEL - {"identified_at_utc", "metrics"}
    missing = required - set(document)
    if missing:
        raise ModelProfileError(f"Missing model profile fields: {sorted(missing)}")
    if document["schema_version"] != 1:
        raise ModelProfileError("Only model schema_version 1 is supported")
    if not isinstance(document["model_version"], str) or not document["model_version"]:
        raise ModelProfileError("model_version is required")
    if not isinstance(document["vehicle_id"], str) or not document["vehicle_id"]:
        raise ModelProfileError("vehicle_id is required")
    for name in ("identification_method",):
        if not isinstance(document[name], str) or not document[name]:
            raise ModelProfileError(f"{name} is required")
    for name in ("identification_bags", "validation_bags"):
        if not isinstance(document[name], list) or not all(isinstance(v, str) for v in document[name]):
            raise ModelProfileError(f"{name} must be a list of bag ids")

    longitudinal = _strict_section(document, "longitudinal")
    traction = _strict_section(document, "traction")
    braking = _strict_section(document, "braking")
    noise = _strict_section(document, "noise")
    health = _strict_section(document, "wheel_health")
    recover_updates = health["recover_updates"]
    if isinstance(recover_updates, bool) or not isinstance(recover_updates, int):
        raise ModelProfileError("wheel_health.recover_updates must be an integer")
    values = {
        "model_version": document["model_version"],
        "tau_s": _number(longitudinal["tau_s"], "longitudinal.tau_s"),
        "c1": _number(longitudinal["c1_inv_s"], "longitudinal.c1_inv_s"),
        "c2": _number(longitudinal["c2_inv_m"], "longitudinal.c2_inv_m"),
        "disturbance_limit_mps2": _number(
            longitudinal["disturbance_limit_mps2"], "longitudinal.disturbance_limit_mps2"
        ),
        "traction_map": _drive_map(traction, "traction", 1),
        "braking_map": _drive_map(braking, "braking", -1),
        "q_v": _number(noise["q_v"], "noise.q_v"),
        "q_a": _number(noise["q_a"], "noise.q_a"),
        "q_d": _number(noise["q_d"], "noise.q_d"),
        "wheel_variance_floor": _number(
            noise["wheel_variance_floor"], "noise.wheel_variance_floor"
        ),
        "gate_normal": _number(health["gate_normal"], "wheel_health.gate_normal"),
        "gate_reject": _number(health["gate_reject"], "wheel_health.gate_reject"),
        "max_wheel_accel": _number(
            health["max_wheel_accel_mps2"], "wheel_health.max_wheel_accel_mps2"
        ),
        "freeze_s": _number(health["freeze_s"], "wheel_health.freeze_s"),
        "reacquire_s": _number(health["reacquire_s"], "wheel_health.reacquire_s"),
        "recover_updates": recover_updates,
        "adapt_disturbance": True,
    }
    try:
        return ModelConfig(**values)
    except (TypeError, ValueError) as exc:
        raise ModelProfileError(str(exc)) from exc
