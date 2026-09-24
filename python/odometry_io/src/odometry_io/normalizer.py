"""Explicit, non-executable source profiles shared by file and ROS adapters."""

import math
from collections import Counter
from decimal import Decimal, InvalidOperation

import yaml
from odometry_core.types import ControlSample, WheelSample


class ProfileError(ValueError):
    pass


def field(record, path, default=None):
    if not path:
        return default
    value = record
    try:
        for part in path.split("."):
            if isinstance(value, dict):
                value = value[part]
            elif isinstance(value, (list, tuple)):
                value = value[int(part)]
            else:
                value = getattr(value, part)
        return value
    except (KeyError, IndexError, AttributeError, TypeError, ValueError):
        return default


def boolean(value):
    if value is True or value == 1 or value == "true" or value == "1":
        return True
    if value is False or value == 0 or value == "false" or value == "0":
        return False
    raise ValueError("Expected a boolean validity flag")


def integer(value):
    decimal = Decimal(str(value))
    if not decimal.is_finite() or decimal != decimal.to_integral_value():
        raise ValueError("Expected finite integer")
    return int(decimal)


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Expected finite number")
    return number


def load_profile(path):
    with open(path, encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    Normalizer(profile)  # Configuration failures must happen before reading a source.
    return profile


class Normalizer:
    def __init__(self, profile):
        if not isinstance(profile, dict):
            raise ProfileError("Profile must be an object")
        self.profile = profile
        self.diagnostics = Counter()
        self.last_error = None
        self.sequence = 0
        time = profile.get("time", {})
        if not time.get("field") or time.get("unit") not in ("ns", "us", "ms", "s", "ros"):
            raise ProfileError("time.field and explicit time.unit are required")
        if not time.get("clock"):
            raise ProfileError("time.clock must identify the common time domain")
        if profile.get("layout") not in ("events", "wide"):
            raise ProfileError("layout must be events or wide")
        control = profile.get("control", {})
        if not control.get("field") or control.get("mapping") not in ("linear", "table"):
            raise ProfileError("control requires field and mapping: linear|table")
        if control["mapping"] == "table" and not control.get("values"):
            raise ProfileError("Discrete controller requires values")
        try:
            integer(time.get("offset_ns", 0))
            if control["mapping"] == "linear":
                finite(control.get("scale", 1))
                finite(control.get("offset", 0))
            else:
                if any(not -1 <= finite(v) <= 1 for v in control["values"].values()):
                    raise ValueError("Mapped controls must be in [-1,1]")
        except (ValueError, TypeError, InvalidOperation) as exc:
            raise ProfileError(str(exc)) from exc
        wheels = profile.get("wheels", [])
        if not wheels or len({w.get("id") for w in wheels}) != len(wheels):
            raise ProfileError("wheels must contain unique configured ids")
        for wheel in wheels:
            if not wheel.get("id") or not wheel.get("field"):
                raise ProfileError("Each wheel needs id and field")
            if wheel.get("unit") not in ("m/s", "km/h", "rad/s", "rpm"):
                raise ProfileError("Unsupported or missing wheel unit")
            if wheel["unit"] in ("rad/s", "rpm"):
                try:
                    if finite(wheel.get("radius_m")) <= 0:
                        raise ValueError("Nonpositive radius")
                except (TypeError, ValueError) as exc:
                    raise ProfileError("Angular wheel speed requires positive radius_m") from exc
        if profile["layout"] == "events" and (
            not profile.get("kind_field") or not profile.get("wheel_id_field")
        ):
            raise ProfileError("Event layout requires kind_field and wheel_id_field")

    def _stamp(self, record):
        config = self.profile["time"]
        raw = field(record, config["field"])
        if config["unit"] == "ros":
            seconds = integer(field(raw, "sec"))
            nanos = integer(field(raw, "nanosec"))
            if not 0 <= nanos < 1_000_000_000:
                raise ValueError("Invalid ROS nanoseconds")
            stamp = seconds * 1_000_000_000 + nanos
        else:
            factor = {"ns": 1, "us": 1000, "ms": 1_000_000, "s": 1_000_000_000}[config["unit"]]
            decimal = Decimal(str(raw)) * factor
            stamp = integer(decimal)
        stamp += integer(config.get("offset_ns", 0))
        if stamp < 0:
            raise ValueError("Negative normalized time")
        return stamp

    def _control(self, raw):
        config = self.profile["control"]
        if config["mapping"] == "table":
            values = {str(key): val for key, val in config["values"].items()}
            value = finite(values[str(raw)])
        else:
            value = finite(raw) * finite(config.get("scale", 1)) + finite(config.get("offset", 0))
        if not math.isfinite(value) or not -1 <= value <= 1:
            raise ValueError("Control outside [-1,1]")
        return value

    def _wheel(self, raw, config):
        value = finite(raw)
        unit = config["unit"]
        if unit == "km/h":
            value /= 3.6
        elif unit == "rad/s":
            value *= config["radius_m"]
        elif unit == "rpm":
            value *= 2 * math.pi * config["radius_m"] / 60
        if not math.isfinite(value) or value < 0:
            raise ValueError("Unsupported reverse/nonfinite wheel speed")
        return value

    def normalize(self, record):
        """One record -> zero or more domain events. Bad records are counted."""
        try:
            stamp = self._stamp(record)
            self.sequence += 1
            seq = integer(field(record, self.profile.get("seq_field"), self.sequence))
            if seq < 0:
                raise ValueError("Negative sequence")
            valid = boolean(field(record, self.profile.get("valid_field"), True))
            control_config = self.profile["control"]
            wheels = self.profile["wheels"]
            if self.profile["layout"] == "events":
                kind = field(record, self.profile["kind_field"])
                if kind == "control":
                    return [
                        ControlSample(
                            stamp, seq, self._control(field(record, control_config["field"])), valid
                        )
                    ]
                if kind != "wheel":
                    raise ValueError("Unknown event kind")
                wheel_id = str(field(record, self.profile["wheel_id_field"]))
                config = next((w for w in wheels if str(w["id"]) == wheel_id), None)
                if config is None:
                    raise ValueError("Unconfigured wheel id")
                return [
                    WheelSample(
                        stamp, seq, wheel_id, self._wheel(field(record, config["field"]), config), valid
                    )
                ]
            result = []
            raw = field(record, control_config["field"])
            if raw is not None and raw != "":
                result.append(ControlSample(stamp, seq, self._control(raw), valid))
            for config in wheels:
                raw = field(record, config["field"])
                if raw is not None and raw != "":
                    result.append(WheelSample(stamp, seq, str(config["id"]), self._wheel(raw, config), valid))
            if not result:
                raise ValueError("No measurements in row")
            return result
        except (ValueError, TypeError, KeyError, InvalidOperation, OverflowError) as exc:
            self.diagnostics["INVALID_RECORD"] += 1
            self.last_error = str(exc)
            return []
