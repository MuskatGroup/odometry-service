"""Explicit field/unit/clock conversion. No ROS or truth enters this layer."""

from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation
import math
from typing import Any, Iterable


class InputError(ValueError):
    pass


def number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise InputError(f"{label}: boolean is not a number")
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as exc:
        raise InputError(f"{label}: expected a finite number") from exc
    if not math.isfinite(result):
        raise InputError(f"{label}: expected a finite number")
    return result


def integer(value: Any, label: str) -> int:
    if isinstance(value, (bool, float)):
        raise InputError(f"{label}: use an integer or decimal string, not float")
    try:
        value_d = Decimal(str(value))
        if not value_d.is_finite() or value_d != value_d.to_integral_value():
            raise InputError(f"{label}: expected an integer")
        return int(value_d)
    except (InvalidOperation, ValueError, TypeError) as exc:
        raise InputError(f"{label}: expected an integer") from exc


def field(record: Any, path: str) -> Any:
    value = record
    try:
        for part in path.split("."):
            value = value[int(part)] if isinstance(value, list) else value[part]
        return value
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise InputError(f"Missing field: {path}") from exc


def flag(value: Any) -> bool:
    if value is True or value == "true" or value == "1" or type(value) is int and value == 1:
        return True
    if value is False or value == "false" or value == "0" or type(value) is int and value == 0:
        return False
    raise InputError(f"Unsupported validity flag: {value!r}")


@dataclass(frozen=True)
class Event:
    stamp_ns: int
    source_id: str
    seq: int
    kind: str
    channel_id: str | None
    value: float | None
    input_valid: bool

    def __post_init__(self):
        if type(self.stamp_ns) is not int or type(self.seq) is not int or self.seq < 0:
            raise InputError("Invalid event timestamp/sequence")
        if not isinstance(self.source_id, str) or not self.source_id:
            raise InputError("source_id is required")
        if self.kind not in ("control", "speed") or type(self.input_valid) is not bool:
            raise InputError("Invalid event kind/validity")
        if self.kind == "speed" and (not isinstance(self.channel_id, str) or not self.channel_id):
            raise InputError("Speed event needs channel_id")
        if self.kind == "control" and self.channel_id is not None:
            raise InputError("Control event must not have channel_id")
        if self.value is None:
            if self.input_valid:
                raise InputError("Valid input cannot be null")
        else:
            value = number(self.value, "event value")
            if self.kind == "control" and not -1 <= value <= 1:
                raise InputError("Normalized control must be in [-1, 1]")

    @property
    def key(self):
        return self.source_id, self.kind, self.channel_id, self.seq

    def to_dict(self):
        result = asdict(self)
        result["stamp_ns"] = str(self.stamp_ns)
        return result

    @classmethod
    def from_dict(cls, record):
        return cls(integer(record["stamp_ns"], "stamp_ns"), record["source_id"],
                   integer(record["seq"], "seq"), record["kind"], record["channel_id"],
                   None if record["value"] is None else number(record["value"], "value"), record["input_valid"])


class Profile:
    """Profile schema is intentionally small; unusual payloads use a custom adapter."""

    TIME_UNITS = {"s": 1_000_000_000, "ms": 1_000_000, "us": 1000, "ns": 1}
    SPEED_UNITS = {"m/s", "km/h", "rad/s", "rpm"}

    def __init__(self, config: dict):
        self.config = config
        if config.get("schema_version") != "adapter-1":
            raise InputError("Expected profile schema_version=adapter-1")
        self.source_id = config.get("source_id")
        if not isinstance(self.source_id, str) or not self.source_id:
            raise InputError("Profile needs source_id")
        self.time = config.get("timestamp", {})
        if self.time.get("unit") not in self.TIME_UNITS or not self.time.get("field"):
            raise InputError("Timestamp needs a field and explicit unit: s/ms/us/ns")
        self.offset_ns = integer(self.time.get("offset_ns", 0), "offset_ns")
        self.channels = config.get("channels", [])
        if not isinstance(self.channels, list) or (not self.channels and not config.get("control")):
            raise InputError("At least one configured speed channel is required")
        ids = [item.get("id") for item in self.channels]
        if any(not isinstance(i, str) or not i for i in ids) or len(set(ids)) != len(ids):
            raise InputError("Channel IDs must be nonempty and unique")
        for item in self.channels:
            if not item.get("field") or item.get("unit") not in self.SPEED_UNITS:
                raise InputError(f"Channel {item['id']}: field and known unit required")
            if item["unit"] in ("rad/s", "rpm") and number(item.get("radius_m"), "radius_m") <= 0:
                raise InputError("Angular speed requires radius_m > 0")
            if item.get("missing", "error") not in ("error", "skip"):
                raise InputError("missing must be error or skip")
            for dependency in item.get("derived_from", []):
                if dependency not in ids or dependency == item["id"]:
                    raise InputError("derived_from must reference other configured channels")
        self.control = config.get("control")
        if self.control and not self.control.get("field"):
            raise InputError("Control needs a field")
        self.fusion = config.get("fusion", {"strategy": "median", "variance_floor": 0.25})
        if number(self.fusion.get("variance_floor", 0.25), "variance_floor") <= 0:
            raise InputError("variance_floor must be positive")
        selected = self.fusion.get("channels", ids)
        if (ids or "channels" in self.fusion) and (
                not selected or len(set(selected)) != len(selected) or not set(selected) <= set(ids)):
            raise InputError("fusion.channels must be a nonempty unique subset")
        for item in self.channels:
            if item["id"] in selected and set(item.get("derived_from", [])) & set(selected):
                raise InputError("Select a derived aggregate OR its input channels, not both")
        self.selected = set(selected)

    def timestamp(self, value) -> int:
        if isinstance(value, bool):
            raise InputError("Boolean timestamp")
        # JSON decoder preserves fractional values as Decimal; floats in custom readers
        # may already have lost nanoseconds and are intentionally rejected.
        if isinstance(value, float):
            raise InputError("Timestamp must be integer/string/Decimal, not float")
        try:
            ns = Decimal(str(value)) * self.TIME_UNITS[self.time["unit"]]
            if not ns.is_finite() or ns != ns.to_integral_value():
                raise InputError("Timestamp does not resolve to whole nanoseconds")
            return int(ns) + self.offset_ns
        except (InvalidOperation, ValueError) as exc:
            raise InputError("Invalid timestamp") from exc


def normalize(records: Iterable[dict], profile: Profile, counters: dict | None = None) -> Iterable[list[Event]]:
    """One input record yields one atomic delivery; no fabricated missing samples.

    Pass a shared `counters` dict when records arrive one call at a time (live
    transports) so generated sequence numbers keep growing across calls.
    """
    counters = {} if counters is None else counters
    for record in records:
        stamp = profile.timestamp(field(record, profile.time["field"]))
        batch = []
        specs = [("speed", spec) for spec in profile.channels]
        if profile.control:
            specs.insert(0, ("control", profile.control))
        for kind, spec in specs:
            try:
                raw = field(record, spec["field"])
            except InputError:
                if spec.get("missing") == "skip":
                    continue
                raise
            if raw in (None, ""):
                if spec.get("missing") == "skip":
                    continue
                raw = None
            valid = flag(field(record, spec["valid_field"])) if "valid_field" in spec else raw is not None
            value = None
            if raw is not None:
                if kind == "control" and "mapping" in spec:
                    if str(raw) not in spec["mapping"]:
                        raise InputError(f"Unknown control position: {raw!r}")
                    value = number(spec["mapping"][str(raw)], "control mapping")
                else:
                    value = number(raw, spec["field"])
                    if kind == "speed":
                        if spec["unit"] == "km/h":
                            value /= 3.6
                        elif spec["unit"] in ("rad/s", "rpm"):
                            value *= number(spec["radius_m"], "radius_m")
                            if spec["unit"] == "rpm":
                                value *= 2 * math.pi / 60
                    else:
                        value = value * number(spec.get("scale", 1), "scale") + number(spec.get("offset", 0), "offset")
            event_stamp = profile.timestamp(field(record, spec["timestamp_field"])) if "timestamp_field" in spec else stamp
            channel = spec["id"] if kind == "speed" else None
            key = kind, channel
            seq_field = spec.get("seq_field", profile.config.get("seq_field"))
            seq = integer(field(record, seq_field), "seq") if seq_field else counters.get(key, 0)
            counters[key] = counters.get(key, 0) + 1
            batch.append(Event(event_stamp, profile.source_id, seq, kind, channel, value, valid))
        yield batch
