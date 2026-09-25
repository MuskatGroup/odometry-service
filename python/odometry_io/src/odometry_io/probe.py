"""Conservative profile draft generator for an unknown CSV/JSON/JSONL sample."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import yaml


def _records(path: Path, limit: int):
    suffix = path.suffix.lower()
    with path.open(encoding="utf-8-sig", newline="") as stream:
        if suffix == ".csv":
            return list(csv.DictReader(stream))[:limit]
        if suffix == ".jsonl":
            return [json.loads(line) for line in stream if line.strip()][:limit]
        if suffix == ".json":
            value = json.load(stream)
            if isinstance(value, list):
                return value[:limit]
            arrays = (
                [item for item in value.values() if isinstance(item, list)] if isinstance(value, dict) else []
            )
            if len(arrays) == 1:
                return arrays[0][:limit]
            raise ValueError("JSON must be an array or contain exactly one top-level array")
    raise ValueError("Probe supports CSV, JSON and JSONL")


def _score(name, tokens):
    lowered = name.lower()
    return max((len(token) for token in tokens if token in lowered), default=0)


def probe(path, limit=200):
    """Return a draft plus warnings; units and clock are never silently guessed as final facts."""
    path = Path(path)
    rows = _records(path, limit)
    if not rows or not all(isinstance(row, dict) for row in rows):
        raise ValueError("No object records found")
    fields = list(rows[0])
    time_field = max(fields, key=lambda name: _score(name, ("timestamp", "stamp", "time", "ts")))
    control_candidates = sorted(
        fields,
        key=lambda name: _score(name, ("controller", "control", "handle", "throttle", "u")),
        reverse=True,
    )
    control_field = control_candidates[0]
    wheel_fields = [name for name in fields if _score(name, ("wheel", "speed", "velocity", "rpm", "omega"))]
    event_layout = any(name.lower() in ("kind", "type", "event_type") for name in fields)
    kind_field = next((name for name in fields if name.lower() in ("kind", "type", "event_type")), None)
    wheel_id_field = next(
        (name for name in fields if name.lower() in ("wheel_id", "channel_id", "sensor_id")), None
    )
    warnings = [
        "REVIEW_REQUIRED: set the actual clock domain for every source",
        "REVIEW_REQUIRED: verify timestamp and wheel units before running",
        "REVIEW_REQUIRED: verify controller normalization to [-1, 1]",
    ]
    if not wheel_fields:
        warnings.append("No likely wheel-speed fields found")
    if time_field == control_field:
        warnings.append("Time/control inference is ambiguous")
    layout = "events" if event_layout and wheel_id_field else "wide"
    wheels = []
    if layout == "events":
        speed = wheel_fields[0] if wheel_fields else "speed_mps"
        wheel_values = sorted(
            {str(row.get(wheel_id_field)) for row in rows if row.get(wheel_id_field) is not None}
        )
        wheels = [{"id": value, "field": speed, "unit": "m/s"} for value in wheel_values]
    else:
        wheels = [{"id": name, "field": name, "unit": "m/s"} for name in wheel_fields]
    draft = {
        "version": "0.1",
        "layout": layout,
        "time": {"field": time_field, "unit": "ns", "clock": "REVIEW_REQUIRED"},
        "control": {"field": control_field, "mapping": "linear", "scale": 1},
        "wheels": wheels,
    }
    if layout == "events":
        draft.update({"kind_field": kind_field, "wheel_id_field": wheel_id_field})
    return {"profile": draft, "warnings": warnings, "sample_count": len(rows), "fields": fields}


def write_probe(path, output, limit=200):
    result = probe(path, limit)
    text = (
        "# "
        + "\n# ".join(result["warnings"])
        + "\n"
        + yaml.safe_dump(result["profile"], allow_unicode=True, sort_keys=False)
    )
    Path(output).write_text(text, encoding="utf-8")
    return result
