"""Draft an adapter-1 profile from a sample of an unknown CSV/JSONL file.

    python -m odometry_io.probe data.csv [--format csv|jsonl] [--write profile.json]

Every guess is heuristic. The draft is validated by actually normalizing the sample, and each
assumption is listed as a note or a TODO. Guessed units and control encodings must be checked
against the organizers' data dictionary; nothing here reads a reference/truth column.
"""

import argparse
from decimal import Decimal
import json
import math
from pathlib import Path
import re
import sys
from statistics import median
from .core import InputError, Profile, normalize
from .sources import READERS

TIME_HINT = re.compile(r"(^|[._])(time|timestamp|stamp|ts|t|sec|secs|date|clock)($|[._])|time|stamp", re.I)
SEQ_HINT = re.compile(r"(^|[._])(seq|sequence|counter|index|idx|frame|n)($|[._])|seq", re.I)
CONTROL_HINT = re.compile(r"control|handle|knob|lever|throttle|notch|traction|brake|command|cmd|pos|ручк|тяг|тормоз", re.I)
STRONG_CONTROL = re.compile(r"control|handle|knob|lever|controller|ручк", re.I)
SPEED_HINT = re.compile(r"speed|vel|omega|rpm|wheel|rot|скор|колес|колёс|\bv\b|_v$|^v_", re.I)
VALID_HINT = re.compile(r"valid|ok|status|healthy|good|quality", re.I)
FAULT_HINT = re.compile(r"fault|error|err|fail|bad", re.I)
AGGREGATE_HINT = re.compile(r"avg|mean|average|fused|combined|total|median", re.I)


def flatten(value, prefix="", out=None, limit=64):
    out = {} if out is None else out
    if isinstance(value, dict):
        for key, item in value.items():
            flatten(item, f"{prefix}.{key}" if prefix else str(key), out, limit)
    elif isinstance(value, list):
        for index, item in enumerate(value[:limit]):
            flatten(item, f"{prefix}.{index}" if prefix else str(index), out, limit)
    else:
        out[prefix] = value
    return out


def to_float(value):
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float, Decimal)):
        return float(value)
    if isinstance(value, str):
        try:
            result = float(value.strip().replace(",", "."))
        except ValueError:
            return None
        return result if math.isfinite(result) else None
    return None


def is_boolean_like(values):
    return bool(values) and all(v in (True, False, 0, 1, "0", "1", "true", "false", "True", "False") for v in values)


class Column:
    def __init__(self, path, values):
        self.path, self.values = path, values
        self.filled = [v for v in values if v not in (None, "")]
        self.nums = [x for x in (to_float(v) for v in self.filled) if x is not None]
        self.missing = 1 - len(self.filled) / len(values) if values else 1
        self.numeric = bool(self.filled) and len(self.nums) / len(self.filled) >= 0.98
        self.strings = sorted({str(v) for v in self.filled if to_float(v) is None and not isinstance(v, bool)})
        self.unique = len(set(self.nums)) if self.numeric else len(set(map(str, self.filled)))
        self.integer_like = self.numeric and all(x == int(x) for x in self.nums)
        self.name = re.split(r"[.]", path)[-1] if path else path
        self.monotonic = self.numeric and len(self.nums) >= 3 and all(a <= b for a, b in zip(self.nums, self.nums[1:]))
        deltas = [b - a for a, b in zip(self.nums, self.nums[1:]) if b > a]
        self.delta = median(deltas) if deltas else 0.0


def guess_time_unit(col):
    name = col.name.lower()
    for pattern, unit in ((r"(ns|nsec|nano)", "ns"), (r"(us|usec|micro)", "us"), (r"(ms|msec|milli)", "ms"),
                          (r"(s|sec|secs|seconds)", "s")):
        if re.search(rf"(^|[_.]){pattern}($|[_.])", name) or (unit != "s" and re.search(pattern + "$", name)):
            return unit, "high", "unit taken from the field name"
    magnitude = abs(median(col.nums))
    if magnitude >= 1e17:
        return "ns", "medium", "epoch-like magnitude"
    if magnitude >= 1e14:
        return "us", "medium", "epoch-like magnitude"
    if magnitude >= 1e11:
        return "ms", "medium", "epoch-like magnitude"
    if magnitude >= 1e8:
        return "s", "medium", "epoch-like magnitude"
    d = col.delta
    if col.integer_like and d >= 1:
        unit = "ms" if d <= 1e4 else "us" if d <= 1e7 else "ns"
        return unit, "low", f"integer counter with median step {d:g}; could be another unit"
    return "s", "low", f"relative time, median step {d:g}; assumed seconds"


def sample_rate_hz(col, unit):
    scale = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}[unit]
    return 1.0 / (col.delta * scale) if col.delta > 0 else None


def guess_speed_unit(col):
    name = col.name.lower()
    peak = max(abs(x) for x in col.nums)
    if "rpm" in name:
        return "rpm", "high", "name contains rpm"
    if re.search(r"omega|rad", name):
        return "rad/s", "medium", "name suggests angular speed"
    if re.search(r"km_?h|kmh|kph", name):
        return "km/h", "high", "name contains km/h"
    if re.search(r"mps|m_s|m/s", name):
        return "m/s", "high", "name contains m/s"
    if peak <= 30:
        return "m/s", "low", f"peak {peak:g} fits m/s for a tram (<=~25 m/s); could be km/h at low speed"
    if peak <= 130:
        return "km/h", "low", f"peak {peak:g} fits km/h; could be a different unit"
    return "rpm", "low", f"peak {peak:g} is too large for m/s or km/h; assumed rpm"


def channel_id(path, used):
    base = re.sub(r"[^0-9a-zA-Z]+", "_", path).strip("_").lower() or "ch"
    candidate, n = base, 2
    while candidate in used:
        candidate, n = f"{base}_{n}", n + 1
    used.add(candidate)
    return candidate


def tokens(path):
    stop = {"valid", "ok", "status", "healthy", "good", "quality", "speed", "vel", "velocity", "omega", "rpm", "wheel", "is"}
    return {t for t in re.split(r"[^a-z0-9]+", path.lower()) if t and t not in stop and not t.isdigit()} | \
           {t for t in re.findall(r"\d+", path)}


CONTROL_WORDS = ((re.compile(r"emerg", re.I), -1.0), (re.compile(r"brake|стоп|тормоз", re.I), -0.4),
                 (re.compile(r"coast|neutral|zero|idle|off|выбег|нейтр", re.I), 0.0),
                 (re.compile(r"traction|power|accel|drive|тяг|run", re.I), 0.6))


def probe(records, source_id="probe"):
    """records: list of dict. Returns (profile, notes, todo)."""
    notes, todo = [], []
    flat = [flatten(r) for r in records]
    paths = list(dict.fromkeys(p for row in flat for p in row))
    cols = {p: Column(p, [row.get(p) for row in flat]) for p in paths}
    profile = {"schema_version": "adapter-1", "source_id": source_id}
    used_paths = set()

    # timestamp: monotonic numeric column, preferring time-like names
    candidates = [c for c in cols.values() if c.numeric and c.monotonic and c.unique > 2 and c.missing == 0]
    scored = sorted(candidates, key=lambda c: (-(3 * bool(TIME_HINT.search(c.path)) + (c.delta > 0) - 2 * bool(SEQ_HINT.search(c.path) and not TIME_HINT.search(c.path))), c.path))
    time_col = scored[0] if scored else None
    if not time_col or not (TIME_HINT.search(time_col.path) or not time_col.integer_like or time_col.delta != 1):
        time_col = next((c for c in scored if TIME_HINT.search(c.path)), time_col if time_col and time_col.delta != 1 else None)
    if time_col is None:
        todo.append("No monotonic numeric timestamp column found; set profile.timestamp.field and unit manually.")
        profile["timestamp"] = {"field": None, "unit": None}
    else:
        unit, confidence, why = guess_time_unit(time_col)
        profile["timestamp"] = {"field": time_col.path, "unit": unit}
        used_paths.add(time_col.path)
        rate = sample_rate_hz(time_col, unit)
        notes.append(f"timestamp: {time_col.path} [{unit}] ({confidence} confidence: {why})"
                     + (f", ~{rate:.3g} Hz" if rate else ""))
        if confidence == "low":
            todo.append(f"Confirm the timestamp unit of {time_col.path} (guessed {unit}) and whether all sources share one clock.")
        sec = [p for p in cols if p.endswith(".sec") or p.endswith(".nanosec")]
        if sec:
            todo.append("Found sec/nanosec pairs: adapter-1 reads one timestamp field; combine them in a custom reader or use a ROS bridge (header stamp is handled there).")
        # gap / duplicate checks
        if time_col.delta > 0:
            diffs = [b - a for a, b in zip(time_col.nums, time_col.nums[1:])]
            if any(d == 0 for d in diffs):
                notes.append("warning: repeated timestamps present (they are merged per timestamp group, conflicting values are rejected)")
            if any(d > 10 * time_col.delta for d in diffs):
                notes.append("warning: gaps longer than 10x the median step (source dropouts or several recordings)")

    # sequence counter
    for col in cols.values():
        if col.path not in used_paths and col.numeric and col.integer_like and col.monotonic and SEQ_HINT.search(col.path) \
                and len(col.nums) > 3 and col.delta == 1 and col.missing == 0:
            profile["seq_field"] = col.path
            used_paths.add(col.path)
            notes.append(f"seq_field: {col.path} (integer counter, step 1)")
            break

    # control
    def control_ok(c):
        if c.path in used_paths or c.missing == 1 or SPEED_HINT.search(c.path) and not STRONG_CONTROL.search(c.path):
            return False
        if not CONTROL_HINT.search(c.path) or is_boolean_like(c.filled) and VALID_HINT.search(c.path):
            return False
        if c.numeric:
            return c.unique <= 25 or (min(c.nums) >= -1 and max(c.nums) <= 1)
        return len(c.strings) <= 12
    controls = sorted((c for c in cols.values() if control_ok(c)), key=lambda c: (not STRONG_CONTROL.search(c.path), c.path))
    if controls:
        col = controls[0]
        used_paths.add(col.path)
        spec = {"field": col.path}
        if col.numeric:
            low, high = min(col.nums), max(col.nums)
            if low >= -1 and high <= 1:
                notes.append(f"control: {col.path} already in [-1, 1]")
            elif col.integer_like and low < 0:
                spec["scale"] = round(1 / max(abs(low), abs(high)), 6)
                notes.append(f"control: {col.path} integer notches {low:g}..{high:g}, scaled symmetrically (negative = braking)")
                todo.append(f"Verify control encoding of {col.path}: sign convention, neutral position and whether traction and brake share the axis.")
            elif col.integer_like:
                spec["scale"] = round(1 / high, 6) if high else 1
                notes.append(f"control: {col.path} notches {low:g}..{high:g}, treated as traction-only")
                todo.append(f"{col.path} has no negative values: braking may live in another field or in the upper notches; confirm the encoding.")
            else:
                spec["scale"] = round(1 / max(abs(low), abs(high)), 6)
                todo.append(f"Continuous control {col.path} range {low:g}..{high:g} scaled to [-1, 1]; confirm the zero point.")
        else:
            mapping = {}
            for word in col.strings:
                for pattern, value in CONTROL_WORDS:
                    if pattern.search(word):
                        mapping[word] = value
                        break
            missing_words = [w for w in col.strings if w not in mapping]
            spec["mapping"] = mapping
            notes.append(f"control: {col.path} categories {col.strings}; mapping values are GUESSES ({mapping})")
            todo.append(f"Check the control mapping of {col.path} (traction=+0.6, brake=-0.4, coast=0 are placeholders)"
                        + (f" and add values for: {missing_words}" if missing_words else "") + ".")
        profile["control"] = spec
        for other in controls[1:]:
            notes.append(f"other control-like column not used: {other.path}")
    else:
        todo.append("No controller-handle column found; the profile has no control (model-only prediction needs it).")

    # wheel speed channels
    speed_cols = [c for c in cols.values() if c.path not in used_paths and c.numeric and SPEED_HINT.search(c.path)
                  and not TIME_HINT.search(c.path) and not is_boolean_like(c.filled)]
    used_ids, channels = set(), []
    for col in speed_cols:
        unit, confidence, why = guess_speed_unit(col)
        spec = {"id": channel_id(col.path, used_ids), "field": col.path, "unit": unit}
        if unit in ("rad/s", "rpm"):
            spec["radius_m"] = None
            todo.append(f"Set radius_m for {col.path} (unit {unit}); the draft cannot guess wheel radius and will not load until it is set.")
        if col.missing > 0:
            spec["missing"] = "skip"
        if confidence == "low":
            todo.append(f"Confirm the unit of {col.path} (guessed {unit}: {why}).")
        notes.append(f"speed channel {spec['id']}: {col.path} [{unit}] ({confidence} confidence: {why})"
                     + (f", {col.missing:.0%} missing -> missing=skip" if col.missing else ""))
        if min(col.nums) < 0:
            notes.append(f"warning: {col.path} has negative values (reverse direction? MVP assumes forward motion)")
        channels.append(spec)
        used_paths.add(col.path)
    profile["channels"] = channels
    if not channels:
        todo.append("No wheel-speed columns found; add profile.channels manually.")

    # validity flags: attach to the channel with the most similar tokens
    for col in cols.values():
        if col.path in used_paths or not (VALID_HINT.search(col.path) or FAULT_HINT.search(col.path)) or not is_boolean_like(col.filled):
            continue
        if FAULT_HINT.search(col.path) and not VALID_HINT.search(col.path):
            notes.append(f"flag {col.path} looks like a FAULT indicator (1 = bad); not used, adapter-1 treats 1 as valid")
            continue
        best = max(channels, key=lambda s: len(tokens(s["field"]) & tokens(col.path)), default=None)
        if best is not None and (len(channels) == 1 or tokens(best["field"]) & tokens(col.path)):
            best["valid_field"] = col.path
            used_paths.add(col.path)
            notes.append(f"validity flag {col.path} -> channel {best['id']}")
        else:
            notes.append(f"flag {col.path} not attached to any channel (ambiguous)")

    # aggregates of other channels must not be fused together with them
    aggregates = [s for s in channels if AGGREGATE_HINT.search(s["field"])]
    parts = [s for s in channels if s not in aggregates]
    if aggregates and parts:
        for agg in aggregates:
            agg["derived_from"] = [s["id"] for s in parts]
        profile["fusion"] = {"strategy": "median", "variance_floor": 0.25, "channels": [s["id"] for s in parts]}
        notes.append(f"aggregate channels {[s['id'] for s in aggregates]} excluded from fusion (derived from {[s['id'] for s in parts]})")
    else:
        profile["fusion"] = {"strategy": "median", "variance_floor": 0.25}
    ignored = [p for p in paths if p not in used_paths]
    if ignored:
        notes.append(f"unused columns: {ignored}")
    return profile, notes, todo


def validate(profile, records):
    """Return an error string if the draft cannot normalize the sample, else None."""
    try:
        prof = Profile(json.loads(json.dumps(profile)))
        events = sum(len(batch) for batch in normalize(records, prof))
        return None if events else "draft produced no events"
    except (InputError, KeyError, TypeError, ValueError) as exc:
        return f"{type(exc).__name__}: {exc}"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path)
    parser.add_argument("--format", choices=sorted(READERS), help="default: from the file extension")
    parser.add_argument("--rows", type=int, default=1000, help="rows to inspect")
    parser.add_argument("--source-id", default="probe")
    parser.add_argument("--write", type=Path, help="write the draft profile here (otherwise printed to stdout)")
    args = parser.parse_args(argv)
    fmt = args.format or {".csv": "csv", ".jsonl": "jsonl", ".ndjson": "jsonl"}.get(args.input.suffix.lower())
    if fmt is None:
        parser.error("cannot infer the format; pass --format")
    records = []
    for record in READERS[fmt](args.input):
        records.append(record)
        if len(records) >= args.rows:
            break
    if not records:
        print("No records found", file=sys.stderr)
        return 2
    profile, notes, todo = probe(records, args.source_id)
    problem = validate(profile, records)
    print(f"Inspected {len(records)} records of {args.input}", file=sys.stderr)
    for line in notes:
        print(f"  - {line}", file=sys.stderr)
    if problem:
        todo.insert(0, f"The draft does not normalize the sample yet: {problem}")
    for line in todo:
        print(f"  TODO: {line}", file=sys.stderr)
    text = json.dumps(profile, ensure_ascii=False, indent=2)
    if args.write:
        args.write.write_text(text, encoding="utf-8")
        print(f"Draft written to {args.write}", file=sys.stderr)
    else:
        print(text)
    return 1 if problem else 0


if __name__ == "__main__":
    raise SystemExit(main())
