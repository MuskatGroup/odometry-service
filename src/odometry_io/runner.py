"""Bounded ordering, deterministic output ticks, and a replaceable estimator."""

from collections import OrderedDict
import heapq
import json
import math
from pathlib import Path
import time
from typing import Protocol
from .core import Event, InputError, Profile, number
from .fusion import STRATEGIES


class Estimator(Protocol):
    def ingest(self, stamp_ns: int, events: list[Event]) -> None: ...
    def advance_to(self, stamp_ns: int) -> dict: ...


class HoldEstimator:
    """B0 wheel + hold only. Does not pretend to implement the proposed EKF."""

    def __init__(self, profile: Profile, speed_timeout_s=0.5):
        self.profile = profile
        strategy = profile.fusion.get("strategy", "median")
        if strategy not in STRATEGIES:
            raise InputError(f"Unknown fusion strategy: {strategy}")
        if strategy == "selected_channel" and profile.fusion.get("channel") not in profile.selected:
            raise InputError("selected_channel requires a selected configured channel")
        self.strategy = STRATEGIES[strategy]
        self.timeout_ns = int(number(speed_timeout_s, "speed_timeout_s") * 1e9)
        if self.timeout_ns <= 0:
            raise InputError("speed_timeout_s must be positive")
        self.time = None
        self.speed = None
        self.position = 0.0
        self.last_speed = None
        self.control = None
        self.last_control = None
        self.channels = {}
        self.sequence = 0
        self.last_used = ()

    def _advance(self, stamp):
        if self.time is not None:
            if stamp < self.time:
                raise InputError("Estimator time cannot move backwards")
            if self.speed is not None:
                self.position += self.speed * ((stamp - self.time) / 1e9)
                if not math.isfinite(self.position):
                    raise InputError("Position overflow")
        self.time = stamp

    def ingest(self, stamp_ns, events):
        self._advance(stamp_ns)
        candidates = []
        configured = {spec["id"] for spec in self.profile.channels}
        streams = set()
        for event in events:
            stream = event.kind, event.channel_id
            if stream in streams:
                raise InputError("Multiple new values for one stream at the same timestamp")
            streams.add(stream)
            if event.kind == "control":
                self.control = event.value if event.input_valid else None
                self.last_control = event.stamp_ns if event.input_valid else None
            else:
                if event.channel_id not in configured:
                    raise InputError(f"Unknown normalized channel: {event.channel_id}")
                self.channels[event.channel_id] = event
                if event.input_valid and event.channel_id in self.profile.selected:
                    candidates.append(event)
        result = self.strategy(candidates, self.profile.fusion) if candidates else None
        if result is not None:
            self.speed = number(result.speed_mps, "fused speed")
            if number(result.variance, "fused variance") <= 0:
                raise InputError("Fusion variance must be positive")
            self.last_speed = stamp_ns
            self.last_used = result.used_channels

    def advance_to(self, stamp_ns):
        self._advance(stamp_ns)
        age = None if self.last_speed is None else (stamp_ns - self.last_speed) / 1e9
        available = any(self.channels[channel].input_valid and
                        stamp_ns - self.channels[channel].stamp_ns <= self.timeout_ns
                        for channel in self.last_used)
        valid = self.last_speed is not None and stamp_ns - self.last_speed <= self.timeout_ns and available
        mode = "INITIALIZING" if self.speed is None else "BASELINE_HOLD" if valid else "INVALID"
        reasons = ["BASELINE_ONLY", "UNCERTAINTY_UNCALIBRATED"]
        if self.speed is not None and not valid:
            reasons.append("SPEED_STALE")
        result = {
            "stamp_ns": str(stamp_ns), "seq": self.sequence,
            "s_m": self.position if self.speed is not None else None, "v_mps": self.speed,
            "control_u": self.control, "speed_age_s": age,
            "control_age_s": None if self.last_control is None else (stamp_ns - self.last_control) / 1e9,
            "mode": mode, "valid": valid, "reason_codes": reasons,
            "sigma_s_m": None, "sigma_v_mps": None,
            "model_version": "wheel-hold-1",
            "speed_channels": [{"channel_id": key, "speed_mps": event.value,
                                "stamp_ns": str(event.stamp_ns), "input_valid": event.input_valid,
                                "age_s": (stamp_ns - event.stamp_ns) / 1e9}
                               for key, event in sorted(self.channels.items())],
        }
        self.sequence += 1
        return result


def ordered_groups(batches, stats, reorder_ns=0, max_pending=10000, dedup_capacity=100000):
    """Close only timestamps strictly below watermark; equal-time packets stay atomic.

    Dedup cache is bounded. Repeats after eviction are still rejected as late when
    behind the closed boundary. No unbounded history or silent timestamp sorting.
    """
    if reorder_ns < 0 or max_pending <= 0 or dedup_capacity <= 0:
        raise InputError("Invalid ordering limits")
    heap, seen = [], OrderedDict()
    closed, high, serial = None, None, 0
    for batch in batches:
        for event in batch:
            if event.key in seen:
                if seen[event.key] != event:
                    raise InputError(f"Conflicting duplicate: {event.key}")
                stats["duplicates"] += 1
                continue
            if closed is not None and event.stamp_ns <= closed:
                stats["late_events"] += 1
                continue
            seen[event.key] = event
            if len(seen) > dedup_capacity:
                seen.popitem(last=False)
            heapq.heappush(heap, (event.stamp_ns, serial, event))
            serial += 1
            high = event.stamp_ns if high is None else max(high, event.stamp_ns)
            if len(heap) > max_pending:
                raise InputError("Pending input limit exceeded; reduce reorder window or increase max_pending")
        if high is None:
            continue
        watermark = high - reorder_ns
        while heap and heap[0][0] < watermark:
            stamp = heap[0][0]
            group = []
            while heap and heap[0][0] == stamp:
                group.append(heapq.heappop(heap)[2])
            closed = stamp
            yield stamp, sorted(group, key=lambda e: (e.kind != "control", e.channel_id or "", e.seq))
    while heap:
        stamp = heap[0][0]
        group = []
        while heap and heap[0][0] == stamp:
            group.append(heapq.heappop(heap)[2])
        yield stamp, sorted(group, key=lambda e: (e.kind != "control", e.channel_id or "", e.seq))


def run(batches, profile: Profile, output: Path, *, estimator: Estimator | None = None,
        output_hz=50, reorder_ms=0, speed_timeout_s=0.5, tail_s=0,
        max_pending=10000, max_outputs=1_000_000):
    hz = number(output_hz, "output_hz")
    if hz <= 0 or hz > 1e9:
        raise InputError("output_hz must be > 0 and <= 1e9")
    step = round(1e9 / hz)
    tail = number(tail_s, "tail_s")
    reorder = number(reorder_ms, "reorder_ms")
    if tail < 0 or reorder < 0 or max_outputs <= 0:
        raise InputError("Invalid output horizon/reorder limits")
    estimator = estimator if estimator is not None else HoldEstimator(profile, speed_timeout_s)
    output.mkdir(parents=True, exist_ok=True)
    # Never silently overwrite a previous run or its artifacts.
    if any(output.iterdir()):
        raise InputError(f"Output directory must be empty: {output}")
    stats = {"duplicates": 0, "late_events": 0, "input_events": 0, "output_frames": 0,
             "valid_frames": 0, "schema_version": "adapter-1"}
    (output / "profile.json").write_text(json.dumps(profile.config, ensure_ascii=False, indent=2), encoding="utf-8")
    config = {"output_hz": hz, "actual_period_ns": step, "reorder_ms": reorder,
              "speed_timeout_s": speed_timeout_s, "tail_s": tail, "max_pending": max_pending,
              "max_outputs": max_outputs, "estimator": type(estimator).__name__}
    if hasattr(estimator, "describe"):
        config["estimator_config"] = estimator.describe()
    (output / "run.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    timings = []
    with (output / "normalized.jsonl").open("w", encoding="utf-8") as raw_stream, \
         (output / "estimates.jsonl").open("w", encoding="utf-8") as estimate_stream:
        def capture():
            for batch in batches:
                batch = list(batch)
                stats["input_events"] += len(batch)
                raw_stream.write(json.dumps({"schema_version": "adapter-1", "events": [e.to_dict() for e in batch]}, allow_nan=False) + "\n")
                yield batch

        def emit(stamp):
            if stats["output_frames"] >= max_outputs:
                raise InputError("Output frame limit exceeded; check timestamp units or split the run")
            started = time.perf_counter()
            result = estimator.advance_to(stamp)
            timings.append(time.perf_counter() - started)
            estimate_stream.write(json.dumps(result, allow_nan=False) + "\n")
            stats["output_frames"] += 1
            stats["valid_frames"] += int(result["valid"])

        next_tick, last_stamp = None, None
        for stamp, group in ordered_groups(capture(), stats, int(reorder * 1e6), max_pending):
            if next_tick is None:
                next_tick = stamp
            while next_tick < stamp:
                emit(next_tick)
                next_tick += step
            started = time.perf_counter()
            estimator.ingest(stamp, group)
            timings.append(time.perf_counter() - started)
            if next_tick == stamp:
                emit(next_tick)
                next_tick += step
            last_stamp = stamp
        if last_stamp is None:
            raise InputError("No input events")
        end = last_stamp + round(tail * 1e9)
        while next_tick <= end:
            emit(next_tick)
            next_tick += step
    stats["valid_fraction"] = stats["valid_frames"] / stats["output_frames"]
    # Wall-clock cost of estimator.ingest/advance_to calls on this machine; kept out of
    # estimates.jsonl so replays stay byte-identical. Not a hard real-time guarantee.
    ordered = sorted(timings)
    stats["compute_ms"] = {"calls": len(ordered)}
    if ordered:
        for name, q in (("p50", 0.50), ("p95", 0.95), ("p99", 0.99)):
            stats["compute_ms"][name] = ordered[min(len(ordered) - 1, math.ceil(q * len(ordered)) - 1)] * 1e3
        stats["compute_ms"]["max"] = ordered[-1] * 1e3
    stats["accuracy"] = None
    stats["accuracy_reason"] = "No independent reference evaluated"
    (output / "report.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
    return stats
