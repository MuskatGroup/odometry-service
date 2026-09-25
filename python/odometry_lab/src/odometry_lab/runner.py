import asyncio
import math
import time
from dataclasses import asdict
from pathlib import Path

from odometry_core import (
    AdaptiveOdometryEstimator,
    ControlSample,
    EstimatorConfig,
    InitialState,
    ModelConfig,
    OdometryEstimator,
)
from odometry_core.types import event_dict
from odometry_io import Normalizer, read_file, websocket_records

from .storage import write_json, write_rows


def _percentile(values, quantile):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    position = (len(values) - 1) * quantile
    left, right = math.floor(position), math.ceil(position)
    return (
        values[left] if left == right else values[left] + (values[right] - values[left]) * (position - left)
    )


def ingest(estimator, event):
    if isinstance(event, ControlSample):
        estimator.ingest_control(event)
    else:
        estimator.ingest_wheel(event)


def create_estimator(name="wheel-hold", model_config=None):
    if name == "wheel-hold":
        if model_config is not None:
            raise ValueError("model_config is only valid for adaptive-ekf")
        return OdometryEstimator(), EstimatorConfig()
    if name == "adaptive-ekf":
        config = model_config or ModelConfig()
        return AdaptiveOdometryEstimator(config), config
    raise ValueError(f"Unknown estimator: {name}")


def run_events(events, initial=None, config=None, hz=50, tail_s=0.0, estimator_name="wheel-hold"):
    if not events:
        raise ValueError("No usable input events")
    if hz <= 0 or tail_s < 0:
        raise ValueError("Invalid run frequency/tail")
    initial = initial or InitialState(events[0].stamp_ns)
    estimator, default_config = create_estimator(estimator_name, config)
    config = config or default_config
    estimator.initialize(initial, config)
    period = round(1e9 / hz)
    if period < 1:
        raise ValueError("Frequency too high")
    endpoint = events[-1].stamp_ns + round(tail_s * 1e9)
    ticks = range(initial.stamp_ns, endpoint + 1, period)
    output, position = [], 0
    last_tick = None
    for tick in ticks:
        while position < len(events) and events[position].stamp_ns <= tick:
            ingest(estimator, events[position])
            position += 1
        began = time.perf_counter()
        frame = estimator.advance_to(tick).to_dict()
        frame["compute_ms"] = (time.perf_counter() - began) * 1000
        output.append(frame)
        last_tick = tick
    if last_tick != endpoint:
        while position < len(events):
            ingest(estimator, events[position])
            position += 1
        frame = estimator.advance_to(endpoint).to_dict()
        frame["compute_ms"] = None
        output.append(frame)
    if hasattr(estimator, "counts"):
        counts = dict(estimator.counts)
    else:
        counts = {"accepted_wheels": estimator.accepted, "rejected_wheels": estimator.rejected}
    return output, counts


async def run_websocket(
    url, normalizer, duration_s=20, hz=50, estimator_name="wheel-hold", model_config=None
):
    if duration_s <= 0 or hz <= 0:
        raise ValueError("Invalid live duration/frequency")
    queue = asyncio.Queue(maxsize=4096)
    events, estimates = [], []
    diagnostics = normalizer.diagnostics

    async def receive():
        try:
            async for event in websocket_records(url, normalizer):
                try:
                    queue.put_nowait(event)
                except asyncio.QueueFull:
                    diagnostics["QUEUE_OVERFLOW"] += 1
        except Exception:
            diagnostics["SOURCE_ERROR"] += 1
        finally:
            diagnostics["SOURCE_DISCONNECTED"] += 1

    task = asyncio.create_task(receive())
    estimator, estimator_config = create_estimator(estimator_name, model_config)
    start = time.monotonic()
    origin = None
    began = None
    try:
        while time.monotonic() - start < duration_s:
            batch = []
            while not queue.empty():
                batch.append(queue.get_nowait())
            if origin is None and batch:
                origin = batch[0].stamp_ns
                began = time.monotonic()
                estimator.initialize(InitialState(origin), estimator_config)
            for event in batch:
                events.append(event)
                ingest(estimator, event)
            if origin is not None:
                stamp = origin + round((time.monotonic() - began) * 1e9)
                frame = estimator.advance_to(stamp).to_dict()
                frame["compute_ms"] = None
                estimates.append(frame)
            await asyncio.sleep(1 / hz)
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
    if not events:
        raise ValueError("Source supplied no usable events")
    counts = (
        dict(estimator.counts)
        if hasattr(estimator, "counts")
        else {"accepted_wheels": estimator.accepted, "rejected_wheels": estimator.rejected}
    )
    return events, estimates, counts


def run(
    source, profile, output, run_id, tail_s=0, duration_s=20, estimator_name="wheel-hold", model_config=None
):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    normalizer = Normalizer(profile)
    if source.startswith(("ws://", "wss://")):
        events, frames, counts = asyncio.run(
            run_websocket(
                source, normalizer, duration_s, estimator_name=estimator_name, model_config=model_config
            )
        )
    else:
        events = read_file(source, normalizer)
        frames, counts = run_events(events, config=model_config, tail_s=tail_s, estimator_name=estimator_name)
    for frame in frames:
        frame.update(run_id=run_id, schema_version="0.2")
    timings = [frame["compute_ms"] for frame in frames if frame.get("compute_ms") is not None]
    write_rows(output / "input-events.jsonl", map(event_dict, events))
    write_rows(output / "estimates.jsonl", frames)
    write_json(
        output / "run.json",
        {
            "run_id": run_id,
            "schema_version": "0.2",
            "status": "completed",
            "source": source,
            "model_version": frames[-1]["model_version"],
            "profile": profile,
            "synthetic": (output / "scenario.json").exists(),
            "config": asdict(
                model_config or (ModelConfig() if estimator_name == "adaptive-ekf" else EstimatorConfig())
            ),
            "diagnostics": {**dict(normalizer.diagnostics), **counts},
            "compute_ms": {
                "p50": _percentile(timings, 0.50),
                "p95": _percentile(timings, 0.95),
                "p99": _percentile(timings, 0.99),
                "max": max(timings, default=None),
            },
        },
    )
    return frames
