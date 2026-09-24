import asyncio
import time
from dataclasses import asdict
from pathlib import Path

from odometry_core import ControlSample, EstimatorConfig, InitialState, OdometryEstimator
from odometry_core.types import event_dict
from odometry_io import Normalizer, read_file, websocket_records

from .storage import write_json, write_rows


def ingest(estimator, event):
    if isinstance(event, ControlSample):
        estimator.ingest_control(event)
    else:
        estimator.ingest_wheel(event)


def run_events(events, initial=None, config=None, hz=50, tail_s=0.0):
    if not events:
        raise ValueError("No usable input events")
    if hz <= 0 or tail_s < 0:
        raise ValueError("Invalid run frequency/tail")
    initial = initial or InitialState(events[0].stamp_ns)
    config = config or EstimatorConfig()
    estimator = OdometryEstimator()
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
    return output, dict(estimator.counts)


async def run_websocket(url, normalizer, duration_s=20, hz=50):
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
    estimator = OdometryEstimator()
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
                estimator.initialize(InitialState(origin), EstimatorConfig())
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
    return events, estimates, dict(estimator.counts)


def run(source, profile, output, run_id, tail_s=0, duration_s=20):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    normalizer = Normalizer(profile)
    if source.startswith(("ws://", "wss://")):
        events, frames, counts = asyncio.run(run_websocket(source, normalizer, duration_s))
    else:
        events = read_file(source, normalizer)
        frames, counts = run_events(events, tail_s=tail_s)
    for frame in frames:
        frame.update(run_id=run_id, schema_version="0.2")
    write_rows(output / "input-events.jsonl", map(event_dict, events))
    write_rows(output / "estimates.jsonl", frames)
    write_json(
        output / "run.json",
        {
            "run_id": run_id,
            "schema_version": "0.2",
            "status": "completed",
            "source": source,
            "model_version": "wheel-hold-v1",
            "profile": profile,
            "synthetic": (output / "scenario.json").exists(),
            "config": asdict(EstimatorConfig()),
            "diagnostics": {**dict(normalizer.diagnostics), **counts},
        },
    )
    return frames
