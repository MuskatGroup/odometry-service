import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import websockets
from odometry_io import Normalizer, ProfileError, load_profile, read_file, websocket_records
from odometry_lab.runner import run_events, run_websocket

ROOT = Path(__file__).resolve().parents[1]


def profile():
    return load_profile(ROOT / "contracts/profiles/events.yaml")


def records():
    return [
        {"kind": "control", "stamp_ns": str(i * 100_000_000), "seq": i, "u": 0, "valid": True}
        for i in range(11)
    ] + [
        {
            "kind": "wheel",
            "stamp_ns": str(i * 100_000_000),
            "seq": i,
            "wheel_id": "left",
            "speed_mps": 10,
            "valid": True,
        }
        for i in range(11)
    ]


def test_equivalent_csv_json_jsonl(tmp_path):
    data = records()
    (tmp_path / "input.json").write_text(json.dumps(data))
    (tmp_path / "input.jsonl").write_text("\n".join(map(json.dumps, data)))
    with (tmp_path / "input.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["kind", "stamp_ns", "seq", "u", "valid", "wheel_id", "speed_mps"]
        )
        writer.writeheader()
        writer.writerows([{**r, "valid": "true"} for r in data])
    sources = [
        read_file(tmp_path / ("input." + ext), Normalizer(profile())) for ext in ("csv", "json", "jsonl")
    ]
    assert sources[0] == sources[1] == sources[2]
    outputs = [run_events(source)[0] for source in sources]
    for result in outputs:
        for frame in result:
            frame.pop("compute_ms")
    assert outputs[0] == outputs[1] == outputs[2]


@pytest.mark.parametrize(
    "unit,value,radius",
    [
        ("m/s", 10, None),
        ("km/h", 36, None),
        ("rad/s", 10 / 0.3, 0.3),
        ("rpm", 10 * 60 / (2 * math.pi * 0.3), 0.3),
    ],
)
def test_units(unit, value, radius):
    p = profile()
    p["wheels"] = [{"id": "left", "field": "speed_mps", "unit": unit, "radius_m": radius}]
    event = Normalizer(p).normalize(
        {"kind": "wheel", "stamp_ns": "0", "seq": 1, "wheel_id": "left", "speed_mps": value}
    )
    assert event[0].speed_mps == pytest.approx(10)


def test_nan_unknown_profile_missing_and_zero():
    p = profile()
    p["wheels"][0]["unit"] = "unknown"
    with pytest.raises(ProfileError):
        Normalizer(p)
    p = profile()
    p["wheels"][0]["unit"] = "rpm"
    with pytest.raises(ProfileError):
        Normalizer(p)
    normalizer = Normalizer(profile())
    assert (
        normalizer.normalize({"kind": "wheel", "stamp_ns": "0", "wheel_id": "left", "speed_mps": "NaN"}) == []
    )
    assert normalizer.diagnostics["INVALID_RECORD"] == 1
    wide = Normalizer(load_profile(ROOT / "contracts/profiles/wide.yaml"))
    assert len(wide.normalize({"time_s": "0.0", "handle": "0", "left_kmh": "", "right_rpm": 0})) == 2


def test_large_timestamp_and_ros_objects():
    p = profile()
    normalizer = Normalizer(p)
    event = normalizer.normalize({"kind": "control", "stamp_ns": "1789000000123456789", "u": 0})[0]
    assert event.stamp_ns == 1789000000123456789
    ros = Normalizer(load_profile(ROOT / "contracts/profiles/ros-events.yaml"))
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=7)), seq=3, u=0.5, valid=True
    )
    assert ros.normalize({"message": msg, "kind": "control"})[0].stamp_ns == 2_000_000_007


async def test_websocket_uses_same_normalizer():
    data = records()

    async def handler(socket):
        for record in data:
            await socket.send(json.dumps(record))

    async with websockets.serve(handler, "127.0.0.1", 0) as server:
        port = server.sockets[0].getsockname()[1]
        normalizer = Normalizer(profile())
        events = [event async for event in websocket_records(f"ws://127.0.0.1:{port}", normalizer)]
    assert len(events) == len(data)
    assert events == [event for row in data for event in Normalizer(profile()).normalize(row)]


async def test_websocket_disconnect_becomes_stale():
    async def handler(socket):
        await socket.send(json.dumps({"kind": "control", "stamp_ns": "0", "seq": 0, "u": 0}))
        await socket.send(
            json.dumps({"kind": "wheel", "stamp_ns": "0", "seq": 0, "wheel_id": "left", "speed_mps": 10})
        )

    async with websockets.serve(handler, "127.0.0.1", 0) as server:
        port = server.sockets[0].getsockname()[1]
        n = Normalizer(profile())
        _, frames, _ = await run_websocket(f"ws://127.0.0.1:{port}", n, duration_s=0.7)
    assert n.diagnostics["SOURCE_DISCONNECTED"] == 1
    assert frames[-1]["valid"] is False
    assert "CONTROL_STALE" in frames[-1]["reason_codes"]
