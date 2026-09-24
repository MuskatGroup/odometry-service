import csv
import json
from pathlib import Path

import websockets
from odometry_core.types import event_key

from .normalizer import field


def read_file(path, normalizer, format=None):
    """Finite offline inputs are sorted before replay; size limit bounds memory."""
    path = Path(path)
    if path.stat().st_size > normalizer.profile.get("max_file_bytes", 100_000_000):
        raise ValueError("Input exceeds max_file_bytes; split the recording")
    format = format or path.suffix.lstrip(".").lower()
    with path.open(encoding="utf-8-sig", newline="") as stream:
        if format == "csv":
            records = csv.DictReader(stream, delimiter=normalizer.profile.get("delimiter", ","))
        elif format == "json":
            data = json.load(stream)
            records = field(data, normalizer.profile.get("records_path"), data)
            if not isinstance(records, list):
                raise ValueError("JSON records_path must select an array")
        elif format == "jsonl":
            records = _json_lines(stream, normalizer)
        else:
            raise ValueError("Unsupported file format")
        events = [event for record in records for event in normalizer.normalize(record)]
    return sorted(events, key=event_key)


def _json_lines(stream, normalizer):
    for line in stream:
        if not line.strip():
            continue
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            normalizer.diagnostics["MALFORMED_JSON"] += 1


async def websocket_records(url, normalizer):
    # Library receive queue and frame size are bounded. No automatic reconnect.
    async with websockets.connect(
        url, max_queue=32, max_size=1_048_576, open_timeout=10, close_timeout=2
    ) as socket:
        async for message in socket:
            try:
                record = json.loads(message)
            except (ValueError, TypeError):
                normalizer.diagnostics["MALFORMED_JSON"] += 1
                continue
            for event in normalizer.normalize(record):
                yield event
