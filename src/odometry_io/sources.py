"""Reader registry: add a transport without modifying normalization or estimation."""

import csv
from decimal import Decimal
import json
from pathlib import Path
from .core import Event, InputError


def read_jsonl(path: Path):
    with path.open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                yield json.loads(line, parse_float=Decimal)
            except (ValueError, TypeError) as exc:
                raise InputError(f"{path}:{line_number}: invalid JSON") from exc


def read_csv(path: Path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        yield from csv.DictReader(stream)


def read_emulator(path):
    from .emulator import read_emulator as reader
    yield from reader(path)


READERS = {"jsonl": read_jsonl, "csv": read_csv, "emulator": read_emulator}


def register_reader(name, reader):
    if name in READERS or not callable(reader):
        raise InputError(f"Cannot register reader: {name}")
    READERS[name] = reader


def read_normalized(path: Path):
    for record in read_jsonl(path):
        if record.get("schema_version") != "adapter-1" or not isinstance(record.get("events"), list):
            raise InputError("Expected adapter-1 normalized batch")
        yield [Event.from_dict(event) for event in record["events"]]
