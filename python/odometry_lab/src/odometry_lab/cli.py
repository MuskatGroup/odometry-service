import argparse
import asyncio
import json
import time
from itertools import groupby
from pathlib import Path
from urllib.request import Request, urlopen

import websockets
from odometry_io import load_profile

from .evaluate import evaluate
from .runner import run
from .scenario import generate
from .storage import read_rows


def request(base, path, body, method="POST"):
    req = Request(
        base.rstrip("/") + path,
        data=json.dumps(body, allow_nan=False).encode(),
        method=method,
        headers={"Content-Type": "application/json"},
    )
    with urlopen(req, timeout=15) as response:
        return response.read()


def import_report(directory, api):
    directory = Path(directory)
    metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
    run_id = metadata["run_id"]
    request(api, "/api/runs", metadata)
    frames = read_rows(directory / "estimates.jsonl")
    for offset in range(0, len(frames), 250):
        request(api, f"/api/runs/{run_id}/telemetry", frames[offset : offset + 250])
    request(
        api,
        f"/api/runs/{run_id}/report",
        json.loads((directory / "report.json").read_text(encoding="utf-8")),
        "PUT",
    )


async def serve_file(path, host, port, system_time=False):
    rows = read_rows(path)

    async def handler(socket):
        previous = int(rows[0]["stamp_ns"]) if rows else 0
        offset = time.time_ns() - previous if system_time else 0
        started = time.monotonic()
        first = previous
        try:
            for stamp, group in groupby(rows, key=lambda r: int(r["stamp_ns"])):
                await asyncio.sleep(max(0, (stamp - first) / 1e9 - (time.monotonic() - started)))
                for row in group:
                    await socket.send(json.dumps({**row, "stamp_ns": str(int(row["stamp_ns"]) + offset)}))
                previous = stamp
        except websockets.ConnectionClosed:
            pass

    async with websockets.serve(handler, host, port, max_queue=32, max_size=1_048_576):
        print(f"WebSocket source listening on ws://{host}:{port}", flush=True)
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description="Odometry Failure Lab")
    commands = parser.add_subparsers(dest="command", required=True)
    gen = commands.add_parser("generate")
    gen.add_argument("--output", required=True)
    gen.add_argument("--seed", type=int, default=42)
    gen.add_argument("--fault", choices=["none", "slip", "slide", "freeze", "dropout"], default="dropout")
    gen.add_argument("--duration", type=float, default=20)
    gen.add_argument("--fault-start", type=float, default=8)
    gen.add_argument("--fault-end", type=float, default=11)
    execute = commands.add_parser("run")
    execute.add_argument("--source", required=True)
    execute.add_argument("--profile", required=True)
    execute.add_argument("--output", required=True)
    execute.add_argument("--run-id", default="demo-001")
    execute.add_argument("--tail", type=float, default=0)
    execute.add_argument("--duration", type=float, default=20, help="Live source observation duration")
    score = commands.add_parser("evaluate")
    score.add_argument("--estimates", required=True)
    score.add_argument("--truth")
    score.add_argument("--output", required=True)
    upload = commands.add_parser("import-report")
    upload.add_argument("--directory", required=True)
    upload.add_argument("--api", default="http://localhost:8080")
    server = commands.add_parser("serve-ws")
    server.add_argument("--source", required=True)
    server.add_argument("--host", default="127.0.0.1")
    server.add_argument("--port", type=int, default=8765)
    server.add_argument(
        "--system-time", action="store_true", help="Align synthetic stream to system time for ROS"
    )
    demo = commands.add_parser("demo")
    demo.add_argument("--output", default="artifacts/demo")
    demo.add_argument("--profile", default="contracts/profiles/events.yaml")
    demo.add_argument("--api")
    args = parser.parse_args()
    if args.command == "generate":
        generate(args.output, args.seed, args.duration, args.fault, args.fault_start, args.fault_end)
    elif args.command == "run":
        run(args.source, load_profile(args.profile), args.output, args.run_id, args.tail, args.duration)
    elif args.command == "evaluate":
        evaluate(args.estimates, args.truth, args.output)
    elif args.command == "import-report":
        import_report(args.directory, args.api)
    elif args.command == "serve-ws":
        asyncio.run(serve_file(args.source, args.host, args.port, args.system_time))
    elif args.command == "demo":
        folder = Path(args.output)
        generate(folder)
        run(str(folder / "events.jsonl"), load_profile(args.profile), folder, "demo-001")
        evaluate(folder / "estimates.jsonl", folder / "truth.jsonl", folder / "report.json")
        if args.api:
            import_report(folder, args.api)
    print("Completed:", args.command)


if __name__ == "__main__":
    main()
