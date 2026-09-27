import argparse
import asyncio
import json
import time
from itertools import groupby
from pathlib import Path
from urllib.request import Request, urlopen

import websockets
from odometry_core import ModelConfig
from odometry_io import load_profile, write_probe

from .benchmark import PRESETS, SCENARIOS, benchmark
from .calibrate import apply_to_profile, calibrate, load_profile_config
from .evaluate import evaluate
from .exportrun import export_run
from .realdata import GNSS_MODES, build_case, dedupe, load_split, real_benchmark
from .realdata import SCENARIOS as REAL_SCENARIOS
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
    gen.add_argument(
        "--fault",
        choices=[
            "none",
            "slip",
            "common_slip",
            "slide",
            "lock",
            "freeze",
            "dropout",
            "scale",
            "grade",
            "grade_in_dropout",
            "traction_scale",
        ],
        default="dropout",
    )
    gen.add_argument("--duration", type=float, default=20)
    gen.add_argument("--fault-start", type=float, default=8)
    gen.add_argument("--fault-end", type=float, default=11)
    gen.add_argument("--wheels", type=int, default=2)
    execute = commands.add_parser("run")
    execute.add_argument("--source", required=True)
    execute.add_argument("--profile", required=True)
    execute.add_argument("--output", required=True)
    execute.add_argument("--run-id", default="demo-001")
    execute.add_argument("--tail", type=float, default=0)
    execute.add_argument("--duration", type=float, default=20, help="Live source observation duration")
    execute.add_argument("--estimator", choices=["wheel-hold", "adaptive-ekf"], default="adaptive-ekf")
    execute.add_argument("--model-config", help="JSON file overriding adaptive EKF parameters")
    score = commands.add_parser("evaluate")
    score.add_argument("--estimates", required=True)
    score.add_argument("--truth")
    score.add_argument("--faults", help="Optional JSONL fault-window annotations")
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
    demo.add_argument("--estimator", choices=["wheel-hold", "adaptive-ekf"], default="adaptive-ekf")
    inspect = commands.add_parser("probe", help="Build a review-required source profile draft")
    inspect.add_argument("source")
    inspect.add_argument("--output", required=True)
    inspect.add_argument("--limit", type=int, default=200)
    bench = commands.add_parser("benchmark")
    bench.add_argument("--output", required=True)
    bench.add_argument("--profile", default="contracts/profiles/events.yaml")
    bench.add_argument("--seeds", default="17", help="Comma-separated integer seeds")
    bench.add_argument("--scenario", action="append", choices=SCENARIOS)
    bench.add_argument("--preset", action="append", choices=tuple(PRESETS))
    real = commands.add_parser("real-benchmark", help="Presets x injected faults on real organizer bags")
    real.add_argument("--output", required=True)
    real.add_argument("--data", default="dataset/data")
    real.add_argument("--derived", default="dataset/derived", help="Output of build_reference_table.py")
    real.add_argument("--subset", choices=["identification", "validation", "test"], default="validation")
    real.add_argument("--bag", action="append", help="Explicit bag id (overrides --subset)")
    real.add_argument("--limit", type=int, help="Use only the first N bags of the subset")
    real.add_argument("--window", type=float, default=120.0, help="Seconds of each real-bag excerpt")
    real.add_argument("--scenario", action="append", choices=tuple(REAL_SCENARIOS))
    real.add_argument("--preset", action="append")
    real.add_argument("--model-profile", help="Model YAML; adds the calibrated preset M1-cal")
    real.add_argument("--gnss-mode", action="append", choices=GNSS_MODES)
    one = commands.add_parser("real-run", help="One real-bag run for the console (map, reference, estimate)")
    one.add_argument("--bag", required=True)
    one.add_argument("--scenario", choices=tuple(REAL_SCENARIOS), default="none")
    one.add_argument("--preset", default="M1-cal")
    one.add_argument("--data", default="dataset/data")
    one.add_argument("--derived", default="dataset/derived")
    one.add_argument("--window", type=float, default=120.0)
    one.add_argument("--fault-start", type=float, default=50.0)
    one.add_argument("--fault-len", type=float, default=20.0)
    one.add_argument("--model-profile", default="configs/models/default.yaml")
    one.add_argument("--gnss-mode", choices=GNSS_MODES, default="never")
    one.add_argument("--output", required=True)
    one.add_argument("--run-id")
    one.add_argument("--api", help="Console URL to upload the run to, e.g. http://localhost:8080")
    calib = commands.add_parser("calibrate-noise", help="Grid-search EKF noise/gates on identification bags")
    calib.add_argument("--data", default="dataset/data")
    calib.add_argument("--derived", default="dataset/derived")
    calib.add_argument("--limit", type=int, default=8, help="Identification bags to use")
    calib.add_argument("--profile", action="append", required=True, help="Model YAML to update (repeatable)")
    args = parser.parse_args()
    if args.command == "generate":
        generate(
            args.output, args.seed, args.duration, args.fault, args.fault_start, args.fault_end, args.wheels
        )
    elif args.command == "run":
        model_config = None
        if args.model_config:
            model_config = ModelConfig.from_dict(
                json.loads(Path(args.model_config).read_text(encoding="utf-8"))
            )
        run(
            args.source,
            load_profile(args.profile),
            args.output,
            args.run_id,
            args.tail,
            args.duration,
            args.estimator,
            model_config,
        )
    elif args.command == "evaluate":
        evaluate(args.estimates, args.truth, args.output, args.faults)
    elif args.command == "import-report":
        import_report(args.directory, args.api)
    elif args.command == "serve-ws":
        asyncio.run(serve_file(args.source, args.host, args.port, args.system_time))
    elif args.command == "demo":
        folder = Path(args.output)
        generate(folder)
        run(
            str(folder / "events.jsonl"),
            load_profile(args.profile),
            folder,
            "demo-001",
            estimator_name=args.estimator,
        )
        evaluate(
            folder / "estimates.jsonl",
            folder / "truth.jsonl",
            folder / "report.json",
            folder / "faults.jsonl",
        )
        if args.api:
            import_report(folder, args.api)
    elif args.command == "probe":
        result = write_probe(args.source, args.output, args.limit)
        print("Warnings:", *result["warnings"], sep="\n- ")
    elif args.command == "benchmark":
        seeds = tuple(int(value) for value in args.seeds.split(","))
        benchmark(args.output, args.profile, seeds, args.scenario or SCENARIOS, args.preset or tuple(PRESETS))
    elif args.command == "real-benchmark":
        split = load_split(Path(args.derived) / "split.json")
        bags = args.bag or dedupe(split[args.subset], split["duplicates"])
        if args.model_profile:
            PRESETS["M1-cal"] = ("adaptive-ekf", load_profile_config(args.model_profile))
        real_benchmark(
            bags[: args.limit] if args.limit else bags,
            args.data,
            Path(args.derived) / "reference",
            args.output,
            tuple(args.scenario or REAL_SCENARIOS),
            tuple(args.preset or PRESETS),
            args.window,
            tuple(args.gnss_mode or ("never",)),
        )
    elif args.command == "real-run":
        PRESETS["M1-cal"] = ("adaptive-ekf", load_profile_config(args.model_profile))
        case = build_case(
            Path(args.data) / args.bag, Path(args.derived) / "reference" / f"{args.bag}.csv", args.window
        )
        if case is None:
            raise SystemExit(f"{args.bag}: no usable GNSS-referenced excerpt")
        run_id = args.run_id or f"{args.bag}-{args.scenario}-{args.preset}"
        export_run(
            case,
            args.scenario,
            args.preset,
            args.output,
            run_id,
            args.fault_start,
            args.fault_len,
            args.gnss_mode,
        )
        if args.api:
            import_report(args.output, args.api)
    elif args.command == "calibrate-noise":
        from odometry_io import load_model_config

        split = load_split(Path(args.derived) / "split.json")
        for profile in args.profile:
            base, metadata, _ = load_model_config(profile)
            vehicle = metadata["vehicle_id"]
            candidates = [bag for bag in split["identification"]
                          if vehicle == "default" or bag.startswith(vehicle + "_")]
            bags = dedupe(candidates, split["duplicates"])[: args.limit]
            result = calibrate(bags, args.data, Path(args.derived) / "reference", base=base)
            print(profile, "best:", result["best"], "baseline:", round(result["baseline_objective_rmse_mps"], 4))
            apply_to_profile(profile, result)
    print("Completed:", args.command)


if __name__ == "__main__":
    main()
