"""A local CLI for switching data sources with the same computation pipeline."""

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys
from .core import InputError, Profile, normalize
from .model import ModelConfig, ModelEstimator
from .runner import run
from .sources import READERS, read_normalized
from .evaluation import evaluate
from .metrics import compute_metrics, load_rows


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Source config with format/input/profile, paths relative to config")
    parser.add_argument("--input", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--format", help="csv/jsonl/emulator/normalized or a registered reader")
    parser.add_argument("--plugin", action="append", default=[], help="Explicit trusted Python module to import for reader/strategy registration")
    parser.add_argument("--truth", type=Path, help="Offline reference JSONL; never passed to estimator")
    parser.add_argument("--faults", type=Path, help="Offline fault windows JSON (faults.json from the emulator); needs --truth")
    parser.add_argument("--estimator", choices=("hold", "model"), default="hold",
                        help="hold = B0 wheel+hold baseline; model = 4-state EKF")
    parser.add_argument("--model-config", type=Path, help="JSON overrides for ModelConfig (model estimator only)")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--output-hz", type=float, default=50)
    parser.add_argument("--reorder-ms", type=float, default=0)
    parser.add_argument("--speed-timeout-s", type=float, default=0.5)
    parser.add_argument("--tail-s", type=float, default=0)
    parser.add_argument("--max-pending", type=int, default=10000)
    parser.add_argument("--max-outputs", type=int, default=1_000_000)
    args = parser.parse_args(argv)
    try:
        for module in args.plugin:
            importlib.import_module(module)
        if args.source:
            if args.input or args.profile or args.format:
                raise InputError("Use --source OR --input/--profile/--format")
            source = json.loads(args.source.read_text(encoding="utf-8-sig"))
            args.format = source["format"]
            args.input = args.source.parent / source["input"]
            args.profile = args.source.parent / source["profile"]
        if not args.input or not args.profile or args.format not in {*READERS, "normalized"}:
            raise InputError("Supply --source or a valid --input, --profile and --format")
        profile = Profile(json.loads(args.profile.read_text(encoding="utf-8-sig")))
        batches = read_normalized(args.input) if args.format == "normalized" else normalize(READERS[args.format](args.input), profile)
        estimator = None
        if args.estimator == "model":
            settings = json.loads(args.model_config.read_text(encoding="utf-8-sig")) if args.model_config else {}
            settings.setdefault("wheel_timeout_s", args.speed_timeout_s)
            estimator = ModelEstimator(profile, ModelConfig.from_dict(settings))
        elif args.model_config:
            raise InputError("--model-config requires --estimator model")
        result = run(batches, profile, args.output, estimator=estimator, output_hz=args.output_hz,
                     reorder_ms=args.reorder_ms, speed_timeout_s=args.speed_timeout_s,
                     tail_s=args.tail_s, max_pending=args.max_pending, max_outputs=args.max_outputs)
        metadata = {"format": args.format, "input_sha256": digest(args.input),
                    "profile_sha256": digest(args.profile), "adapter_version": "adapter-1"}
        (args.output / "source.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        if args.faults and not args.truth:
            raise InputError("--faults requires --truth")
        if args.truth:
            result["accuracy"] = evaluate(args.output / "estimates.jsonl", args.truth)
            result["accuracy_reason"] = result["accuracy"]["reason"]
            result["truth_sha256"] = digest(args.truth)
            faults = json.loads(args.faults.read_text(encoding="utf-8-sig")) if args.faults else None
            result["metrics"] = compute_metrics(load_rows(args.output / "estimates.jsonl"), load_rows(args.truth), faults)
            (args.output / "report.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (InputError, OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(f"Input/run error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
