from pathlib import Path

from odometry_io import load_profile
from odometry_lab.evaluate import evaluate
from odometry_lab.runner import run
from odometry_lab.scenario import generate
from odometry_lab.storage import read_rows, write_rows

ROOT = Path(__file__).resolve().parents[1]


def test_generator_repeatable_and_truth_is_separate(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    generate(a)
    generate(b)
    assert (a / "events.jsonl").read_bytes() == (b / "events.jsonl").read_bytes()
    assert all("s_m" not in row and "fault" not in row for row in read_rows(a / "events.jsonl"))
    rows = read_rows(a / "events.jsonl")
    assert not any(r["kind"] == "wheel" and 8e9 <= int(r["stamp_ns"]) < 11e9 for r in rows)


def test_full_offline_demo_and_no_truth_metric(tmp_path):
    generate(tmp_path)
    run(
        str(tmp_path / "events.jsonl"),
        load_profile(ROOT / "contracts/profiles/events.yaml"),
        tmp_path,
        "test-run",
    )
    report = evaluate(tmp_path / "estimates.jsonl", tmp_path / "truth.jsonl")
    assert report["metrics"]["sample_count"] == 1001
    assert report["metrics"]["speed_rmse_mps"] >= 0
    assert 0 < report["metrics"]["valid_fraction"] < 1
    assert evaluate(tmp_path / "estimates.jsonl")["metrics"]["speed_rmse_mps"] is None


def test_exact_estimate_and_interpolation(tmp_path):
    rows = [
        {
            "stamp_ns": str(i * 1_000_000_000),
            "v_mps": 10.0,
            "s_m": i * 10.0,
            "valid": True,
            "mode": "FUSED",
            "reason_codes": [],
        }
        for i in range(6)
    ]
    write_rows(tmp_path / "estimate.jsonl", rows)
    write_rows(tmp_path / "truth.jsonl", [rows[0], rows[-1]])
    report = evaluate(tmp_path / "estimate.jsonl", tmp_path / "truth.jsonl")
    assert report["metrics"]["speed_rmse_mps"] == 0
    assert report["metrics"]["position_mae_m"] == 0
    assert report["metrics"]["valid_fraction"] == 1
