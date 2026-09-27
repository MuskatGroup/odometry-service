import csv
import importlib.util
from pathlib import Path

import pytest

SOURCE = Path(__file__).parents[1] / "tools/score_gnss_reference.py"
SPEC = importlib.util.spec_from_file_location("score_gnss_reference", SOURCE)
scoring = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(scoring)


def write_csv(path, fields, rows):
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        writer.writerows(rows)


def test_gnss_scoring_excludes_preanchor_positions_and_missing_reference(tmp_path):
    write_csv(tmp_path / "reference.csv", ["stamp_ns", "v_ref", "s_ref", "route_id"], [
        [0, 10, 100, "route"], [100_000_000, 10, 101, "route"],
        [200_000_000, "", "", "route"], [300_000_000, 10, 103, "route"],
    ])
    write_csv(tmp_path / "estimates.csv", ["stamp_ns", "v_mps", "s_m", "valid"], [
        [0, 10, 0, 1], [50_000_000, 11, 100.5, 1], [150_000_000, 999, 999, 0],
    ])
    write_csv(tmp_path / "position.csv", ["stamp_ns", "frame"], [
        [0, "odom"], [50_000_000, "map"], [150_000_000, "map"],
    ])
    result = scoring.score(tmp_path, tmp_path / "reference.csv", "route")
    assert result["speed_all_mps"]["count"] == 2
    assert result["speed_all_mps"]["rmse"] == pytest.approx(2**-0.5)
    assert result["along_track_m"]["count"] == 1
    assert result["along_track_m"]["rmse"] == 0
    assert scoring.score(tmp_path, tmp_path / "reference.csv")["along_track_m"]["rmse"] is None


def test_empty_error_set_is_unavailable_not_zero():
    assert scoring.summarize([])["rmse"] is None
