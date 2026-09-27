import importlib.util
from copy import deepcopy
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "compare_benchmarks", Path(__file__).parents[1] / "tools/compare_benchmarks.py",
)
comparison = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(comparison)


def report():
    return {
        "window_s": 120.0,
        "rows": [{
            "bag": "bag-a", "scenario": "freeze-2", "preset": "M1-cal", "gnss_mode": "never",
            "speed_rmse_mps": 1.0, "position_rmse_m": 20.0, "availability": 0.95,
        }],
    }


def test_compare_pairs_metrics_and_keeps_missing_position_unavailable():
    before, after = report(), report()
    after["rows"][0].update(speed_rmse_mps=0.5, position_rmse_m=None, availability=0.9)
    result = comparison.compare(before, after)
    assert result[0]["speed_rmse_mps"] == {"pairs": 1, "before": 1.0, "after": 0.5}
    assert result[0]["position_rmse_m"] == {"pairs": 0, "before": None, "after": None}
    assert result[0]["availability"]["after"] == 0.9
    assert "n/a → n/a (0/1 pairs)" in comparison.markdown(result)


@pytest.mark.parametrize("field", comparison.KEYS)
def test_compare_rejects_mismatched_experiment_keys(field):
    before, after = report(), report()
    after["rows"][0][field] = "different"
    with pytest.raises(ValueError, match="cohorts"):
        comparison.compare(before, after)


def test_compare_rejects_duplicates_and_different_windows():
    before, after = report(), report()
    after["rows"].append(deepcopy(after["rows"][0]))
    with pytest.raises(ValueError, match="duplicate"):
        comparison.compare(before, after)
    after = report()
    after["window_s"] = 60.0
    with pytest.raises(ValueError, match="window"):
        comparison.compare(before, after)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1.0, True])
def test_compare_rejects_non_metrics(value):
    before, after = report(), report()
    after["rows"][0]["speed_rmse_mps"] = value
    with pytest.raises(ValueError, match="Invalid"):
        comparison.compare(before, after)
