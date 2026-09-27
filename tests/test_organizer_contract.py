import importlib.util
from pathlib import Path

import pytest

MODULE = (
    Path(__file__).parents[1]
    / "ros2_ws"
    / "src"
    / "odometry_node"
    / "odometry_node"
    / "organizer.py"
)
SPEC = importlib.util.spec_from_file_location("organizer_contract", MODULE)
organizer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(organizer)


@pytest.mark.parametrize("raw, expected", [(3.6, 1.0), (36.0, 10.0)])
def test_wheel_kmh_to_mps(raw, expected):
    assert organizer.wheel_kmh_to_mps(raw) == pytest.approx(expected)


@pytest.mark.parametrize("raw, expected", [(-15, -1.0), (0, 0.0), (15, 1.0)])
def test_controller_position_to_u(raw, expected):
    assert organizer.controller_position_to_u(raw) == expected


def test_invalid_organizer_values_are_rejected():
    with pytest.raises(ValueError):
        organizer.controller_position_to_u(16)
    with pytest.raises(ValueError):
        organizer.wheel_kmh_to_mps(float("nan"))


def test_bounded_latency_percentiles():
    assert organizer.percentile([], 0.99) is None
    assert organizer.percentile([1, 2, 3, 4, 5], 0.95) == pytest.approx(4.8)
    assert organizer.percentile([1, 2, 3, 4, 100], 0.99) == pytest.approx(96.16)
