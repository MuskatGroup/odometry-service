from pathlib import Path

import pytest
from odometry_io import ModelProfileError, load_model_config

PROFILE = """
schema_version: 1
model_version: tram-graybox-test
vehicle_id: default
identified_at_utc: null
identification_method: constrained_least_squares
identification_bags: [bag-a]
validation_bags: [bag-b]
longitudinal:
  tau_s: 0.3
  c1_inv_s: 0.01
  c2_inv_m: 0.001
  disturbance_limit_mps2: 1.5
traction:
  controller_u: [0.1, 1.0]
  speed_mps: [0.0, 10.0]
  acceleration_mps2: [[0.2, 0.1], [1.0, 0.5]]
braking:
  controller_u: [-0.1, -1.0]
  speed_mps: [0.0, 10.0]
  acceleration_mps2: [[-0.1, -0.2], [-0.5, -1.0]]
noise:
  q_v: 0.01
  q_a: 0.05
  q_d: 0.001
  wheel_variance_floor: 0.04
wheel_health:
  gate_normal: 9.0
  gate_reject: 36.0
  max_wheel_accel_mps2: 6.0
  freeze_s: 1.0
  reacquire_s: 10.0
  recover_updates: 5
metrics:
  reference_coverage: null
"""


def test_strict_model_profile_and_default_fallback(tmp_path: Path):
    (tmp_path / "default.yaml").write_text(PROFILE, encoding="utf-8")
    config, metadata, selected = load_model_config(tmp_path, "30618")
    assert selected.name == "default.yaml"
    assert config.model_version == "tram-graybox-test"
    assert config.traction_map.evaluate(1.0, 10.0)[0] == pytest.approx(0.5)
    assert config.braking_map.evaluate(-1.0, 10.0)[0] == pytest.approx(-1.0)
    assert metadata["identification_bags"] == ["bag-a"]


def test_unknown_model_profile_field_is_rejected(tmp_path: Path):
    path = tmp_path / "default.yaml"
    path.write_text(PROFILE + "unexpected: true\n", encoding="utf-8")
    with pytest.raises(ModelProfileError, match="Unknown model profile"):
        load_model_config(path)


def test_wrong_table_dimensions_are_rejected(tmp_path: Path):
    path = tmp_path / "default.yaml"
    path.write_text(PROFILE.replace("[[0.2, 0.1], [1.0, 0.5]]", "[[0.2], [1.0]]"), encoding="utf-8")
    with pytest.raises(ModelProfileError, match="dimensions"):
        load_model_config(path)
