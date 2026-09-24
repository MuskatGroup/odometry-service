import json
from pathlib import Path
import tempfile
import unittest
from odometry_io.api import ControlSample, EstimatorConfig, InitialState, OdometryEstimator, WheelSample
from odometry_io.core import Event, InputError, Profile, normalize
from odometry_io.emulator import simulate
from odometry_io.evaluation import evaluate
from odometry_io.model import ModelConfig, ModelEstimator
from odometry_io.runner import run

MS = 1_000_000
STEP = 20 * MS


def profile():
    return Profile({"schema_version": "adapter-1", "source_id": "t",
                    "timestamp": {"field": "t", "unit": "ns"}, "control": {"field": "u"},
                    "channels": [{"id": "a", "field": "a", "unit": "m/s"}, {"id": "b", "field": "b", "unit": "m/s"}]})


class Rig:
    """Drives a ModelEstimator on a 50 Hz grid with a scripted wheel signal."""

    def __init__(self, **config):
        self.est = ModelEstimator(profile(), ModelConfig(**config))
        self.seq = {"c": 0, "a": 0, "b": 0}
        self.t = 0
        self.est.ingest(0, [Event(0, "t", 0, "control", None, 0.0, True)])
        self.seq["c"] = 1

    def tick(self, wheels):
        """wheels: value for both channels, or None to skip them (dropout)."""
        self.t += STEP
        events = []
        if wheels is not None:
            for ch in ("a", "b"):
                events.append(Event(self.t, "t", self.seq[ch], "speed", ch, wheels, True))
                self.seq[ch] += 1
        self.est.ingest(self.t, events)
        return self.est.advance_to(self.t)

    def run(self, seconds, wheels):
        frame = None
        for _ in range(round(seconds * 1e9 / STEP)):
            frame = self.tick(wheels)
        return frame


class ModelTests(unittest.TestCase):
    def test_steady_speed_integrates_position_and_reports_uncertainty(self):
        rig = Rig()
        rig.tick(10.0)
        frame = rig.run(5, 10.0)
        self.assertEqual(frame["mode"], "FUSED")
        self.assertTrue(frame["valid"])
        self.assertAlmostEqual(frame["v_mps"], 10.0, delta=0.05)
        self.assertAlmostEqual(frame["s_m"], 10.0 * 5.0, delta=1.0)
        self.assertEqual(len(frame["covariance_4x4"]), 16)
        self.assertGreater(frame["sigma_s_m"], 0)
        self.assertIn("UNCERTAINTY_UNCALIBRATED", frame["reason_codes"])

    def test_no_estimate_before_first_wheel(self):
        rig = Rig()
        frame = rig.est.advance_to(STEP)
        self.assertEqual(frame["mode"], "INITIALIZING")
        self.assertFalse(frame["valid"])
        self.assertIsNone(frame["s_m"])
        self.assertIsNone(frame["covariance_4x4"])

    def test_dropout_is_model_only_then_invalid_after_horizon(self):
        rig = Rig(max_model_only_s=5.0)
        rig.run(3, 10.0)
        frame = rig.run(1, None)
        self.assertEqual(frame["mode"], "MODEL_ONLY")
        self.assertTrue(frame["valid"])
        self.assertIn("WHEEL_STALE", frame["reason_codes"])
        sigma_early = frame["sigma_s_m"]
        frame = rig.run(2, None)
        self.assertGreater(frame["sigma_s_m"], sigma_early)  # uncertainty grows with the outage
        self.assertGreater(frame["s_m"], 3 * 10.0 - 5)  # prediction continues, not frozen
        frame = rig.run(3, None)
        self.assertEqual(frame["mode"], "INVALID")
        self.assertFalse(frame["valid"])
        self.assertIn("MODEL_ONLY_HORIZON", frame["reason_codes"])
        self.assertIsNotNone(frame["v_mps"])  # finite diagnostic forecast is kept

    def test_slip_is_rejected_and_recovery_needs_a_streak(self):
        rig = Rig(recover_updates=5)
        rig.run(3, 10.0)
        frame = rig.run(0.5, 14.0)
        self.assertEqual(frame["mode"], "DEGRADED")
        self.assertIn("WHEEL_REJECTED", frame["reason_codes"])
        self.assertAlmostEqual(frame["v_mps"], 10.0, delta=0.5)
        self.assertGreater(frame["rejected_wheel_count"], 0)
        frame = rig.run(0.06, 10.0)  # only three good samples: still not trusted
        self.assertEqual(frame["mode"], "DEGRADED")
        frame = rig.run(0.5, 10.0)
        self.assertEqual(frame["mode"], "FUSED")

    def test_locked_wheels_do_not_confirm_a_stop(self):
        rig = Rig()
        rig.run(3, 10.0)
        frame = rig.run(1, 0.0)  # wheels report 0 while the tram keeps rolling
        self.assertGreater(frame["v_mps"], 8.0)
        self.assertIn("WHEEL_REJECTED", frame["reason_codes"])

    def test_braking_never_drives_speed_negative(self):
        est = ModelEstimator(profile(), ModelConfig())
        est.ingest(0, [Event(0, "t", 0, "control", None, -1.0, True), Event(0, "t", 0, "speed", "a", 1.0, True)])
        for i in range(1, 200):
            frame = est.advance_to(i * STEP)
            self.assertGreaterEqual(frame["v_mps"], 0.0)
        self.assertIn(frame["mode"], ("MODEL_ONLY", "INVALID"))

    def test_huge_gap_resets_but_keeps_position(self):
        rig = Rig(max_gap_s=5.0)
        frame = rig.run(2, 10.0)
        position = frame["s_m"]
        frame = rig.est.advance_to(rig.t + 10_000_000_000)
        self.assertEqual(frame["mode"], "INITIALIZING")
        self.assertIn("GAP_RESET", frame["reason_codes"])
        rig.t += 10_000_000_000
        frame = rig.tick(10.0)
        self.assertEqual(frame["mode"], "FUSED")
        self.assertAlmostEqual(frame["s_m"], position, delta=0.5)

    def test_control_timeout_invalidates_when_configured(self):
        rig = Rig(control_timeout_s=1.0)
        rig.run(0.5, 10.0)
        self.assertTrue(rig.run(0.1, 10.0)["valid"])
        frame = rig.run(1.5, 10.0)
        self.assertEqual(frame["mode"], "INVALID")
        self.assertIn("CONTROL_STALE", frame["reason_codes"])

    def test_config_validation(self):
        with self.assertRaises(InputError):
            ModelConfig(tau_s=0)
        with self.assertRaises(InputError):
            ModelConfig(gate_normal=10, gate_reject=5)
        with self.assertRaises(InputError):
            ModelConfig.from_dict({"nonsense": 1})

    def test_deterministic_replay(self):
        def frames():
            rig = Rig()
            return [rig.tick(10.0 + (0.3 if i % 7 == 0 else 0.0)) for i in range(150)]
        self.assertEqual(frames(), frames())


class RobustnessTests(unittest.TestCase):
    """Regression tests for the gate-collapse failure found by the scenario benchmark."""

    def test_sustained_lock_does_not_pull_the_estimate_onto_the_wheels(self):
        rig = Rig()
        rig.run(3, 5.0)
        frame = rig.run(3, 0.0)  # three seconds of locked wheels while the tram keeps rolling
        self.assertGreater(frame["v_mps"], 3.5)
        self.assertEqual(frame["mode"], "DEGRADED")
        self.assertIn("WHEEL_REJECTED", frame["reason_codes"])

    def test_rejected_wheels_cannot_be_trusted_forever_reacquire_after_horizon(self):
        rig = Rig(reacquire_s=2.0, max_model_only_s=2.0)
        rig.run(3, 5.0)
        frame = rig.run(5, 9.0)  # wheels settle on a very different but self-consistent speed
        self.assertAlmostEqual(frame["v_mps"], 9.0, delta=0.5)

    def test_reacquire_is_flagged(self):
        rig = Rig(reacquire_s=1.0)
        rig.run(2, 5.0)
        seen = set()
        for _ in range(150):
            seen.update(rig.tick(9.0)["reason_codes"])
        self.assertIn("WHEEL_REACQUIRED", seen)

    def test_fixed_disturbance_has_no_variance_growth(self):
        adaptive, fixed = Rig(adapt_disturbance=True), Rig()
        adaptive.run(1, 5.0)
        fixed.run(1, 5.0)
        frame_a, frame_f = adaptive.run(20, None), fixed.run(20, None)
        self.assertLess(frame_f["covariance_4x4"][15], 1e-6)
        self.assertGreater(frame_a["covariance_4x4"][15], 0.01)
        self.assertEqual(frame_f["disturbance_mps2"], 0.0)

    def test_ablation_switches(self):
        model_only = Rig(use_wheels=False, max_model_only_s=1e9)
        model_only.tick(5.0)
        frame = model_only.run(3, 20.0)  # wheel corrections are ignored after initialization
        self.assertLess(frame["v_mps"], 6.0)
        plain = Rig(robust=False, adapt_disturbance=False)
        plain.run(1, 5.0)
        self.assertGreater(plain.run(1, 9.0)["v_mps"], 8.0)  # no gate: follows the wheels

    def test_boolean_switches_are_validated(self):
        with self.assertRaises(InputError):
            ModelConfig(robust=1)

    def test_disturbance_adaptation_is_off_by_default(self):
        self.assertFalse(ModelConfig().adapt_disturbance)


class FreezeAndVoteTests(unittest.TestCase):
    def feed(self, est, t, u, a, b):
        events = [Event(t, "t", t // STEP, "control", None, u, True),
                  Event(t, "t", t // STEP, "speed", "a", a, True), Event(t, "t", t // STEP, "speed", "b", b, True)]
        est.ingest(t, events)
        return est.advance_to(t)

    def test_frozen_wheels_are_dropped_while_the_control_expects_a_speed_change(self):
        est = ModelEstimator(profile(), ModelConfig())
        frame = None
        for i in range(60):  # 1.2 s of healthy accelerating data
            v = 3.0 + 0.66 * i * 0.02
            frame = self.feed(est, i * STEP, 0.6, v, v)
        frozen = 3.0 + 0.66 * 59 * 0.02
        for i in range(60, 200):  # both wheels repeat one value while u keeps commanding traction
            frame = self.feed(est, i * STEP, 0.6, frozen, frozen)
        self.assertIn("WHEEL_FROZEN", frame["reason_codes"])
        self.assertEqual(frame["mode"], "MODEL_ONLY")
        self.assertGreater(frame["v_mps"], frozen + 0.5)  # the model keeps accelerating; wheels are not trusted

    def test_steady_coasting_with_identical_readings_is_not_flagged(self):
        est = ModelEstimator(profile(), ModelConfig())
        for i in range(250):
            frame = self.feed(est, i * STEP, 0.0, 5.0, 5.0)  # quantized steady speed, no command
        self.assertNotIn("WHEEL_FROZEN", frame["reason_codes"])
        self.assertEqual(frame["mode"], "FUSED")

    def test_one_frozen_channel_is_excluded_the_other_still_updates(self):
        est = ModelEstimator(profile(), ModelConfig())
        for i in range(60):
            v = 3.0 + 0.66 * i * 0.02
            self.feed(est, i * STEP, 0.6, v, v)
        for i in range(60, 200):
            v = 3.0 + 0.66 * i * 0.02
            frame = self.feed(est, i * STEP, 0.6, 4.0, v)  # channel a is frozen at 4.0
        self.assertIn("WHEEL_FROZEN", frame["reason_codes"])
        self.assertEqual(frame["mode"], "FUSED")
        self.assertAlmostEqual(frame["v_mps"], 3.0 + 0.66 * 199 * 0.02, delta=0.3)

    def test_channel_vote_drops_the_slipping_wheel_even_with_two_channels(self):
        est = ModelEstimator(profile(), ModelConfig())
        for i in range(100):
            self.feed(est, i * STEP, 0.0, 5.0 + 0.001 * i, 5.0 + 0.001 * i)
        for i in range(100, 200):
            frame = self.feed(est, i * STEP, 0.0, 5.0 + 0.001 * i, 7.5)  # b overreads by 50%
        self.assertAlmostEqual(frame["v_mps"], 5.2, delta=0.3)
        self.assertGreater(frame["rejected_wheel_count"], 50)
        self.assertIn("WHEEL_REJECTED", frame["reason_codes"])

    def test_vote_leaves_the_decision_to_the_fused_gate_when_nothing_agrees(self):
        est = ModelEstimator(profile(), ModelConfig())
        for i in range(100):
            self.feed(est, i * STEP, 0.0, 5.0, 5.0)
        frame = self.feed(est, 100 * STEP, 0.0, 8.0, 9.0)  # both channels far from the prediction
        self.assertLess(frame["v_mps"], 6.0)
        self.assertEqual(frame["rejected_wheel_count"], 1)  # rejected as one fused sample, not per channel


class ScenarioTests(unittest.TestCase):
    """Synthetic slip + dropout: the EKF must beat wheel+hold on path error."""

    def evaluate(self, estimator_factory):
        config = {"seed": 17, "steps": 400, "step_ns": "50000000", "channels": 2,
                  "initial_speed_mps": 5, "noise_mps": 0.02, "dropout_steps": [120, 150], "slip_steps": [220, 240]}
        prof = Profile({"schema_version": "adapter-1", "source_id": "synthetic-emulator",
                        "timestamp": {"field": "stamp_ns", "unit": "ns"}, "seq_field": "seq",
                        "control": {"field": "control"},
                        "channels": [{"id": "left", "field": "speeds.0", "unit": "m/s", "missing": "skip"},
                                     {"id": "right", "field": "speeds.1", "unit": "m/s", "missing": "skip"}]})
        pairs = list(simulate(config))
        with tempfile.TemporaryDirectory() as tmp:
            truth = Path(tmp) / "truth.jsonl"
            truth.write_text("\n".join(json.dumps(t) for _, t in pairs), encoding="utf-8")
            out = Path(tmp) / "out"
            run(normalize((r for r, _ in pairs), prof), prof, out, estimator=estimator_factory(prof),
                output_hz=20, tail_s=0)
            return evaluate(out / "estimates.jsonl", truth)

    def test_model_beats_hold_under_slip_and_dropout(self):
        cfg = ModelConfig(kt=1.8, kb=1.8, c1=0.02, v_brake=0.05)
        hold = self.evaluate(lambda p: None)
        model = self.evaluate(lambda p: ModelEstimator(p, cfg))
        self.assertGreater(model["matched_frames"], 300)
        self.assertLess(model["rmse_v_mps"], hold["rmse_v_mps"])
        self.assertLess(model["rmse_s_m"], hold["rmse_s_m"])


class FacadeTests(unittest.TestCase):
    def make(self):
        return OdometryEstimator(EstimatorConfig(wheel_ids=("w1", "w2")))

    def feed(self, est, n=60):
        frames = []
        est.ingest_control(ControlSample(0, 0, 0.0))
        for i in range(n):
            t = i * STEP
            est.ingest_wheel(WheelSample(t, i, "w1", 8.0))
            est.ingest_wheel(WheelSample(t, i, "w2", 8.0))
            frames.append(est.advance_to(t))
        return frames

    def test_contract_round_trip_and_json_safe_stamp(self):
        est = self.make()
        frames = self.feed(est)
        self.assertEqual(frames[-1].mode, "FUSED")
        self.assertEqual(frames[-1].stamp_ns, 59 * STEP)
        self.assertIsInstance(frames[-1].to_dict()["stamp_ns"], str)
        self.assertEqual(frames[-1].accepted_wheel_count, 60)

    def test_reset_reproduces_and_initialize_sets_state(self):
        est = self.make()
        first = self.feed(est)
        est.reset()
        self.assertEqual(first, self.feed(est))
        est.initialize(InitialState(0, 100.0, 8.0))
        est.ingest_control(ControlSample(0, 0, 0.0))
        frame = est.advance_to(STEP)
        self.assertEqual(frame.mode, "MODEL_ONLY")  # initialized, no wheel yet
        self.assertAlmostEqual(frame.s_m, 100.0 + 8.0 * 0.02, delta=0.05)

    def test_rejects_unknown_wheel_and_duplicate_ids(self):
        est = self.make()
        with self.assertRaises(InputError):
            est.ingest_wheel(WheelSample(0, 0, "nope", 1.0))
        with self.assertRaises(InputError):
            OdometryEstimator(EstimatorConfig(wheel_ids=("a", "a")))


if __name__ == "__main__":
    unittest.main()
