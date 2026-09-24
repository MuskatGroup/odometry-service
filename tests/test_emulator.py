import json
from pathlib import Path
import tempfile
import unittest

from odometry_io.cli import main
from odometry_io.core import InputError, Profile, normalize
from odometry_io.emulator import fault_intervals, simulate, read_emulator
from odometry_io.runner import run
from odometry_io.sources import read_normalized


ROOT = Path(__file__).resolve().parents[1]


class EmulatorTests(unittest.TestCase):
    def test_deterministic_and_truth_not_in_input(self):
        c = {"steps": 10, "seed": 17, "channels": 8}
        first = list(simulate(c))
        self.assertEqual(first, list(simulate(c)))
        self.assertEqual(len(first[0][0]["speeds"]), 8)
        self.assertNotIn("s_m", first[0][0])
        self.assertNotIn("v_mps", first[0][0])

    def test_dropout_does_not_change_truth(self):
        c = {"steps": 10, "dropout_steps": [2, 5], "noise_mps": 0}
        a = list(simulate(c))
        b = list(simulate({**c, "dropout_steps": [0, 0]}))
        self.assertEqual([t for _, t in a], [t for _, t in b])
        self.assertEqual(a[2][0]["speeds"], {})

    def test_emulator_replay_preserves_estimates(self):
        p = Profile(json.loads((ROOT / "examples/profile-emulator.json").read_text()))
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp)
            run(normalize(read_emulator(ROOT / "examples/emulator.json"), p), p, dest / "emulator")
            run(read_normalized(dest / "emulator/normalized.jsonl"), p, dest / "replay")
            self.assertEqual((dest / "emulator/estimates.jsonl").read_bytes(),
                             (dest / "replay/estimates.jsonl").read_bytes())

    def test_cli_switches_source_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in ("csv", "jsonl", "emulator"):
                dest = Path(tmp) / name
                self.assertEqual(main(["--source", str(ROOT / f"examples/source-{name}.json"),
                                       "--output", str(dest)]), 0)
                self.assertTrue((dest / "report.json").is_file())

    def test_invalid_emulator_config(self):
        with self.assertRaises(InputError):
            list(simulate({"step_ns": "0"}))


class FaultTests(unittest.TestCase):
    BASE = {"steps": 40, "noise_mps": 0, "channels": 3, "seed": 1, "initial_speed_mps": 5}

    def speeds(self, faults, **extra):
        return [r["speeds"] for r, _ in simulate({**self.BASE, "faults": faults, **extra})]

    def test_measurement_faults_touch_only_the_reported_wheels(self):
        clean = self.speeds([])
        lock = self.speeds([{"type": "lock", "start": 10, "end": 20, "channels": [1]}])
        self.assertEqual(lock[15]["1"], 0)
        self.assertEqual(lock[15]["0"], clean[15]["0"])
        scale = self.speeds([{"type": "scale", "start": 10, "end": 20, "amount": 1.5}])
        self.assertAlmostEqual(scale[15]["2"], clean[15]["2"] * 1.5)
        slip = self.speeds([{"type": "slip", "start": 10, "end": 20, "channels": [0], "amount": 2}])
        self.assertAlmostEqual(slip[15]["0"], clean[15]["0"] + 2)

    def test_freeze_repeats_the_last_value_while_stamps_continue(self):
        rows = [r for r, _ in simulate({**self.BASE, "control_profile": [[0, 0.5]],
                                        "faults": [{"type": "freeze", "start": 10, "end": 20, "channels": [0]}]})]
        frozen = {r["speeds"]["0"] for r in rows[10:20]}
        self.assertEqual(frozen, {rows[9]["speeds"]["0"]})
        self.assertNotEqual(rows[25]["speeds"]["0"], rows[9]["speeds"]["0"])
        self.assertEqual(len({r["stamp_ns"] for r in rows}), 40)

    def test_dropout_removes_messages_for_the_selected_channels_only(self):
        rows = self.speeds([{"type": "dropout", "start": 5, "end": 8, "channels": [0]}])
        self.assertNotIn("0", rows[6])
        self.assertIn("1", rows[6])

    def test_physics_faults_change_truth_but_measurement_faults_do_not(self):
        base = [t for _, t in simulate({**self.BASE, "faults": []})]
        measured = [t for _, t in simulate({**self.BASE, "faults": [{"type": "scale", "start": 5, "end": 30}]})]
        grade = [t for _, t in simulate({**self.BASE, "faults": [{"type": "grade", "start": 5, "end": 30, "amount": 0.5}]})]
        self.assertEqual(base, measured)
        self.assertGreater(grade[-1]["s_m"], base[-1]["s_m"])

    def test_actuator_lag_changes_the_motion(self):
        fast = [t for _, t in simulate({**self.BASE, "faults": []})][-1]["v_mps"]
        lagged = [t for _, t in simulate({**self.BASE, "faults": [], "actuator_tau_s": 2.0})][-1]["v_mps"]
        self.assertNotEqual(fast, lagged)

    def test_fault_windows_are_exported_in_ns_for_the_evaluator(self):
        windows = fault_intervals({**self.BASE, "step_ns": "50000000",
                                   "faults": [{"type": "dropout", "start": 20, "end": 40}]})
        self.assertEqual(windows[0]["start_ns"], "1000000000")
        self.assertEqual(windows[0]["end_ns"], "2000000000")

    def test_invalid_fault_definitions(self):
        for faults in ([{"type": "nope", "start": 0, "end": 1}], [{"type": "lock", "start": 5, "end": 1}],
                       [{"type": "lock", "start": 0, "end": 1, "channels": [9]}]):
            with self.assertRaises(InputError):
                list(simulate({**self.BASE, "faults": faults}))
        with self.assertRaises(InputError):
            list(simulate({**self.BASE, "control_profile": [[1, 0.2]]}))
        with self.assertRaises(InputError):
            list(simulate({**self.BASE, "control_profile": [[0, 2.0]]}))


if __name__ == "__main__":
    unittest.main()
