import copy
import json
from pathlib import Path
import tempfile
import unittest
from odometry_io.core import Profile, normalize
from odometry_io.emulator import simulate
from odometry_io.probe import main, probe, validate
from odometry_io.runner import run
from odometry_io.sources import read_csv

ROOT = Path(__file__).resolve().parents[1]


def ros_like(n=200, omega=True):
    t0 = 1_790_000_000_000_000_000
    rows = []
    for i in range(n):
        rows.append({"header": {"stamp_ns": t0 + i * 20_000_000}, "controller_notch": [3, 3, 2, 0, 0, -1, -2, -3][i // 25],
                     "wheels": [{"omega_rad_s": 40.0 + (i % 7) * 0.01, "ok": 1},
                                {"omega_rad_s": None if i % 50 == 0 else 40.1, "ok": 0 if i % 50 == 0 else 1}],
                     "avg_wheel_speed_mps": 14.0, "counter": i})
    return rows


class ProbeTests(unittest.TestCase):
    def test_emulator_sample_gives_a_draft_that_runs_end_to_end(self):
        records = [r for r, _ in simulate({"seed": 3, "steps": 200, "channels": 2})]
        profile, notes, todo = probe(records)
        self.assertEqual(profile["timestamp"], {"field": "stamp_ns", "unit": "ns"})
        self.assertEqual(profile["seq_field"], "seq")
        self.assertEqual(profile["control"], {"field": "control"})
        self.assertEqual([c["field"] for c in profile["channels"]], ["speeds.0", "speeds.1"])
        self.assertTrue(all(c["missing"] == "skip" for c in profile["channels"]))
        self.assertIsNone(validate(profile, records))
        with tempfile.TemporaryDirectory() as tmp:
            prof = Profile(copy.deepcopy(profile))
            report = run(normalize(records, prof), prof, Path(tmp) / "out")
            self.assertGreater(report["valid_frames"], 0)

    def test_csv_fixture_finds_categories_units_and_marks_guesses(self):
        records = list(read_csv(ROOT / "examples/input.csv"))
        profile, notes, todo = probe(records)
        self.assertEqual(profile["timestamp"], {"field": "time_s", "unit": "s"})
        self.assertEqual(profile["control"]["mapping"], {"brake": -0.4, "coast": 0.0, "traction": 0.6})
        self.assertEqual({c["field"]: c["unit"] for c in profile["channels"]},
                         {"speed_left": "km/h", "speed_right": "km/h"})
        self.assertTrue(any("control mapping" in t for t in todo))
        self.assertTrue(any("Confirm the unit" in t for t in todo))
        self.assertIsNone(validate(profile, records))

    def test_angular_speed_requires_a_radius_and_never_guesses_one(self):
        records = ros_like()
        profile, notes, todo = probe(records)
        omega = [c for c in profile["channels"] if c["unit"] == "rad/s"]
        self.assertEqual(len(omega), 2)
        self.assertTrue(all(c["radius_m"] is None for c in omega))
        self.assertIn("radius_m", validate(profile, records))
        self.assertTrue(any("radius_m" in t for t in todo))
        for c in omega:
            c["radius_m"] = 0.35
        self.assertIsNone(validate(profile, records))

    def test_nested_lists_flags_notches_and_aggregates(self):
        profile, notes, todo = probe(ros_like())
        self.assertEqual(profile["timestamp"], {"field": "header.stamp_ns", "unit": "ns"})
        self.assertEqual(profile["seq_field"], "counter")
        self.assertEqual(profile["control"], {"field": "controller_notch", "scale": round(1 / 3, 6)})
        flags = {c["field"]: c.get("valid_field") for c in profile["channels"]}
        self.assertEqual(flags["wheels.0.omega_rad_s"], "wheels.0.ok")
        self.assertEqual(flags["wheels.1.omega_rad_s"], "wheels.1.ok")
        aggregate = next(c for c in profile["channels"] if c["field"] == "avg_wheel_speed_mps")
        self.assertEqual(len(aggregate["derived_from"]), 2)
        self.assertEqual(len(profile["fusion"]["channels"]), 2)  # the aggregate is not fused with its parts
        self.assertNotIn(aggregate["id"], profile["fusion"]["channels"])

    def test_fault_flags_are_not_silently_inverted(self):
        rows = [{"t_ms": i * 40, "handle": 0.1, "v_kmh": 30 + i % 3, "fault_left": 0} for i in range(50)]
        profile, notes, todo = probe(rows)
        self.assertNotIn("valid_field", profile["channels"][0])
        self.assertTrue(any("FAULT indicator" in n for n in notes))
        self.assertEqual(profile["timestamp"]["unit"], "ms")

    def test_missing_timestamp_and_speed_are_reported_not_invented(self):
        profile, notes, todo = probe([{"a": "x", "b": "y"}] * 5)
        self.assertIsNone(profile["timestamp"]["field"])
        self.assertEqual(profile["channels"], [])
        self.assertTrue(any("timestamp" in t for t in todo))
        self.assertTrue(any("wheel-speed" in t for t in todo))

    def test_cli_writes_a_loadable_profile_and_signals_incomplete_drafts(self):
        with tempfile.TemporaryDirectory() as tmp:
            good = Path(tmp) / "profile.json"
            self.assertEqual(main([str(ROOT / "examples/input.csv"), "--write", str(good)]), 0)
            Profile(json.loads(good.read_text()))
            bad = Path(tmp) / "ros.jsonl"
            bad.write_text("\n".join(json.dumps(r) for r in ros_like(60)))
            self.assertEqual(main([str(bad), "--write", str(Path(tmp) / "ros-profile.json")]), 1)


if __name__ == "__main__":
    unittest.main()
