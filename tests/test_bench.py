import json
from pathlib import Path
import tempfile
import unittest
from odometry_io.bench import PRESETS, base_config, main, scenarios
from odometry_io.core import InputError
from odometry_io.emulator import fault_intervals


class BenchTests(unittest.TestCase):
    def test_every_scenario_defines_valid_fault_windows(self):
        for name, faults in scenarios().items():
            windows = fault_intervals({**base_config(1), "faults": faults})
            self.assertEqual(len(windows), len(faults), name)

    def test_small_run_writes_reports_and_criteria(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "b"
            self.assertEqual(main(["--output", str(out), "--seeds", "1", "--scenario", "clean",
                                   "--scenario", "lock", "--preset", "B0", "--preset", "M2"]), 0)
            data = json.loads((out / "bench.json").read_text())
            self.assertEqual({(r["scenario"], r["preset"]) for r in data["rows"]},
                             {(s, p) for s in ("clean", "lock") for p in ("B0", "M2")})
            lock = {r["preset"]: r for r in data["rows"] if r["scenario"] == "lock"}
            self.assertLess(lock["M2"]["rmse_v_mps"], lock["B0"]["rmse_v_mps"])  # gate must survive wheel lock
            self.assertIn("clean_rmse_v_not_worse", data["criteria"])
            self.assertIn("| lock | M2 |", (out / "bench.md").read_text())
            with self.assertRaises(InputError):
                main(["--output", str(out)])  # never overwrite

    def test_presets_cover_the_documented_variants(self):
        self.assertEqual(set(PRESETS), {"B0", "B1", "B2", "M1", "M2"})


if __name__ == "__main__":
    unittest.main()
