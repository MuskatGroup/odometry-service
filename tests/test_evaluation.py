import json
from pathlib import Path
import tempfile
import unittest
from odometry_io.evaluation import evaluate
from odometry_io.core import InputError


class EvaluationTests(unittest.TestCase):
    def test_invalid_frames_are_included_and_missing_truth_is_not_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp)
            predictions = [{"stamp_ns": "0", "v_mps": 3, "s_m": 1, "valid": True},
                           {"stamp_ns": "1", "v_mps": 5, "s_m": 3, "valid": False},
                           {"stamp_ns": "2", "v_mps": 1, "s_m": 0, "valid": True}]
            references = [{"stamp_ns": "0", "v_mps": 1, "s_m": 0},
                          {"stamp_ns": "1", "v_mps": 3, "s_m": 0}]
            est, truth = dest / "estimates.jsonl", dest / "truth.jsonl"
            est.write_text("\n".join(json.dumps(x) for x in predictions))
            truth.write_text("\n".join(json.dumps(x) for x in references))
            result = evaluate(est, truth)
            self.assertEqual(result["rmse_v_mps"], 2)
            self.assertEqual(result["matched_frames"], 2)
            self.assertEqual(result["valid_matched_frames"], 1)
            self.assertEqual(result["estimate_frames"], 3)
            self.assertEqual(result["last_matched_position_error_m"], 3)
            truth.write_text("")
            result = evaluate(est, truth)
            self.assertIsNone(result["rmse_v_mps"])

    def test_reference_duplicates_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp)
            (dest / "est").write_text("")
            row = json.dumps({"stamp_ns": "0", "v_mps": 0, "s_m": 0}) + "\n"
            (dest / "truth").write_text(row * 2)
            with self.assertRaises(InputError):
                evaluate(dest / "est", dest / "truth")
