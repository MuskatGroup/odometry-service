import unittest
from odometry_io.core import InputError
from odometry_io.metrics import Reference, compute_metrics

S = 1_000_000_000


def truth(step_s=0.1, seconds=20, v=2.0):
    return [{"stamp_ns": str(round(i * step_s * S)), "v_mps": v, "s_m": v * i * step_s}
            for i in range(round(seconds / step_s) + 1)]


def estimates(step_s=0.05, seconds=20, v=2.0, offset=0.0, mode="FUSED", valid=True, sigma=0.1, first=0.0):
    rows = []
    for i in range(round(seconds / step_s) + 1):
        t = i * step_s
        rows.append({"stamp_ns": str(round(t * S)), "v_mps": v if t >= first else None,
                     "s_m": v * t + offset if t >= first else None, "sigma_v_mps": sigma, "sigma_s_m": sigma,
                     "mode": mode, "valid": valid, "reason_codes": []})
    return rows


class MetricsTests(unittest.TestCase):
    def test_interpolates_a_coarser_reference_and_reports_zero_error(self):
        m = compute_metrics(estimates(), truth())
        self.assertEqual(m["speed_error_mps"]["rmse"], 0)
        self.assertAlmostEqual(m["path_error_m"]["rmse"], 0, places=9)
        self.assertEqual(m["frames"]["matched"], m["frames"]["total"])
        self.assertEqual(m["availability"]["valid_fraction"], 1)

    def test_origin_is_aligned_once_not_per_frame(self):
        m = compute_metrics(estimates(offset=5.0), truth())
        self.assertAlmostEqual(m["origin_offset_m"], 5.0)
        self.assertAlmostEqual(m["path_error_m"]["final"], 0, places=9)
        drifting = estimates()
        for i, row in enumerate(drifting):
            row["s_m"] += 0.01 * i  # a real drift must remain visible after the one-time alignment
        self.assertGreater(compute_metrics(drifting, truth())["path_error_m"]["final"], 1)

    def test_missing_estimates_and_out_of_range_frames_are_counted_not_zeroed(self):
        rows = estimates(seconds=25, first=3.0)
        m = compute_metrics(rows, truth(seconds=20))
        self.assertGreater(m["frames"]["no_estimate"], 0)
        self.assertGreater(m["frames"]["outside_reference"], 0)
        self.assertEqual(m["frames"]["matched"] + m["frames"]["no_estimate"] + m["frames"]["outside_reference"],
                         m["frames"]["total"])

    def test_coverage_uses_sigma_and_reports_by_mode(self):
        rows = estimates(v=2.5, sigma=0.1)  # constant 0.5 m/s error, far outside 1.96 sigma
        m = compute_metrics(rows, truth())
        self.assertEqual(m["coverage_1_96_sigma"]["overall"]["speed"], 0)
        self.assertIn("FUSED", m["coverage_1_96_sigma"]["by_mode"])
        self.assertEqual(compute_metrics(estimates(sigma=0.1), truth())["coverage_1_96_sigma"]["overall"]["speed"], 1)

    def test_fault_windows_drift_detector_delay_and_horizons(self):
        rows = estimates()
        for row in rows:
            t = int(row["stamp_ns"]) / S
            if 5 <= t < 12:  # dropout: model drifts 0.5 m/s faster than truth, flagged from 5.5 s
                row["v_mps"] = 2.5
                row["s_m"] = 2.0 * 5 + 2.5 * (t - 5) if t >= 5 else row["s_m"]
                row["mode"] = "MODEL_ONLY" if t >= 5.5 else "FUSED"
            elif t >= 12:
                row["s_m"] = 2.0 * t + 3.5  # offset persisted after the outage
        faults = [{"type": "dropout", "start_ns": str(5 * S), "end_ns": str(12 * S)}]
        m = compute_metrics(rows, truth(), faults)["faults"]
        entry = m["entries"][0]
        self.assertAlmostEqual(entry["path_drift_m"], 0.5 * 7, delta=0.1)
        self.assertAlmostEqual(entry["detect_delay_s"], 0.5, delta=0.06)
        self.assertEqual(sorted(entry["horizons"]), ["1", "5"])  # 15 s and 30 s exceed the 7 s window
        self.assertAlmostEqual(entry["horizons"]["5"]["path_drift_m"], 2.5, delta=0.1)
        self.assertAlmostEqual(m["detector"]["recall"], 1, delta=0.1)
        self.assertEqual(m["detector"]["precision"], 1)
        self.assertEqual(entry["recovery_s"], 0)  # FUSED again in the first frame after the window

    def test_baseline_hold_mode_is_not_a_detection(self):
        rows = estimates(mode="BASELINE_HOLD")
        faults = [{"type": "slip", "start_ns": str(5 * S), "end_ns": str(8 * S)}]
        det = compute_metrics(rows, truth(), faults)["faults"]["detector"]
        self.assertEqual(det["flagged_frames"], 0)
        self.assertEqual(det["recall"], 0)

    def test_reference_errors(self):
        with self.assertRaises(InputError):
            Reference(truth()[:2] + truth()[:1])
        self.assertIsNone(Reference(truth()).at(10 ** 15))


if __name__ == "__main__":
    unittest.main()
