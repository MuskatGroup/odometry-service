import copy
from decimal import Decimal
import json
import math
from pathlib import Path
import tempfile
import unittest

from odometry_io import Event, InputError, Profile, normalize, register_reader, register_strategy, run
from odometry_io.fusion import median_strategy
from odometry_io.runner import ordered_groups
from odometry_io.sources import READERS, read_csv, read_jsonl, read_normalized


ROOT = Path(__file__).resolve().parents[1]


def config():
    return {"schema_version": "adapter-1", "source_id": "test",
            "timestamp": {"field": "t", "unit": "s"},
            "channels": [{"id": "wheel", "field": "v", "unit": "m/s"}],
            "fusion": {"strategy": "median", "variance_floor": 0.25}}


def speed(t, seq=0, value=10, channel="wheel", valid=True):
    return Event(t, "test", seq, "speed", channel, value, valid)


class AdapterTests(unittest.TestCase):
    def test_timestamp_keeps_nanoseconds(self):
        p = Profile(config())
        self.assertEqual(p.timestamp("1720000000.123456789"), 1720000000123456789)
        with self.assertRaises(InputError):
            p.timestamp(1720000000.1234567)
        with self.assertRaises(InputError):
            p.timestamp("0.0000000001")

    def test_units_and_nested_array(self):
        c = config()
        c["channels"] = [{"id": "wheel", "field": "w.0.rpm", "unit": "rpm", "radius_m": 0.5}]
        events = list(normalize([{"t": "0", "w": [{"rpm": 60}]}], Profile(c)))
        self.assertAlmostEqual(events[0][0].value, math.pi)

    def test_unknown_units_and_missing_radius_fail_before_data(self):
        for unit in ("mph", "rpm"):
            c = config()
            c["channels"][0]["unit"] = unit
            with self.assertRaises(InputError):
                Profile(c)

    def test_control_mapping_and_missing_measurement(self):
        c = config()
        c["control"] = {"field": "u", "mapping": {"brake": -0.5}}
        c["channels"][0]["missing"] = "skip"
        batches = list(normalize([{"t": "0", "u": "brake", "v": 0},
                                  {"t": "1", "u": "brake", "v": None}], Profile(c)))
        self.assertEqual(batches[0][1].value, 0)
        self.assertEqual(len(batches[1]), 1)
        self.assertEqual(batches[0][0].value, -0.5)
        with self.assertRaises(InputError):
            list(normalize([{"t": "0", "u": "unknown", "v": 0}], Profile(c)))

    def test_finite_numbers_and_validity(self):
        for value in ("nan", "inf", True):
            with self.assertRaises(InputError):
                list(normalize([{"t": "0", "v": value}], Profile(config())))
        c = config()
        c["channels"][0]["valid_field"] = "ok"
        e = list(normalize([{"t": "0", "v": 0, "ok": "false"}], Profile(c)))[0][0]
        self.assertFalse(e.input_valid)
        with self.assertRaises(InputError):
            list(normalize([{"t": "0", "v": None, "ok": True}], Profile(c)))

    def test_derived_aggregate_cannot_double_count(self):
        c = config()
        c["channels"].append({"id": "aggregate", "field": "agg", "unit": "m/s", "derived_from": ["wheel"]})
        with self.assertRaises(InputError):
            Profile(c)
        c["fusion"]["channels"] = ["aggregate"]
        self.assertEqual(Profile(c).selected, {"aggregate"})

    def test_no_automatic_variance_reduction(self):
        settings = {"variance_floor": 0.25}
        for count in (1, 2, 4, 8):
            result = median_strategy([speed(0, channel=str(i)) for i in range(count)], settings)
            self.assertEqual(result.variance, 0.25)
            self.assertEqual(result.speed_mps, 10)

    def test_source_registry_extension(self):
        register_reader("test_memory", lambda path: iter([{"t": "0", "v": 5}]))
        e = list(normalize(READERS["test_memory"](None), Profile(config())))[0][0]
        self.assertEqual(e.value, 5)
        with self.assertRaises(InputError):
            register_reader("csv", lambda path: [])

    def test_same_time_groups_and_conflicting_duplicates(self):
        stats = {"duplicates": 0, "late_events": 0}
        e = speed(0)
        groups = list(ordered_groups([[e], [e], [speed(1, 1)]], stats))
        self.assertEqual(stats["duplicates"], 1)
        self.assertEqual(len(groups), 2)
        with self.assertRaises(InputError):
            list(ordered_groups([[e], [speed(0, value=20)]], stats))

    def test_late_input_and_bounded_reordering(self):
        batches = [[speed(0, 0)], [speed(20, 2)], [speed(10, 1)], [speed(30, 3)]]
        stats = {"duplicates": 0, "late_events": 0}
        self.assertEqual([g[0] for g in ordered_groups(batches, stats, reorder_ns=15)], [0, 10, 20, 30])
        batches = [[speed(0, 0)], [speed(20, 2)], [speed(30, 3)], [speed(10, 1)]]
        stats = {"duplicates": 0, "late_events": 0}
        self.assertEqual([g[0] for g in ordered_groups(batches, stats)], [0, 20, 30])
        self.assertEqual(stats["late_events"], 1)

    def test_same_timestamp_separate_packets_merge(self):
        stats = {"duplicates": 0, "late_events": 0}
        groups = list(ordered_groups([[speed(0, channel="a")], [speed(0, channel="b")]], stats))
        self.assertEqual(len(groups), 1)
        self.assertEqual(len(groups[0][1]), 2)

    def test_resource_limit(self):
        with self.assertRaises(InputError):
            list(ordered_groups([[speed(0, channel=str(i)) for i in range(3)]],
                                {"duplicates": 0, "late_events": 0}, max_pending=2))

    def test_csv_jsonl_replay_equivalence(self):
        p = Profile(json.loads((ROOT / "examples/profile.json").read_text()))
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            run(normalize(read_csv(ROOT / "examples/input.csv"), p), p, base / "csv")
            run(normalize(read_jsonl(ROOT / "examples/input.jsonl"), p), p, base / "jsonl")
            run(read_normalized(base / "csv/normalized.jsonl"), p, base / "replay")
            expected = (base / "csv/estimates.jsonl").read_bytes()
            self.assertEqual(expected, (base / "jsonl/estimates.jsonl").read_bytes())
            self.assertEqual(expected, (base / "replay/estimates.jsonl").read_bytes())
            rows = [json.loads(line) for line in expected.splitlines()]
            self.assertAlmostEqual(rows[-1]["s_m"], 20)
            self.assertTrue(any(row["mode"] == "INVALID" for row in rows))
            self.assertTrue(rows[-1]["valid"])

    def test_ten_mps_for_five_seconds_and_tail_timeout(self):
        with tempfile.TemporaryDirectory() as temp:
            dest = Path(temp) / "run"
            report = run([[speed(0)]], Profile(config()), dest, tail_s=5)
            rows = [json.loads(line) for line in (dest / "estimates.jsonl").read_text().splitlines()]
            self.assertAlmostEqual(rows[-1]["s_m"], 50)
            self.assertFalse(rows[-1]["valid"])
            self.assertIsNone(rows[-1]["sigma_v_mps"])
            self.assertIsNone(report["accuracy"])
            with self.assertRaises(InputError):
                run([[speed(0)]], Profile(config()), dest)

    def test_selected_channel_strategy(self):
        c = config()
        c["channels"].append({"id": "other", "field": "other", "unit": "m/s"})
        c["fusion"] = {"strategy": "selected_channel", "channel": "wheel", "variance_floor": 1}
        with tempfile.TemporaryDirectory() as temp:
            dest = Path(temp) / "run"
            run([[speed(0, value=7), speed(0, channel="other", value=100)]], Profile(c), dest)
            self.assertEqual(json.loads((dest / "estimates.jsonl").read_text())["v_mps"], 7)

    def test_invalid_packet_revokes_freshness(self):
        with tempfile.TemporaryDirectory() as temp:
            dest = Path(temp) / "run"
            run([[speed(0)], [speed(20_000_000, 1, None, valid=False)]], Profile(config()), dest)
            rows = [json.loads(line) for line in (dest / "estimates.jsonl").read_text().splitlines()]
            self.assertFalse(rows[-1]["valid"])

    def test_unknown_normalized_channel_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(InputError):
                run([[speed(0, channel="unexpected")]], Profile(config()), Path(temp) / "run")

    def test_normalized_decimal_roundtrip(self):
        event = Event.from_dict({**speed(0).to_dict(), "value": Decimal("1.25")})
        self.assertIn("1.25", json.dumps(event.to_dict()))


if __name__ == "__main__":
    unittest.main()
