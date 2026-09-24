import unittest
from odometry_io.core import InputError
from odometry_io.ros_bridge import RosBridge, header_stamp_ns

CONFIG = {
    "profile": {
        "schema_version": "adapter-1", "source_id": "ros",
        "control": {"field": "data", "topic": "/tram/control"},
        "channels": [
            {"id": "w1", "field": "wheels.0.omega", "unit": "rad/s", "radius_m": 0.4, "topic": "/tram/wheels"},
            {"id": "w2", "field": "wheels.1.omega", "unit": "rad/s", "radius_m": 0.4, "topic": "/tram/wheels"}],
    },
    "inputs": [{"topic": "/tram/control", "type": "std_msgs/msg/Float64"},
               {"topic": "/tram/wheels", "type": "custom/msg/Wheels"}],
    "reorder_ms": 10,
}


def header(ns):
    return {"stamp": {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}}


def wheels(ns, omega):
    return {"header": header(ns), "wheels": [{"omega": omega}, {"omega": omega}]}


class RosBridgeTests(unittest.TestCase):
    def test_messages_become_estimates_with_units_and_header_time(self):
        bridge = RosBridge(CONFIG)
        bridge.on_message("/tram/control", {"data": 0.0}, 0)  # no header -> receive time
        frame = None
        for i in range(100):
            t = i * 20_000_000
            self.assertEqual(bridge.on_message("/tram/wheels", wheels(t, 25.0), t + 3_000_000), 2)
            frame = bridge.tick(t + 15_000_000) or frame
        self.assertEqual(frame["mode"], "FUSED")
        self.assertAlmostEqual(frame["v_mps"], 10.0, delta=0.1)  # 25 rad/s * 0.4 m

    def test_sequence_numbers_survive_across_messages(self):
        bridge = RosBridge(CONFIG)
        for i in range(3):
            bridge.on_message("/tram/wheels", wheels(i * 1000, 10.0), 0)
        self.assertEqual(bridge.session.stats["duplicates"], 0)
        self.assertEqual(bridge.counters["/tram/wheels"][("speed", "w1")], 3)

    def test_tick_runs_during_silence_and_never_goes_backwards(self):
        bridge = RosBridge(CONFIG)
        bridge.on_message("/tram/control", {"data": 0.0}, 0)
        bridge.on_message("/tram/wheels", wheels(0, 25.0), 0)
        self.assertTrue(bridge.tick(50_000_000)["valid"])
        self.assertFalse(bridge.tick(2_000_000_000)["mode"] == "FUSED")  # transport is silent
        self.assertIsNone(bridge.tick(2_000_000_000))

    def test_odometry_projection_only_for_valid_estimates(self):
        bridge = RosBridge(CONFIG)
        bridge.on_message("/tram/wheels", wheels(0, 25.0), 0)
        frame = bridge.tick(100_000_000)
        odom = bridge.odometry(frame)
        self.assertEqual(odom["frame_id"], "odom_1d")
        self.assertGreater(odom["var_x"], 0)
        frame = bridge.tick(60_000_000_000)
        self.assertFalse(frame["valid"])
        self.assertIsNone(bridge.odometry(frame))

    def test_config_errors_are_explicit(self):
        bad = {**CONFIG, "profile": {**CONFIG["profile"], "control": {"field": "data", "topic": "/other"}}}
        with self.assertRaises(InputError):
            RosBridge(bad)
        with self.assertRaises(InputError):
            RosBridge(CONFIG).on_message("/nope", {}, 0)
        self.assertIsNone(header_stamp_ns({"data": 1}))


if __name__ == "__main__":
    unittest.main()
