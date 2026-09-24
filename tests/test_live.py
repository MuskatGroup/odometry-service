import unittest
from odometry_io.core import Event, InputError, Profile
from odometry_io.live import LiveSession, replay_actions


def profile():
    return Profile({"schema_version": "adapter-1", "source_id": "emulator",
                    "timestamp": {"field": "t", "unit": "ns"},
                    "channels": [{"id": "a", "field": "v", "unit": "m/s"}]})


def sample(stamp=0, seq=0):
    return Event(stamp, "emulator", seq, "speed", "a", 10, True)


class LiveTests(unittest.TestCase):
    def test_silent_transport_times_out_and_replays(self):
        journal, frames = [], []
        session = LiveSession(profile(), journal=journal.append)
        session.ingest([sample()])
        frames.append(session.advance_to(0))
        frames.append(session.advance_to(1_000_000_000))
        self.assertTrue(frames[0]["valid"])
        self.assertFalse(frames[-1]["valid"])
        self.assertEqual(frames[-1]["s_m"], 10)
        self.assertEqual(frames, list(replay_actions(journal, LiveSession(profile()))))

    def test_future_packets_wait_and_late_packets_are_rejected(self):
        session = LiveSession(profile())
        session.ingest([sample(100, 1)])
        self.assertEqual(session.advance_to(50)["mode"], "INITIALIZING")
        session.ingest([sample(0, 0)])
        self.assertEqual(session.stats["late_events"], 1)
        self.assertTrue(session.advance_to(100)["valid"])

    def test_clock_reset_is_explicit(self):
        session = LiveSession(profile())
        session.advance_to(20)
        with self.assertRaises(InputError):
            session.advance_to(10)

    def test_queue_limit_and_duplicates(self):
        session = LiveSession(profile(), max_pending=1)
        session.ingest([sample()])
        session.ingest([sample()])
        self.assertEqual(session.stats["duplicates"], 1)
        with self.assertRaises(InputError):
            session.ingest([sample(10, 1)])
