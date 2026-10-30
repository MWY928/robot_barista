"""Offline regressions: no connections permitted."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import record_trajectory as recorder
import smooth_trajectory as smooth
import trajectory_player as player
from robot_common import read_config


def take():
    return {"schema_version": 1, "source": recorder.RECORDING_SOURCE,
            "stop_reason": "duration", "error": None, "replay_verified": False,
            "feedback_frames": {"user_index": 0, "tool_index": 0,
                                "user_frame": [0]*6, "tool_frame": [0]*6},
            "sample_count": 11,
            "samples": [{"t": i*.02, "joint": [i*.001]*6,
                         "pose": [i*.002]*6, "error_status": 0,
                         "controller_timestamp_raw": i} for i in range(11)]}


class RetimingTests(unittest.TestCase):
    def setUp(self):
        guard = patch("socket.create_connection", side_effect=AssertionError("No hardware"))
        guard.start()
        self.addCleanup(guard.stop)

    def test_pairs_status_endpoints_and_input_preserved(self):
        raw = take()
        before = copy.deepcopy(raw)
        output = smooth.retime_recording(raw, {})
        self.assertEqual(raw, before)
        self.assertFalse(output["replay_verified"])
        self.assertAlmostEqual(output["samples"][-1]["t"], .3)
        for a, b in zip(raw["samples"], output["samples"]):
            self.assertEqual({k:v for k,v in a.items() if k != "t"},
                             {k:v for k,v in b.items() if k != "t"})
        player.validate_recording(output, {})

    def test_invalid_raw_not_sanitized(self):
        variants = []
        for field, value in (("error", "disconnect"), ("stop_reason", "error"),
                             ("sample_count", 99), ("source", "V3")):
            data = take(); data[field] = value; variants.append(data)
        for field, value in (("t", 0), ("t", .5), ("t", float("nan")),
                             ("error_status", 1), ("joint", [float("inf")]*6)):
            data = take(); data["samples"][1][field] = value; variants.append(data)
        for data in variants:
            with self.subTest(data=data), self.assertRaises(ValueError):
                smooth.retime_recording(data, {})

    def test_exclusive_raw_and_final_save(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"take.json"
            raw = take()
            result = recorder.process_and_save(raw, path, {})
            self.assertEqual(json.loads(path.read_text()), result)
            self.assertEqual(json.loads(path.with_name("take_raw.json").read_text()), raw)
            with self.assertRaises(FileExistsError):
                recorder.process_and_save(raw, path, {})

    def test_failed_capture_only_raw_backup(self):
        for reason in ("disconnect", "one_sample"):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"take.json"
                raw = take()
                if reason == "disconnect":
                    raw.update(error="disconnect", stop_reason="error")
                else:
                    raw["samples"] = raw["samples"][:1]
                    raw["sample_count"] = 1
                with self.assertRaises(ValueError):
                    recorder.process_and_save(raw, path, {})
                self.assertFalse(path.exists())
                self.assertEqual(json.loads(path.with_name("take_raw.json").read_text()), raw)

    def test_unretimable_gap_or_dynamics_rejected(self):
        raw = take()
        raw["samples"][1]["joint"] = [10000]*6
        with self.assertRaisesRegex(ValueError, "Cannot retime"):
            smooth.retime_recording(raw, {})

    def test_existing_smooth_passes_new_config(self):
        config = read_config()
        self.assertEqual(config["trajectory_settings"]["playback_speed"], .35)
        self.assertEqual(player.DEFAULT_LIMITS["playback_speed"], .35)
        trajectory = player.load_trajectory("pick_cup_smooth", config)
        self.assertAlmostEqual(trajectory.duration/.35, 28.320593900030612)


if __name__ == "__main__":
    unittest.main()
