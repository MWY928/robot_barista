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
    def test_speed_two_bounds_and_adaptive_validation(self):
        config = {"trajectory_settings": {"playback_speed": 2}}
        self.assertEqual(player.limits_for(config)["playback_speed"], 2)
        for value in (0, -1, True, float("nan"), float("inf"), 4.01):
            with self.subTest(value=value), self.assertRaises(ValueError):
                player.limits_for({"trajectory_settings": {"playback_speed": value}})
        raw = take()
        before = copy.deepcopy(raw)
        output = smooth.adaptive_retime_recording(raw, config)
        self.assertEqual(raw, before)
        self.assertEqual(config["trajectory_settings"], {"playback_speed": 2})
        self.assertEqual(output["time_processing"]["validated_playback_speed"], 2)
        player.validate_recording(output, config)
        originals = iter(output["samples"])
        for source in raw["samples"]:
            vertex = next(s for s in originals
                          if s.get("controller_timestamp_raw") == source["controller_timestamp_raw"])
            self.assertEqual(vertex["joint"], source["joint"])
            self.assertEqual(vertex["pose"], source["pose"])

    def test_speed_two_does_not_bypass_dynamics(self):
        raw = take()
        raw["samples"][1]["joint"] = [1000]*6
        with self.assertRaisesRegex(ValueError, "speed limit"):
            player.validate_recording(raw, {"trajectory_settings": {"playback_speed": 2}})

    def test_generated_speed_two_trajectory(self):
        config = read_config()
        config["trajectory_settings"]["playback_speed"] = 2
        result = player.load_trajectory("cup_to_coffee_speed2_adaptive_v2", config)
        self.assertEqual(result.data["time_processing"]["validated_playback_speed"], 2)
        self.assertFalse(result.data["replay_verified"])

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
        # Historical fixture is checked at its commissioned offline speed,
        # independent of the operator's current global playback setting.
        config["trajectory_settings"]["playback_speed"] = .2
        self.assertEqual(player.DEFAULT_LIMITS["playback_speed"], .35)
        trajectory = player.load_trajectory("pick_cup_smooth", config)
        self.assertAlmostEqual(trajectory.duration/.2, 49.56103932505357)

    def test_current_cup_to_coffee_speed1(self):
        config = read_config()
        config["trajectory_settings"]["playback_speed"] = 1
        entry = config["trajectories"]["cup_to_coffee"]
        path = Path(entry["file"])
        if not path.is_absolute():
            path = Path(smooth.ROOT)/path
        raw = json.loads(path.read_text())
        before = copy.deepcopy(raw)
        output = smooth.retime_paired_recording(raw, config)
        player.validate_recording(output, config)
        self.assertEqual(raw, before)
        self.assertFalse(output["replay_verified"])
        stretch = output["time_processing"]["uniform_stretch"]
        vertices = [s for s in output["samples"] if "interpolation" not in s]
        self.assertEqual(len(vertices), len(raw["samples"]))
        for original, vertex in zip(raw["samples"], vertices):
            self.assertEqual(vertex, dict(original, t=original["t"]*stretch))
        for endpoint in (0, -1):
            for key in ("joint", "pose"):
                self.assertEqual(output["samples"][endpoint][key], raw["samples"][endpoint][key])

    def test_stretched_gaps_resampled_and_pauses_preserved(self):
        raw = take()
        for index, sample in enumerate(raw["samples"]):
            sample["t"] = index*.19
            sample["joint"] = [0 if index < 5 else 3.0]*6
            sample["pose"] = [0 if index < 5 else .1]*6
        config = {"trajectory_settings": {"playback_speed": 1}}
        output = smooth.retime_paired_recording(raw, config)
        player.validate_recording(output, config)
        self.assertGreater(len(output["samples"]), len(raw["samples"]))
        stretch = output["time_processing"]["uniform_stretch"]
        for sample in output["samples"]:
            if sample["t"] <= raw["samples"][4]["t"]*stretch:
                self.assertEqual(sample["joint"], [0]*6)
            if "interpolation" in sample:
                info = sample["interpolation"]
                a, b = raw["samples"][info["source_segment"]:info["source_segment"]+2]
                for key in ("joint", "pose"):
                    self.assertEqual(sample[key], [x+(y-x)*info["fraction"] for x,y in zip(a[key], b[key])])
                self.assertNotIn("controller_timestamp_raw", sample)

    def test_paired_retime_rejects_bad_raw_and_bounds(self):
        for field, value in (("t", .5), ("t", 0), ("t", float("nan")),
                             ("error_status", 1), ("pose", [float("inf")]*6)):
            raw = take()
            raw["samples"][1][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                smooth.retime_paired_recording(raw, {})
        for bounds in ({"max_samples": 2}, {"stretch_step": .00001},
                       {"max_stretch": 11}):
            with self.assertRaises(ValueError):
                smooth.retime_paired_recording(take(), {}, **bounds)
        raw = take()
        raw["samples"][1]["joint"] = [10000]*6
        with self.assertRaisesRegex(ValueError, "Cannot retime"):
            smooth.retime_paired_recording(raw, {}, max_stretch=1)

    def test_adaptive_retime_keeps_vertices_and_passes_player(self):
        raw = take()
        config = {"trajectory_settings": {"playback_speed": 1}}
        output = smooth.adaptive_retime_recording(raw, config, "adaptive")
        player.validate_recording(output, config)
        self.assertEqual(len(output["samples"]), len(raw["samples"]))
        self.assertLess(output["recorded_duration_s"], raw["samples"][-1]["t"])
        self.assertEqual(output["samples"][0]["joint"], raw["samples"][0]["joint"])
        self.assertEqual(output["samples"][-1]["pose"], raw["samples"][-1]["pose"])
        self.assertEqual(output["time_processing"]["target_speedup"], 1.5)


if __name__ == "__main__":
    unittest.main()
