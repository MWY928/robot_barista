"""Hardware-free tests for teaching and feedback trajectory capture."""

import contextlib
import copy
import importlib.util
import io
import json
import socket
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

import record_trajectory as recorder
import robot_common
import teach_point
from trajectory_player import DEFAULT_LIMITS, LiveFeedback

DTYPE = robot_common.load_sdk().MyType


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class FakeSocket:
    def __init__(self, data, clock, fragment=1440, ending=None):
        self.data = bytearray(data)
        self.clock = clock
        self.fragment = fragment
        self.timeout = 0.1
        self.ending = ending

    def settimeout(self, timeout):
        self.timeout = timeout

    def gettimeout(self):
        return self.timeout

    def recv(self, length):
        if not self.data:
            if self.ending is not None:
                raise self.ending
            self.clock.now += self.timeout
            raise socket.timeout()
        self.clock.now += 0.001
        length = min(length, self.fragment)
        chunk = self.data[:length]
        del self.data[:length]
        return bytes(chunk)


def packet(timestamp=1, **changes):
    frame = np.zeros(1, dtype=DTYPE)
    frame["len"] = DTYPE.itemsize
    frame["TestValue"] = recorder.MAGIC
    frame["TimeStamp"] = timestamp
    frame["QActual"] = [1, 2, 3, 4, 5, 6]
    frame["ToolVectorActual"] = [100, 200, 300, 90, 0, 10]
    for key, value in changes.items():
        frame[key] = value
    return frame.tobytes()


class FeedbackTests(unittest.TestCase):
    config = {"robot_ip": "192.0.2.1", "user_index": 0, "tool_index": 0}

    def capture(self, data, ending=None, **kwargs):
        clock = Clock()
        connection = FakeSocket(data, clock, ending=ending)
        with patch.object(recorder.time, "perf_counter", clock):
            return recorder.capture(connection, DTYPE, self.config, duration=0.05,
                                    rate=125, timeout=0.1, name="test", **kwargs)

    def test_fragmented_and_combined_tcp_packets(self):
        clock = Clock()
        connection = FakeSocket(packet(1) + packet(2), clock, fragment=73)
        with patch.object(recorder.time, "perf_counter", clock):
            reader = recorder.FeedbackReader(connection, DTYPE)
            self.assertEqual(int(reader.read()["TimeStamp"]), 1)
            self.assertEqual(int(reader.read()["TimeStamp"]), 2)

    def test_invalid_header_rejected(self):
        clock = Clock()
        with patch.object(recorder.time, "perf_counter", clock):
            reader = recorder.FeedbackReader(FakeSocket(packet(TestValue=0), clock), DTYPE)
            with self.assertRaisesRegex(ValueError, "header"):
                reader.read()

    def test_truncated_packet_rejected(self):
        clock = Clock()
        connection = FakeSocket(packet()[:100], clock, ending=ConnectionError("closed"))
        with patch.object(recorder.time, "perf_counter", clock):
            with self.assertRaises(ConnectionError):
                recorder.FeedbackReader(connection, DTYPE).read()

    def test_duration_saves_timed_synchronized_samples(self):
        result = self.capture(b"".join(packet(i) for i in range(1, 31)))
        self.assertEqual(result["stop_reason"], "duration")
        self.assertEqual(result["source"], "dobot_v4_feedback_30004")
        self.assertTrue(result["has_multiple_samples"])
        self.assertEqual(result["samples"][0]["t"], 0)
        self.assertEqual(result["samples"][0]["joint"], [1, 2, 3, 4, 5, 6])
        self.assertEqual(result["samples"][0]["pose"], [100, 200, 300, 90, 0, 10])
        self.assertGreater(result["average_rate_hz"], 0)
        self.assertGreater(result["max_sample_gap_s"], 0)
        self.assertLessEqual(result["average_rate_hz"], 125.01)
        json.dumps(result, allow_nan=False)

    def test_ctrl_c_retains_recorded_samples(self):
        result = self.capture(packet(), ending=KeyboardInterrupt())
        self.assertEqual(result["stop_reason"], "interrupted")
        self.assertEqual(result["sample_count"], 1)
        self.assertFalse(result["replay_verified"])

    def test_disconnect_retains_partial_take_with_error(self):
        result = self.capture(packet(), ending=ConnectionError("disconnected"))
        self.assertEqual(result["sample_count"], 1)
        self.assertEqual(result["stop_reason"], "error")
        self.assertIn("disconnected", result["error"])

    def test_wrong_frames_and_nonfinite_values_rejected(self):
        for data in (packet(Tool=1), packet(QActual=[float("nan")] * 6)):
            with self.subTest(data=data[:8]):
                result = self.capture(data)
                self.assertEqual(result["sample_count"], 0)
                self.assertEqual(result["stop_reason"], "error")

    def test_frame_definition_change_stops_recording(self):
        result = self.capture(packet(1) + packet(2, ToolValue=[1] * 6))
        self.assertIn("definitions changed", result["error"])

    def test_live_feedback_decodes_v4_queue_and_motion_state(self):
        monitor = LiveFeedback(self.config)
        clock = Clock()
        monitor.socket = FakeSocket(packet(7, CurrentCommandId=42, RunningStatus=1,
                                           EnableStatus=1, QDActual=[0.1]*6), clock)
        reader = recorder.FeedbackReader(monitor.socket, DTYPE)

        def read_once():
            frame = reader.read()
            monitor.stopping.set()
            return frame

        monitor.reader = Mock(read=read_once)
        with patch.object(recorder.time, "perf_counter", clock):
            monitor._receive()
        state = monitor.state()
        self.assertEqual(state["joint"], [1, 2, 3, 4, 5, 6])
        self.assertEqual(state["pose"], [100, 200, 300, 90, 0, 10])
        self.assertEqual(state["joint_velocity"], [0.1]*6)
        self.assertEqual(state["current_command_id"], 42)
        self.assertEqual(state["running_status"], 1)
        self.assertEqual(state["enable_status"], 1)
        self.assertEqual(state["frames"]["tool_frame"], [0.]*6)

    def test_timestamp_reset_stops_and_duplicates_are_skipped(self):
        result = self.capture(packet(2) + packet(2) + packet(1))
        self.assertEqual(result["sample_count"], 1)
        self.assertIn("backwards", result["error"])

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.json"
            recorder.save_recording({"samples": []}, path)
            before = path.read_bytes()
            with self.assertRaises(FileExistsError):
                recorder.save_recording({"samples": [1]}, path)
            self.assertEqual(path.read_bytes(), before)

    def test_invalid_cli_does_not_connect(self):
        for options in (["take", "--rate", "nan"], ["take", "--duration", "0"],
                        ["take", "--timeout", "inf"], ["../take"]):
            with self.subTest(options=options), patch("sys.argv", ["record_trajectory.py", *options]), \
                    patch.object(recorder.socket, "create_connection") as connect, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    recorder.main()
                connect.assert_not_called()


class TeachingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "points.json"
        self.config = {"robot_ip": "192.0.2.1", "user_index": 0, "tool_index": 0,
                       "points": {"HOME": {"joint": None, "pose": None}},
                       "safe_routes_physically_verified": {"americano": True, "ice_americano": True}}
        robot_common.write_config(self.config, self.path)

    @contextlib.contextmanager
    def connection(self, config, for_movement):
        self.assertFalse(for_movement)
        yield Mock(), None

    def teach(self, answer="SAVE", name="LATTE_START"):
        with patch.object(teach_point, "connect", self.connection), \
                patch.object(teach_point, "read_vector", side_effect=[[1.] * 6, [2.] * 6]), \
                patch("builtins.input", return_value=answer), contextlib.redirect_stdout(io.StringIO()):
            return teach_point.teach_point(name, self.path)

    def test_new_point_shared_settings_and_flags(self):
        self.assertTrue(self.teach())
        saved = robot_common.read_config(self.path)
        self.assertEqual(saved["points"]["LATTE_START"], {"joint": [1.] * 6, "pose": [2.] * 6})
        self.assertEqual(saved["robot_ip"], self.config["robot_ip"])
        self.assertEqual(saved["safe_routes_physically_verified"],
                         {"americano": False, "ice_americano": False})

    def test_cancel_preserves_config(self):
        before = self.path.read_bytes()
        self.assertFalse(self.teach(answer="cancel"))
        self.assertEqual(self.path.read_bytes(), before)

    def test_atomic_write_failure_preserves_config(self):
        before = self.path.read_bytes()
        with patch.object(robot_common.os, "replace", side_effect=OSError("disk failure")):
            with self.assertRaises(OSError):
                self.teach()
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])

    def test_bad_point_name_does_not_connect(self):
        with patch.object(teach_point, "connect") as connect:
            with self.assertRaises(ValueError):
                teach_point.teach_point("../bad", self.path)
            connect.assert_not_called()

    def test_move_saved_point_and_abort_on_rejection(self):
        point = {"joint": [1, 2, 3, 4, 5, 6], "pose": [100, 200, 300, 0, 0, 0]}
        self.config["points"]["HOME"] = point
        robot_common.write_config(self.config, self.path)
        before = self.path.read_bytes()
        for accepted in (True, False):
            with self.subTest(accepted=accepted):
                dashboard, move = Mock(), Mock()
                move.connection._receive_timeout = 90
                for obj, name, reply in (
                    (dashboard, "GetPose", "0,{0,0,0,0,0,0},GetPose();"),
                    (dashboard, "SpeedFactor", "0,{},SpeedFactor();"),
                    (dashboard, "EnableRobot", "0,{},EnableRobot();"),
                    (move, "MovJ", f"{0 if accepted else -1},{{42}},MovJ();"),
                ):
                    command = getattr(obj, name)
                    command.__name__ = name
                    command.return_value = reply
                player = Mock(spec=teach_point.TrajectoryPlayer)
                player.limits = dict(DEFAULT_LIMITS)
                player.state.return_value = {"joint": [0]*6, "pose": [0]*6, "frames": {}}
                feedback = Mock()
                feedback.state.return_value = {"enable_status": 1}

                @contextlib.contextmanager
                def connection(config, for_movement):
                    self.assertTrue(for_movement)
                    yield dashboard, move

                @contextlib.contextmanager
                def live_feedback(config):
                    yield feedback

                with patch.object(teach_point, "connect", connection), \
                        patch.object(teach_point, "LiveFeedback", live_feedback), \
                        patch.object(teach_point, "TrajectoryPlayer", return_value=player), \
                        patch("builtins.input", return_value="Y"), contextlib.redirect_stdout(io.StringIO()):
                    if accepted:
                        teach_point.move_point("HOME", self.path, speed=5)
                        player.wait_for_target.assert_called_once()
                        player.assert_target.assert_called_once_with(point)
                    else:
                        with self.assertRaises(RuntimeError):
                            teach_point.move_point("HOME", self.path)
                        player.wait_for_target.assert_not_called()
                        player.stop_after_error.assert_called_once()
                move.MovJ.assert_called_once_with(point["joint"], robot_common.load_sdk().CoordinateType.JOINT, cp=0)
                self.assertEqual(self.path.read_bytes(), before)

    def test_move_invalid_point_or_cancel_does_not_connect(self):
        with patch.object(teach_point, "connect") as connect:
            with self.assertRaises(ValueError):
                teach_point.move_point("HOME", self.path)  # Untaught point.
            with self.assertRaises(ValueError):
                teach_point.move_point("UNKNOWN", self.path)
            self.config["points"]["HOME"] = {"joint": [0]*6, "pose": [0]*6}
            robot_common.write_config(self.config, self.path)
            with patch("builtins.input", return_value="cancel"), contextlib.redirect_stdout(io.StringIO()):
                self.assertIsNone(teach_point.move_point("HOME", self.path))
            connect.assert_not_called()

    def test_recipes_keep_their_plans_and_delegate_teaching(self):
        for file, steps in (("Americano/nova2_coffee_v1_1.py", 8),
                            ("IceAmericano/nova2_ice_coffee_v1_1.py", 10)):
            spec = importlib.util.spec_from_file_location("recipe_test", robot_common.ROOT / file)
            recipe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(recipe)
            self.assertEqual(len(recipe.PLAN), steps)
            self.assertIs(recipe.connect, robot_common.connect)
            self.assertIs(recipe.teach_point, teach_point.teach_point)
            with patch("sys.argv", [file, "--teach", "HOME"]), \
                    patch.object(recipe, "teach_point") as teach:
                recipe.main()
                teach.assert_called_once_with("HOME")
            config = copy.deepcopy(robot_common.read_config())
            with contextlib.redirect_stdout(io.StringIO()):
                recipe.dry_run(config)


if __name__ == "__main__":
    unittest.main()
