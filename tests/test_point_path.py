"""Manual point capture and sequential MovL playback without robot hardware."""

import contextlib
import copy
import io
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import point_path as pp


def point(x):
    return {"joint": [x, 0, 0, 0, 0, 0], "pose": [x, 0, 0, 0, 0, 0]}


class PointPathTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        patcher = patch.object(pp, "POINT_PATH_DIR", Path(directory.name))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.config = {"robot_ip": "192.0.2.1", "user_index": 0, "tool_index": 0,
                       "trajectory_settings": {"max_joint_acceleration_deg_s2": .001,
                                               "linear_timeout_s": .1}}
        self.frames = {"user_index": 0, "tool_index": 0, "user_frame": [0.]*6, "tool_frame": [0.]*6}
        self.data = {"schema_version": 1, "source": pp.POINT_PATH_SOURCE, "complete": True,
                     "feedback_frames": self.frames, "points": [point(0), point(10), point(20)]}

    def write_path(self, data=None):
        path = pp.point_path_file("test")
        path.write_text(json.dumps(self.data if data is None else data), encoding="utf-8")
        return path

    def state(self, x, **changes):
        return dict(point(x), frames=copy.deepcopy(self.frames), joint_velocity=[0.]*6,
                    enable_status=0, drag_status=0, current_command_id=0, running_status=0,
                    received_at=time.monotonic(), **changes)

    @contextlib.contextmanager
    def capture(self, states, keys):
        feedback = Mock()
        feedback.state.side_effect = states

        @contextlib.contextmanager
        def live(config):
            yield feedback

        with patch.object(pp, "LiveFeedback", live), patch("builtins.input", side_effect=keys), \
                patch.object(pp, "connect") as connection, contextlib.redirect_stdout(io.StringIO()):
            yield
        connection.assert_not_called()

    def test_enter_undo_and_exclusive_saved_file(self):
        with self.capture([self.state(0), self.state(10), self.state(20)], ["", "", "u", "", "q"]):
            path = pp.record_point_path("test", self.config)
        data = pp.load_point_path("test", self.config)
        self.assertEqual(data["points"], [point(0), point(20)])
        before = path.read_bytes()
        with patch.object(pp, "LiveFeedback") as live, self.assertRaises(FileExistsError):
            pp.record_point_path("test", self.config)
        live.assert_not_called()
        self.assertEqual(path.read_bytes(), before)

    def test_ctrl_c_saves_complete_points(self):
        with self.capture([self.state(0), self.state(10)], ["", "", KeyboardInterrupt()]):
            pp.record_point_path("test", self.config)
        self.assertEqual(pp.load_point_path("test", self.config)["points"], [point(0), point(10)])

    def test_moving_point_rejected_and_frame_failure_keeps_incomplete_take(self):
        moving = self.state(10)
        moving["joint_velocity"] = [1.]*6
        changed = self.state(20)
        changed["frames"]["tool_frame"][0] = 1
        with self.capture([self.state(0), moving, changed], ["", "", ""]):
            with self.assertRaisesRegex(RuntimeError, "definitions changed"):
                pp.record_point_path("test", self.config)
        data = json.loads(pp.point_path_file("test").read_text(encoding="utf-8"))
        self.assertEqual(data["points"], [point(0)])
        self.assertFalse(data["complete"])
        with self.assertRaisesRegex(ValueError, "incomplete"):
            pp.load_point_path("test", self.config)

    def test_invalid_paths_rejected_before_connection_or_confirmation(self):
        variants = []
        for field, value in (("source", "dobot_v4_feedback_30004"), ("complete", False),
                             ("points", [point(0)])):
            data = copy.deepcopy(self.data)
            data[field] = value
            variants.append(data)
        data = copy.deepcopy(self.data)
        data["points"][1]["pose"][0] = float("nan")
        variants.append(data)
        data = copy.deepcopy(self.data)
        data["feedback_frames"]["tool_index"] = 1
        variants.append(data)
        for data in variants:
            self.write_path(data)
            with self.subTest(data=data), patch.object(pp, "connect") as connect, \
                    patch.object(pp, "_confirm") as confirm:
                with self.assertRaises(ValueError):
                    pp.play_point_path("test", self.config)
                connect.assert_not_called()
                confirm.assert_not_called()
        with self.assertRaises(ValueError):
            pp.point_path_file("../escape")

    @contextlib.contextmanager
    def robot(self, initial=0, never_arrive=False):
        self.current = self.state(initial)
        self.events, self.pending, self.reads = [], None, 0
        connection = Mock(_receive_timeout=90)
        connection._socket.gettimeout.return_value = 90
        dashboard, move, feedback = Mock(connection=connection), Mock(connection=connection), Mock()

        def status():
            if self.pending is not None:
                self.reads += 1
                if self.reads >= 2 and not never_arrive:
                    self.current.update(self.pending, running_status=0,
                                        current_command_id=self.current["current_command_id"]+1)
                    self.events.append("arrived")
                    self.pending = None
            return dict(copy.deepcopy(self.current), received_at=time.monotonic())

        def enable():
            self.current["enable_status"] = 1
            return "0,{},EnableRobot();"

        def movl(pose, coord_type, **kwargs):
            self.assertIsNone(self.pending, "Next target issued before previous arrival")
            self.assertEqual(kwargs, {"user": 0, "tool": 0, "cp": 0})
            self.pending = {"pose": list(pose), "joint": list(pose)}
            self.reads = 0
            self.current["running_status"] = 1
            self.events.append(("MovL", pose[0]))
            return f"0,{{{self.current['current_command_id']+1}}},MovL();"

        for obj, name, action in ((dashboard, "GetPose", lambda: "0,{"+",".join(map(str, self.current["pose"]))+"},GetPose();"),
                                  (dashboard, "SpeedFactor", lambda speed: "0,{},SpeedFactor();"),
                                  (dashboard, "EnableRobot", enable), (move, "MovL", movl),
                                  (dashboard, "Stop", lambda: "0,{},Stop();")):
            command = getattr(obj, name)
            command.__name__ = name
            command.side_effect = action
        feedback.state.side_effect = status

        @contextlib.contextmanager
        def connect(config, for_movement, *, select_frames):
            self.assertTrue(for_movement)
            self.assertFalse(select_frames)
            yield dashboard, move

        @contextlib.contextmanager
        def live(config):
            yield feedback

        self.dashboard, self.move = dashboard, move
        with patch.object(pp, "connect", connect), patch.object(pp, "LiveFeedback", live), \
                patch.object(pp, "_confirm") as confirm, contextlib.redirect_stdout(io.StringIO()):
            yield confirm

    def test_controller_planned_movel_waits_for_every_point(self):
        path = self.write_path()
        before = path.read_bytes()
        with self.robot():
            final = pp.play_point_path("test", self.config, speed=20)
        self.assertEqual(self.events, [("MovL", 0), "arrived", ("MovL", 10), "arrived", ("MovL", 20), "arrived"])
        self.assertEqual(final["pose"], point(20)["pose"])
        self.dashboard.SpeedFactor.assert_called_once_with(20)
        self.move.ServoJ.assert_not_called()
        self.assertEqual(path.read_bytes(), before)

    def test_reverse_and_step_do_not_modify_path(self):
        path = self.write_path()
        before = path.read_bytes()
        with self.robot(initial=20) as confirm:
            pp.play_point_path("test", self.config, reverse=True, step=True)
            self.assertEqual(confirm.call_count, 4)
        self.assertEqual([e[1] for e in self.events if isinstance(e, tuple)], [20, 10, 0])
        self.assertEqual(path.read_bytes(), before)

    def test_far_start_rejected_before_enable(self):
        self.write_path()
        with self.robot(initial=100), self.assertRaisesRegex(RuntimeError, "Not near"):
            pp.play_point_path("test", self.config)
        self.dashboard.EnableRobot.assert_not_called()
        self.move.MovL.assert_not_called()

    def test_unreached_point_aborts_before_next_movel(self):
        self.write_path()
        self.config["trajectory_settings"]["linear_timeout_s"] = .02
        with self.robot(never_arrive=True), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "Endpoint"):
                pp.play_point_path("test", self.config)
        self.assertEqual(self.move.MovL.call_count, 1)
        self.dashboard.Stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
