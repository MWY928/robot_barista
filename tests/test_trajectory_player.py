"""Seven offline playback checks using synthetic data; use recipe --check-steps for recorded robot motion."""

import copy
import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import robot_common
import trajectory_player as tp
from recipe_common import inspect_workflow

FRAMES = {"user_index": 0, "tool_index": 0, "user_frame": [0.]*6, "tool_frame": [0.]*6}


def recording(start=0, verified=True):
    samples = [{"t": index*0.04, "joint": [start+index*0.004, 0, 0, 0, 0, 0],
                "pose": [start+index*0.004, 0, 0, 0, 0, 0], "error_status": 0}
               for index in range(4)]
    return {"schema_version": 1, "source": tp.RECORDING_SOURCE,
            "stop_reason": "interrupted", "error": None, "replay_verified": verified,
            "feedback_frames": copy.deepcopy(FRAMES), "sample_count": len(samples), "samples": samples}


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now

    def sleep(self, delay):
        self.now += delay


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = {"robot_ip": "192.0.2.1", "user_index": 0, "tool_index": 0,
                       "points": {"HOME": {"joint": [0.]*6, "pose": [0.]*6}},
                       "safe_routes_physically_verified": {"americano": True, "ice_americano": True},
                       "trajectories": {}}
        self.clock = Clock()

    def write_take(self, name="test", data=None):
        data = recording() if data is None else data
        path = Path(self.directory.name) / f"{name}.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        self.config["trajectories"][name] = {"file": str(path)}
        return path

    def rig(self, trajectory, offset=0):
        self.current = copy.deepcopy(trajectory.first)
        self.current["pose"][0] += offset
        self.current.update(frames=copy.deepcopy(FRAMES), enable_status=1,
                            drag_status=0, error_status=0, joint_velocity=[0.]*6,
                            current_command_id=42, running_status=0)
        self.events = []
        self.socket = Mock()
        self.socket.gettimeout.return_value = 90
        self.dashboard = Mock(socket_dobot=self.socket)
        self.move = Mock(socket_dobot=self.socket)
        self.monitor = Mock()

        def state():
            return dict(copy.deepcopy(self.current), received_at=self.clock()+0.0001)

        self.monitor.state.side_effect = state

        def movl(pose, *args, **kwargs):
            self.events.append("MovL")
            self.current["pose"] = list(pose)
            self.current["joint"] = list(trajectory.first["joint"])
            return "0,{42},MovL();"

        def servoj(target, **kwargs):
            self.events.append("ServoJ")
            self.current["joint"] = list(target)
            self.current["pose"] = list(target)
            return "0,{42},ServoJ();"

        for name, side_effect in (("MovL", movl), ("ServoJ", servoj),
                                  ("Sync", lambda: (self.events.append("Sync"), "0,{},Sync();")[1])):
            command = getattr(self.move, name)
            command.__name__ = name
            command.side_effect = side_effect
        self.dashboard.ResetRobot.__name__ = "ResetRobot"
        self.dashboard.ResetRobot.return_value = "0,{},ResetRobot();"
        return tp.TrajectoryPlayer(self.dashboard, self.move, self.config, self.monitor)

    @contextlib.contextmanager
    def simulation(self):
        with patch.object(tp.time, "monotonic", self.clock), patch.object(tp.time, "sleep", self.clock.sleep), \
                contextlib.redirect_stdout(io.StringIO()):
            yield

    @contextlib.contextmanager
    def robot_session(self, player):
        """Substitute only hardware/confirmation so the public runner is exercised."""
        @contextlib.contextmanager
        def connection(*args, **kwargs):
            yield self.dashboard, self.move

        @contextlib.contextmanager
        def feedback(*args, **kwargs):
            yield self.monitor

        self.dashboard.GetPose.__name__ = "GetPose"
        self.dashboard.GetPose.side_effect = lambda: "0,{" + ",".join(map(str, self.current["pose"])) + "},GetPose();"
        for name in ("SpeedFactor", "EnableRobot"):
            command = getattr(self.dashboard, name)
            command.__name__ = name
            command.return_value = f"0,{{}},{name}();"
        with self.simulation(), patch.object(tp, "connect", connection), \
                patch.object(tp, "LiveFeedback", feedback), \
                patch.object(tp, "TrajectoryPlayer", return_value=player), \
                patch.object(tp, "_confirm_motion"):
            yield

    def test_reverse_irregular_timeline_and_no_mutation(self):
        data = recording()
        for sample, t in zip(data["samples"], (0, 0.03, 0.08, 0.12)):
            sample["t"] = t
            sample["controller_timestamp_raw"] = {"value": t + 100}
        original = copy.deepcopy(data)
        source = tp.validate_recording(data, self.config)
        reverse = tp.reverse_trajectory(source, self.config)
        for actual, expected in zip(reverse.times, (0, 0.04, 0.09, 0.12)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(reverse.duration, source.duration)
        self.assertEqual(reverse.data["recorded_duration_s"], source.duration)
        self.assertEqual(reverse.data["sample_count"], 4)
        self.assertFalse(reverse.data["replay_verified"])
        for new, old in zip(reverse.samples, reversed(source.samples)):
            for key in ("joint", "pose", "controller_timestamp_raw"):
                self.assertEqual(new[key], old[key])
        for t in (0, 0.01, 0.04, 0.065, 0.09, 0.12):
            for actual, expected in zip(reverse.joint_at(t), source.joint_at(source.duration-t)):
                self.assertAlmostEqual(actual, expected)
        self.assertEqual(reverse.joint_at(-1), source.last["joint"])
        self.assertEqual(reverse.joint_at(1), source.first["joint"])
        reverse.first["joint"][0] = 123
        reverse.frames["tool_frame"][0] = 123
        reverse.first["controller_timestamp_raw"]["value"] = 123
        self.assertEqual(data, original)

    def test_reverse_dynamics_before_connection(self):
        data = recording()
        data["samples"] = [
            {"t": t, "joint": [q, 0, 0, 0, 0, 0], "pose": [0]*6}
            for t, q in ((0, 0), (0.02, 0.06), (0.05, 0.18))]
        data["sample_count"] = 3
        self.write_take(data=data)
        self.config["trajectory_settings"] = {"playback_speed": 1}
        tp.load_trajectory("test", self.config)  # Forward passes the same limits.
        with patch.object(tp, "connect") as connection, patch.object(tp, "_confirm_motion") as confirm:
            with self.assertRaisesRegex(ValueError, "phase=start"):
                tp.single_trajectory("test", self.config, reverse=True, commissioning=True)
            connection.assert_not_called()
            confirm.assert_not_called()

    def test_reverse_source_protections(self):
        for key, value in (("source", "invalid"), ("error", "failed"), ("sample_count", 99)):
            with self.subTest(key=key):
                data = recording()
                data[key] = value
                self.write_take(data=data)
                with patch.object(tp, "connect") as connection, self.assertRaises(ValueError):
                    tp.single_trajectory("test", self.config, reverse=True, commissioning=True)
                connection.assert_not_called()

    def test_reverse_requires_commissioning_even_if_forward_verified(self):
        for verified in (True, False):
            with self.subTest(verified=verified):
                self.write_take(data=recording(verified=verified))
                with patch.object(tp, "connect") as connection, patch.object(tp, "_confirm_motion") as confirm:
                    with self.assertRaisesRegex(ValueError, "Reverse playback is unverified.*--commissioning"):
                        tp.single_trajectory("test", self.config, reverse=True)
                    connection.assert_not_called()
                    confirm.assert_not_called()
        with self.assertRaisesRegex(ValueError, "replay_verified"):
            tp.single_trajectory("test", self.config)

    def test_reverse_public_runner_and_alignment(self):
        for verified in (True, False):
            for align_only in (True, False):
                with self.subTest(verified=verified, align_only=align_only):
                    path = self.write_take(data=recording(verified=verified))
                    original = path.read_bytes()
                    forward = tp.load_trajectory("test", self.config)
                    reverse = tp.reverse_trajectory(forward, self.config)
                    player = self.rig(reverse, offset=1)
                    with self.robot_session(player), contextlib.redirect_stdout(io.StringIO()) as output:
                        state = tp.single_trajectory("test", self.config, reverse=True,
                                                     commissioning=True, align_only=align_only)
                    self.assertIn("Playback direction: reverse", output.getvalue())
                    self.assertEqual(self.move.MovL.call_args.args[0], forward.last["pose"])
                    self.assertEqual(state["joint"], (forward.last if align_only else forward.first)["joint"])
                    self.assertEqual(path.read_bytes(), original)
                    if align_only:
                        self.move.ServoJ.assert_not_called()
                    else:
                        self.move.ServoJ.assert_called()

    def test_reverse_cli_propagation(self):
        for extra, expected in (([], False), (["--reverse"], True)):
            with patch.object(tp.sys, "argv", ["trajectory_player.py", "test", "--commissioning", "--align-only", *extra]), \
                    patch.object(tp, "read_config", return_value=self.config), \
                    patch.object(tp, "single_trajectory") as runner:
                tp.main()
                runner.assert_called_once_with("test", self.config, commissioning=True,
                                               align_only=True, reverse=expected)

    def test_load_recording(self):
        """A usable take loads; a broken timeline is rejected."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config, require_verified=True)
        self.assertAlmostEqual(trajectory.joint_at(0.02)[0], 0.002)
        data = recording()
        data["samples"][1]["t"] = 0
        self.write_take(data=data)
        with self.assertRaises(ValueError):
            tp.load_trajectory("test", self.config)

    def test_resampled_acceleration_reports_joint_and_phase(self):
        for phase, positions in (("start", [0.0, 0.68, 1.36]),
                                 ("path", [0.0, 0.0, 0.68]),
                                 ("stop", [0.0, 0.34, 1.02])):
            with self.subTest(phase=phase):
                data = recording()
                data["source"] = tp.RECORDING_SOURCE
                data["samples"] = [
                    {"t": index*0.04, "joint": [position, 0, 0, 0, 0, 0],
                     "pose": [0.0]*6, "error_status": 0}
                    for index, position in enumerate(positions)
                ]
                data["sample_count"] = len(data["samples"])
                self.write_take(data=data)
                self.config["trajectory_settings"] = {"playback_speed": 0.25}
                with patch.object(tp, "connect") as connection:
                    with self.assertRaises(ValueError) as failure:
                        tp.single_trajectory("test", self.config, commissioning=True)
                    connection.assert_not_called()
                message = str(failure.exception)
                self.assertIn("J1=106.250 deg/s^2 > 90.000 deg/s^2", message)
                self.assertIn(f"phase={phase}", message)
                self.assertIn("playback_t=", message)
                # Slowing playback passes the same check without raising limits.
                self.config["trajectory_settings"]["playback_speed"] = 0.1
                tp.load_trajectory("test", self.config)

    def test_move_to_start(self):
        """Already at start: no move. Small offset: one checked correction."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for offset, expected in ((0, []), (1, ["MovL"])):
            with self.subTest(offset=offset):
                player = self.rig(trajectory, offset=offset)
                with self.simulation():
                    player.ensure_start(trajectory)
                self.assertEqual(self.events, expected)

    def test_reject_unsafe_start(self):
        """Position, orientation, joints and changed frames can all block motion."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for field, index, value in (("pose", 0, 6), ("pose", 5, 4), ("joint", 0, 6), ("frame", 0, 1)):
            with self.subTest(field=field, index=index):
                player = self.rig(trajectory)
                if field == "frame":
                    self.current["frames"]["tool_frame"][index] = value
                else:
                    self.current[field][index] = value
                with self.assertRaises(RuntimeError):
                    player.ensure_start(trajectory)
                self.assertEqual(self.events, [])
                with self.robot_session(player), self.assertRaises(RuntimeError):
                    tp.single_trajectory("test", self.config, commissioning=True)
                self.dashboard.EnableRobot.assert_not_called()

    def test_alignment_must_arrive(self):
        """An accepted MovL command is insufficient if the arm never arrives."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        player = self.rig(trajectory)
        self.current["joint"][0] += 2
        player.limits["linear_timeout_s"] = 0.03
        self.move.MovL.side_effect = lambda *args, **kwargs: "0,{42},MovL();"
        with self.simulation(), self.assertRaisesRegex(RuntimeError, "Endpoint"):
            player.play(trajectory)
        self.assertNotIn("ServoJ", self.events)

    def test_play_single_trajectory(self):
        """The public runner aligns, plays a real-format take, and returns actual state."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for align_only in (True, False):
            with self.subTest(align_only=align_only):
                player = self.rig(trajectory, offset=1)
                with self.robot_session(player):
                    state = tp.single_trajectory("test", self.config, commissioning=True, align_only=align_only)
                self.assertEqual(self.events[:1], ["MovL"])
                if align_only:
                    self.assertEqual(self.events, ["MovL"])
                    self.assertEqual(state["joint"], trajectory.first["joint"])
                else:
                    self.assertGreater(self.events.count("ServoJ"), 1)
                    self.assertEqual(self.events.count("Sync"), 0)  # V4 uses queue feedback.
                    self.assertEqual(self.events[-1], "ServoJ")
                    self.assertEqual(state["joint"], trajectory.last["joint"])

    def test_abort_unhealthy_playback(self):
        """Late commands, lost tracking or stale feedback interrupt replay."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for mode in ("late", "tracking", "stale"):
            with self.subTest(mode=mode):
                player = self.rig(trajectory)
                if mode == "late":
                    original = self.move.ServoJ.side_effect

                    def slow(*args, **kwargs):
                        result = original(*args, **kwargs)
                        self.clock.now += 0.2
                        return result

                    self.move.ServoJ.side_effect = slow
                elif mode == "tracking":
                    player.limits["max_tracking_error_deg"] = 0.00001
                    self.move.ServoJ.side_effect = lambda *args, **kwargs: "0,{42},ServoJ();"
                else:
                    self.monitor.state.side_effect = RuntimeError("stale")
                with self.simulation(), self.assertRaises(RuntimeError):
                    player.play(trajectory)

    def test_cancel_playback_before_alignment(self):
        self.write_take()
        player = self.rig(tp.load_trajectory("test", self.config), offset=1)
        with self.robot_session(player):
            tp._confirm_motion.side_effect = [None, RuntimeError("Operator aborted")]
            with self.assertRaisesRegex(RuntimeError, "Operator aborted"):
                tp.single_trajectory("test", self.config, commissioning=True)
        self.assertEqual(self.events, [])
        self.move.MovL.assert_not_called()
        self.move.ServoJ.assert_not_called()

    def test_workflow_preflight(self):
        """Adjacent segments must join; a missing file must block connection."""
        self.write_take("a", recording(0))
        self.write_take("b", recording(0.012))
        plan = [("trajectory", "a", "a"), ("trajectory", "b", "b")]
        takes = inspect_workflow(self.config, plan, "americano", report=False)
        self.assertEqual(list(takes), ["a", "b"])
        self.write_take("b", recording(10))
        with self.assertRaisesRegex(ValueError, "junction exceeds"):
            inspect_workflow(self.config, plan, "americano", report=False)
        for script in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            spec = importlib.util.spec_from_file_location("test_recipe", robot_common.ROOT / script)
            recipe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(recipe)
            with patch.object(recipe, "connect") as connect, contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(ValueError):
                    recipe.run(self.config)
                connect.assert_not_called()



if __name__ == "__main__":
    unittest.main()

#python trajectory_player.py pick_cup --commissioning
