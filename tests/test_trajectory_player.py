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
    return {"schema_version": 1, "source": "dobot_v3_feedback_30004",
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
                            drag_status=0, error_status=0, joint_velocity=[0.]*6)
        self.events = []
        self.socket = Mock()
        self.socket.gettimeout.return_value = 90
        self.dashboard = Mock(socket_dobot=self.socket)
        self.move = Mock(socket_dobot=self.socket)
        self.monitor = Mock()

        def state():
            return dict(copy.deepcopy(self.current), received_at=self.clock()+0.0001)

        self.monitor.state.side_effect = state

        def movl(*pose):
            self.events.append("MovL")
            self.current["pose"] = list(pose)
            self.current["joint"] = list(trajectory.first["joint"])
            return "0,{},MovL();"

        def servoj(*args):
            self.events.append("ServoJ")
            self.current["joint"] = list(args[:6])
            self.current["pose"] = list(args[:6])
            return "0,{},ServoJ();"

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

    def test_move_to_start(self):
        """Already at start: no move. Small offset: one checked correction."""
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for offset, expected in ((0, []), (1, ["MovL", "Sync"])):
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
        self.move.MovL.side_effect = lambda *args: "0,{},MovL();"
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
                self.assertEqual(self.events[:2], ["MovL", "Sync"])
                if align_only:
                    self.assertEqual(self.events, ["MovL", "Sync"])
                    self.assertEqual(state["joint"], trajectory.first["joint"])
                else:
                    self.assertGreater(self.events.count("ServoJ"), 1)
                    self.assertEqual(self.events.count("Sync"), 2)  # alignment + segment end
                    self.assertEqual(self.events[-1], "Sync")
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

                    def slow(*args):
                        result = original(*args)
                        self.clock.now += 0.2
                        return result

                    self.move.ServoJ.side_effect = slow
                elif mode == "tracking":
                    player.limits["max_tracking_error_deg"] = 0.00001
                    self.move.ServoJ.side_effect = lambda *args: "0,{},ServoJ();"
                else:
                    self.monitor.state.side_effect = RuntimeError("stale")
                with self.simulation(), self.assertRaises(RuntimeError):
                    player.play(trajectory)

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
