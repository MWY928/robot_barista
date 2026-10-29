"""Synthetic recordings and robot doubles only; never connect to hardware."""

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

    def test_orientation_wrap_and_equivalent_euler_angles(self):
        self.assertAlmostEqual(tp.orientation_error([0, 0, 179], [0, 0, -179]), 2)
        self.assertAlmostEqual(tp.orientation_error([0, 90, 0], [90, 90, 90]), 0, places=5)

    def test_all_margin_components_and_joint_revolutions(self):
        state = {"pose": [0]*6, "joint": [0]*6}
        limits = tp.limits_for(self.config)
        for field, index, value in (("pose", 0, 6), ("pose", 5, 4), ("joint", 3, 6), ("joint", 5, 360)):
            target = copy.deepcopy(state)
            target[field][index] = value
            self.assertFalse(tp.within(tp.state_errors(state, target), limits))

    def test_real_recording_format_and_interpolation(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config, require_verified=True)
        self.assertAlmostEqual(trajectory.joint_at(0.02)[0], 0.002)
        self.assertEqual(trajectory.joint_at(100), trajectory.last["joint"])

    def test_invalid_takes_rejected_before_connecting(self):
        mutations = [lambda d: d.update(stop_reason="error"),
                     lambda d: d.update(replay_verified=False),
                     lambda d: d.update(sample_count=99),
                     lambda d: d["feedback_frames"].update(tool_index=1),
                     lambda d: d["samples"][1].update(t=0),
                     lambda d: d["samples"][1].update(t=100),
                     lambda d: d["samples"][1].update(joint=[float("nan")]*6),
                     lambda d: d["samples"][1].update(joint=[200]*6),
                     lambda d: d["samples"][1].update(error_status=1)]
        for mutate in mutations:
            data = recording()
            mutate(data)
            self.write_take(data=data)
            with self.subTest(mutation=mutate), self.assertRaises(ValueError):
                tp.load_trajectory("test", self.config, require_verified=True)

    def test_inspection_follows_plan_and_rejects_large_junction(self):
        self.write_take("a", recording(0))
        self.write_take("b", recording(10))
        plan = [("trajectory", "a", "a"), ("cup_signal", None, "wait"), ("trajectory", "b", "b")]
        with self.assertRaisesRegex(ValueError, "junction exceeds"):
            inspect_workflow(self.config, plan, "americano", report=False)
        self.write_take("b", recording(0.012))
        takes = inspect_workflow(self.config, plan, "americano", report=False)
        self.assertEqual(list(takes), ["a", "b"])

    def test_small_error_aligns_once_then_checks_again(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        player = self.rig(trajectory, offset=1)
        with patch.object(tp.time, "monotonic", self.clock), patch.object(tp.time, "sleep", self.clock.sleep), \
                contextlib.redirect_stdout(io.StringIO()):
            player.ensure_start(trajectory)
        self.assertEqual(self.events, ["MovL", "Sync"])

    def test_large_error_or_changed_frames_does_not_move(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for offset, frame_change in ((6, False), (0, True)):
            player = self.rig(trajectory, offset=offset)
            if frame_change:
                self.current["frames"]["tool_frame"][0] = 10
            with self.assertRaises(RuntimeError):
                player.ensure_start(trajectory)
            self.assertEqual(self.events, [])

    def test_exact_start_needs_no_alignment(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        player = self.rig(trajectory)
        player.ensure_start(trajectory)
        self.assertEqual(self.events, [])

    def test_continuous_servo_playback_has_only_final_sync(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        player = self.rig(trajectory)
        with patch.object(tp.time, "monotonic", self.clock), patch.object(tp.time, "sleep", self.clock.sleep):
            player.play(trajectory)
        self.assertGreater(self.events.count("ServoJ"), 1)
        self.assertEqual(self.events.count("Sync"), 1)
        self.assertEqual(self.events[-1], "Sync")
        self.assertNotIn("MovL", self.events)
        self.assertEqual(self.current["joint"], trajectory.last["joint"])

    def test_late_ack_or_tracking_error_aborts(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        for mode in ("late", "tracking", "stale"):
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
            with patch.object(tp.time, "monotonic", self.clock), patch.object(tp.time, "sleep", self.clock.sleep):
                with self.subTest(mode=mode), self.assertRaises(RuntimeError):
                    player.play(trajectory)

    def test_endpoint_failure_is_not_treated_as_success(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        player = self.rig(trajectory)
        self.current["pose"][0] = 100
        player.limits["endpoint_timeout_s"] = 0.03
        with patch.object(tp.time, "monotonic", self.clock), patch.object(tp.time, "sleep", self.clock.sleep):
            with self.assertRaisesRegex(RuntimeError, "Endpoint"):
                player.wait_for_target(trajectory.last)

    def test_alignment_must_reach_recorded_joints_before_replay(self):
        self.write_take()
        trajectory = tp.load_trajectory("test", self.config)
        player = self.rig(trajectory)
        self.current["joint"][0] += 2
        player.limits["linear_timeout_s"] = 0.03
        self.move.MovL.side_effect = lambda *args: (self.events.append("MovL"), "0,{},MovL();")[1]
        with patch.object(tp.time, "monotonic", self.clock), patch.object(tp.time, "sleep", self.clock.sleep), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "Endpoint"):
                player.play(trajectory)
        self.assertNotIn("ServoJ", self.events)

    def test_feedback_age_and_error_gate(self):
        monitor = tp.LiveFeedback(self.config)
        monitor.latest = {"received_at": self.clock()-1, "error_status": 0}
        with patch.object(tp.time, "monotonic", self.clock):
            with self.assertRaisesRegex(RuntimeError, "stale"):
                monitor.state()
            monitor.latest.update(received_at=self.clock(), error_status=1)
            with self.assertRaisesRegex(RuntimeError, "reports an error"):
                monitor.state()

    def test_limits_validation_and_verification_gates(self):
        for setting in ({"start_position_margin_mm": float("nan")}, {"playback_speed": 2},
                        {"servo_period_s": 0.001}, {"arrival_joint_deg": 10}):
            with self.subTest(setting=setting), self.assertRaises(ValueError):
                tp.limits_for(dict(self.config, trajectory_settings=setting))
        self.write_take("a", recording(verified=False))
        plan = [("trajectory", "a", "test")]
        inspect_workflow(self.config, plan, "americano", require_verified=False, report=False)
        with self.assertRaisesRegex(ValueError, "replay_verified"):
            inspect_workflow(self.config, plan, "americano", require_verified=True, report=False)

    def test_full_recipe_keeps_waits_and_commissioning_confirmation(self):
        for script in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            spec = importlib.util.spec_from_file_location("full_recipe", robot_common.ROOT / script)
            recipe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(recipe)
            for commissioning in (False, True):
                events = []
                player = Mock(spec=tp.TrajectoryPlayer)
                player.limits = {"endpoint_timeout_s": 5}
                player.play.side_effect = lambda take: events.append(take.name)
                player.move_linear.side_effect = lambda target: events.append("MovL")
                takes = {name: Mock(name=name) for action,name,_ in recipe.PLAN if action == "trajectory"}
                for name,take in takes.items():
                    take.name = name
                dash = Mock()
                dash.SpeedFactor.__name__ = "SpeedFactor"
                dash.SpeedFactor.return_value = "0,{},SpeedFactor();"
                dash.EnableRobot.__name__ = "EnableRobot"
                dash.EnableRobot.return_value = "0,{},EnableRobot();"

                @contextlib.contextmanager
                def connection(*args, **kwargs):
                    yield dash, Mock()

                feedback = Mock()
                feedback.__enter__ = Mock(return_value=feedback)
                feedback.__exit__ = Mock(return_value=False)
                feedback.state.return_value = {"enable_status": 1}
                with contextlib.ExitStack() as stack:
                    check = stack.enter_context(patch.object(recipe, "check_trajectories", return_value=takes))
                    stack.enter_context(patch.object(recipe, "connect", connection))
                    stack.enter_context(patch.object(recipe, "LiveFeedback", return_value=feedback))
                    stack.enter_context(patch.object(recipe, "TrajectoryPlayer", return_value=player))
                    confirm = stack.enter_context(patch.object(recipe, "operator_confirm"))
                    stack.enter_context(patch.object(recipe, "wait_cup_drop_finished", side_effect=lambda: events.append("cup")))
                    stack.enter_context(patch.object(recipe, "wait_coffee_finished", side_effect=lambda: events.append("coffee")))
                    if hasattr(recipe, "wait_ice_water_finished"):
                        stack.enter_context(patch.object(recipe, "wait_ice_water_finished", side_effect=lambda: events.append("ice")))
                    stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                    recipe.run(self.config, auto=True, commissioning=commissioning)
                check.assert_called_once_with(self.config, require_verified=not commissioning)
                expected = ["MovL", "pick_cup", "cup"]
                expected += ["cup_to_ice", "ice", "ice_to_coffee"] if "IceAmericano" in script else ["cup_to_coffee"]
                expected += ["coffee", "coffee_to_window", "MovL"]
                self.assertEqual(events, expected)
                self.assertEqual(confirm.call_count, 1+len(takes)+2 if commissioning else 0)

    def test_missing_file_preflight_never_connects_or_enables(self):
        for script in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            spec = importlib.util.spec_from_file_location("test_recipe", robot_common.ROOT / script)
            recipe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(recipe)
            with patch.object(recipe, "connect") as connect, \
                    patch.object(recipe, "enable_robot") as enable, contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(ValueError):
                    recipe.run(self.config)
                connect.assert_not_called()
                enable.assert_not_called()

    def test_shared_segments_and_wait_order(self):
        recipes = []
        for script in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            spec = importlib.util.spec_from_file_location("test_recipe", robot_common.ROOT / script)
            recipe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(recipe)
            recipes.append(recipe)
        hot, iced = recipes
        self.assertEqual([n for a,n,_ in hot.PLAN if a == "trajectory"],
                         ["pick_cup", "cup_to_coffee", "coffee_to_window"])
        self.assertEqual([n for a,n,_ in iced.PLAN if a == "trajectory"],
                         ["pick_cup", "cup_to_ice", "ice_to_coffee", "coffee_to_window"])
        self.assertEqual([a for a,_,_ in iced.PLAN if a.endswith("_signal")],
                         ["cup_signal", "ice_signal", "coffee_signal"])


if __name__ == "__main__":
    unittest.main()
