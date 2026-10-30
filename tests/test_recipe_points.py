"""Mixed point/trajectory recipes, with no robot or device connections."""

import contextlib
import copy
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from robot_common import ROOT
from trajectory_player import RECORDING_SOURCE, TrajectoryPlayer


class PointRecipeTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.frames = {"user_index": 0, "tool_index": 0, "user_frame": [0]*6, "tool_frame": [0]*6}
        self.config = {
            "robot_ip": "192.0.2.1", "user_index": 0, "tool_index": 0,
            "points": {name: self.point(x) for name, x in
                       (("HOME", 0), ("CUP_RECEIVE", 20), ("ICE_POS", 30), ("COFFEE_POS", 40))},
            "safe_routes_physically_verified": {"americano": True, "ice_americano": True},
            "trajectories": {},
        }
        for name, x in (("pick_cup", 0), ("cup_to_coffee", 25), ("cup_to_ice", 23),
                        ("ice_to_coffee", 35), ("coffee_to_window", 45)):
            samples = [dict(self.point(x+i*.004), t=i*.04, error_status=0) for i in range(3)]
            data = {"schema_version": 1, "source": RECORDING_SOURCE, "samples": samples,
                    "sample_count": 3, "feedback_frames": self.frames,
                    "stop_reason": "duration", "error": None, "replay_verified": True}
            path = self.directory / f"{name}.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            self.config["trajectories"][name] = {"file": str(path)}

    @staticmethod
    def point(x):
        return {"joint": [x, 0, 0, 0, 0, 0], "pose": [x, 0, 0, 0, 0, 0]}

    def recipe(self, ice=False):
        file = "IceAmericano/nova2_ice_coffee_v1_1.py" if ice else "Americano/nova2_coffee_v1_1.py"
        spec = importlib.util.spec_from_file_location("point_recipe_test", ROOT / file)
        recipe = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(recipe)
        return recipe

    @contextlib.contextmanager
    def session(self, recipe, fail_point=None):
        self.events = []
        self.current = copy.deepcopy(self.config["points"]["HOME"])
        takes = recipe.check_trajectories(self.config, require_verified=True)
        player = Mock(spec=TrajectoryPlayer)
        player.limits = {"endpoint_timeout_s": 1}

        def assert_target(target):
            if self.current != target:
                raise RuntimeError("Unexpected movement before planned connection")

        player.check_home.side_effect = assert_target
        player.assert_target.side_effect = assert_target

        def linear(target):
            for name, point in self.config["points"].items():
                if target is point:
                    self.events.append("point:"+name)
                    if name == fail_point:
                        raise RuntimeError("Point arrival failed")
                    break
            else:
                name = next(name for name, take in takes.items() if take.first == target)
                self.events.append("entry:"+name)
            self.current = copy.deepcopy(target)

        def play(take):
            assert_target(take.first)
            self.events.append("trajectory:"+take.name)
            self.current = copy.deepcopy(take.last)

        player.move_linear.side_effect = linear
        player.play.side_effect = play
        self.player = player
        dashboard, feedback = Mock(), Mock()
        for name in ("SpeedFactor", "EnableRobot"):
            command = getattr(dashboard, name)
            command.__name__ = name
            command.return_value = f"0,{{}},{name}();"
        feedback.state.return_value = {"enable_status": 1}

        @contextlib.contextmanager
        def connection(*args, **kwargs):
            yield dashboard, Mock()

        @contextlib.contextmanager
        def live(*args):
            yield feedback

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(recipe, "connect", connection))
            stack.enter_context(patch.object(recipe, "LiveFeedback", live))
            stack.enter_context(patch.object(recipe, "TrajectoryPlayer", return_value=player))
            for name, event in (("wait_cup_drop_finished", "cup"),
                                ("wait_coffee_finished", "coffee"),
                                ("wait_ice_water_finished", "ice")):
                if hasattr(recipe, name):
                    stack.enter_context(patch.object(recipe, name, side_effect=lambda e=event: self.events.append(e)))
            yield

    def test_points_arrive_before_devices_and_departure_entries(self):
        for ice in (False, True):
            with self.subTest(ice=ice), contextlib.redirect_stdout(io.StringIO()):
                recipe = self.recipe(ice)
                with self.session(recipe):
                    recipe.run(self.config, auto=True)
                expected = ["entry:pick_cup", "trajectory:pick_cup", "point:CUP_RECEIVE", "cup"]
                if ice:
                    expected += ["entry:cup_to_ice", "trajectory:cup_to_ice", "point:ICE_POS", "ice",
                                 "entry:ice_to_coffee", "trajectory:ice_to_coffee"]
                else:
                    expected += ["entry:cup_to_coffee", "trajectory:cup_to_coffee"]
                expected += ["point:COFFEE_POS", "coffee", "entry:coffee_to_window",
                             "trajectory:coffee_to_window", "point:HOME"]
                self.assertEqual(self.events, expected)

    def test_failed_precise_arrival_prevents_device_and_next_segment(self):
        recipe = self.recipe()
        with contextlib.redirect_stdout(io.StringIO()), self.session(recipe, fail_point="CUP_RECEIVE"):
            with self.assertRaisesRegex(RuntimeError, "Point arrival"):
                recipe.run(self.config, auto=True)
        self.assertNotIn("cup", self.events)
        self.assertNotIn("entry:cup_to_coffee", self.events)
        self.player.stop_after_error.assert_called_once()

    def test_missing_point_blocks_connection_in_both_recipes(self):
        self.config["points"]["COFFEE_POS"] = {"pose": None, "joint": None}
        for ice in (False, True):
            recipe = self.recipe(ice)
            with self.subTest(ice=ice), patch.object(recipe, "connect") as connect, \
                    contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, "COFFEE_POS"):
                    recipe.run(self.config, auto=True)
                connect.assert_not_called()

    def test_frame_mismatch_rejected_even_with_point_between_trajectories(self):
        path = Path(self.config["trajectories"]["cup_to_coffee"]["file"])
        data = json.loads(path.read_text(encoding="utf-8"))
        data["feedback_frames"]["tool_frame"][0] = 1
        path.write_text(json.dumps(data), encoding="utf-8")
        recipe = self.recipe()
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "Tool/User"):
            recipe.check_trajectories(self.config)


if __name__ == "__main__":
    unittest.main()
