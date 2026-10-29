"""Check enable failures and recipe startup ordering without robot hardware."""

import contextlib
import importlib.util
import io
import unittest
from unittest.mock import Mock, patch

import robot_common


class Dashboard:
    def __init__(self, events, reply="0,{},EnableRobot();"):
        self.events = events
        self.reply = reply

    def EnableRobot(self):
        self.events.append("enable")
        return self.reply

    def SpeedFactor(self, factor):
        self.events.append("speed")
        return "0,{},SpeedFactor();"


class Move:
    def __init__(self, events):
        self.events = events

    def MovL(self, *pose):
        self.events.append("move")
        return "0,{},MovL();"

    def Sync(self):
        self.events.append("sync")
        return "0,{},Sync();"


class EnableTests(unittest.TestCase):
    def run_recipe(self, file, reply="0,{},EnableRobot();", home_ok=True):
        spec = importlib.util.spec_from_file_location("enable_recipe", robot_common.ROOT / file)
        recipe = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(recipe)
        self.assertIs(recipe.enable_robot, robot_common.enable_robot)
        events = []
        dashboard = Dashboard(events, reply)

        @contextlib.contextmanager
        def connection(config, for_movement):
            self.assertTrue(for_movement)
            yield dashboard, Move(events)

        def home_check(*args):
            events.append("home")
            if not home_ok:
                raise RuntimeError("Robot is not at HOME")

        player = Mock()
        player.check_home.side_effect = home_check
        player.limits = {"endpoint_timeout_s": 5}
        player.play.side_effect = lambda trajectory: events.append("play")
        feedback = Mock()
        feedback.__enter__ = Mock(return_value=feedback)
        feedback.__exit__ = Mock(return_value=False)
        feedback.state.return_value = {"enable_status": 1}

        with patch.object(recipe, "connect", connection), \
                patch.object(recipe, "check_trajectories", return_value={"pick_cup": Mock()}), \
                patch.object(recipe, "LiveFeedback", return_value=feedback), \
                patch.object(recipe, "TrajectoryPlayer", return_value=player), \
                patch.object(recipe, "PLAN", [("trajectory", "pick_cup", "test")]), \
                contextlib.redirect_stdout(io.StringIO()):
            config = {"points": {"HOME": {"pose": [0]*6, "joint": [0]*6}}}
            if not home_ok or not reply.startswith("0,"):
                with self.assertRaises(RuntimeError):
                    recipe.run(config, auto=True)
            else:
                recipe.run(config, auto=True)
        return events

    def test_recipes_check_home_then_enable_before_motion(self):
        for file in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            with self.subTest(file=file):
                self.assertEqual(self.run_recipe(file), ["home", "speed", "enable", "play"])

    def test_enable_error_or_incomplete_reply_prevents_motion(self):
        for file in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            for reply in ("-1,{},EnableRobot();", "incomplete"):
                with self.subTest(file=file, reply=reply):
                    self.assertEqual(self.run_recipe(file, reply), ["home", "speed", "enable"])

    def test_wrong_home_prevents_enable_and_motion(self):
        for file in ("Americano/nova2_coffee_v1_1.py", "IceAmericano/nova2_ice_coffee_v1_1.py"):
            with self.subTest(file=file):
                self.assertEqual(self.run_recipe(file, home_ok=False), ["home"])

    def test_standalone_enable_uses_dashboard_connection(self):
        events = []

        @contextlib.contextmanager
        def connection(config, for_movement):
            self.assertFalse(for_movement)
            yield Dashboard(events), None

        with patch("sys.argv", ["robot_common.py", "--enable"]), \
                patch.object(robot_common, "read_config", return_value={}), \
                patch.object(robot_common, "connect", connection), \
                contextlib.redirect_stdout(io.StringIO()):
            robot_common.main()
        self.assertEqual(events, ["enable"])


if __name__ == "__main__":
    unittest.main()
