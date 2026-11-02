"""仅 mock：不打开串口、不连接机械臂。"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import point_americano as flow


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.events = []
        self.enterContext(patch.object(flow.time, "monotonic", side_effect=lambda: self.now))
        self.enterContext(patch.object(flow.time, "sleep", side_effect=self.sleep))
        self.enterContext(patch.object(flow, "preflight", return_value=bytes(38)))
        self.play = self.enterContext(patch.object(flow, "play_point_path",
                                                  side_effect=lambda name, *a, **k: self.events.append(name)))
        self.cup_factory = self.enterContext(patch.object(flow, "SingleShotDispenser"))
        self.coffee_factory = self.enterContext(patch.object(flow, "BoundedCoffee"))
        self.cup = self.cup_factory.return_value.__enter__.return_value
        self.machine = self.coffee_factory.return_value.__enter__.return_value
        self.cup.dispense.side_effect = lambda **k: self.events.append("drop") or True
        self.machine.do_product.side_effect = lambda *a: self.events.append(("coffee", a))
        self.cup.query_status.side_effect = [self.cup_status(0), self.cup_status(1), self.cup_status(0)]
        self.statuses([2, 0x10, 1, 0xA2])

    def sleep(self, duration):
        self.events.append(("sleep", duration))
        self.now += duration

    @staticmethod
    def cup_status(state):
        return SimpleNamespace(ready=state == 0, work_state=state, motor_fault=False,
                               drawer_open=False, cup_missing=False)

    def statuses(self, values):
        def status(value):
            return flow.coffee.MachineStatus(0, value, 0, 0, 0, 0, 0, b"")
        self.machine.get_machine_status.side_effect = [status(value) for value in values]
        self.machine.get_machine_status.return_value = status(values[-1])

    def test_order_and_low_nibble_transition(self):
        flow.run({}, execute=True, speed=7, continuous=True, radius=3)
        self.assertEqual(self.events, ["pick_cup", "drop", ("sleep", .3), "cup2coffee",
                                      ("coffee", (5,)), ("sleep", 1), ("sleep", 1),
                                      ("sleep", 1), ("sleep", 2), "coffee2window"])
        self.assertEqual(self.cup.query_status.call_count, 3)
        self.assertEqual(self.machine.get_machine_status.call_count, 4)
        self.play.assert_called_with("coffee2window", {}, speed=7, continuous=True, radius=3)

    def test_preview_never_connects(self):
        flow.run({})
        self.cup_factory.assert_not_called()
        self.coffee_factory.assert_not_called()
        self.play.assert_not_called()

    def test_cup_ack_does_not_allow_motion(self):
        self.cup.query_status.side_effect = None
        self.cup.query_status.return_value = self.cup_status(1)
        self.cup.query_status.return_value.ready = True
        with self.assertRaises(TimeoutError):
            flow.run({}, execute=True, cup_timeout=.6)
        self.assertEqual(self.play.call_count, 1)
        self.cup.dispense.assert_called_once_with(drink_id=1)
        self.machine.do_product.assert_not_called()

    def test_ready_without_busy_times_out(self):
        self.machine.get_machine_status.side_effect = None
        with self.assertRaises(TimeoutError):
            flow.run({}, execute=True, coffee_timeout=2)
        self.assertEqual(self.play.call_count, 2)
        self.machine.do_product.assert_called_once_with(5)

    def test_busy_without_completion_times_out(self):
        self.statuses([0])
        self.machine.get_machine_status.side_effect = None
        with self.assertRaises(TimeoutError):
            flow.run({}, execute=True, coffee_timeout=2)
        self.assertEqual(self.play.call_count, 2)

    def test_protocol_error_aborts_without_retry(self):
        self.cup.dispense.side_effect = flow.cups.ProtocolError("lost reply")
        with self.assertRaises(flow.cups.ProtocolError):
            flow.run({}, execute=True)
        self.assertEqual(self.play.call_count, 1)
        self.assertEqual(self.cup.dispense.call_count, 1)


class AdapterTests(unittest.TestCase):
    def test_actual_dispenser_adapter_writes_once_on_timeout(self):
        dev = flow.SingleShotDispenser.__new__(flow.SingleShotDispenser)
        dev.ser = Mock()
        dev.ser.read.return_value = b""
        dev.deadline = flow.time.monotonic() + 10
        with self.assertRaises(flow.cups.ProtocolError):
            dev.dispense()
        dev.ser.write.assert_called_once_with(bytes.fromhex("04 07 AA 01 00 B6 FF"))

    def test_invalid_path_or_recipe_prevents_connections(self):
        for target in ("load_point_path", "coffee.product_data"):
            with self.subTest(target=target), patch("point_americano." + target,
                    side_effect=ValueError("invalid")), patch.object(flow, "SingleShotDispenser") as cups:
                with self.assertRaises(ValueError):
                    flow.run({}, execute=True)
                cups.assert_not_called()


if __name__ == "__main__":
    unittest.main()