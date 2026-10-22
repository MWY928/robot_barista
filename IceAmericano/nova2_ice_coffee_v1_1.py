#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DOBOT Nova 2 single-arm iced Americano workflow.

Use the shared nova2_points.json at the repository root and verify all taught routes.
  python nova2_ice_coffee_v1_1.py --dry-run
  python nova2_ice_coffee_v1_1.py --teach SAFE_CUP
  python nova2_ice_coffee_v1_1.py --run
"""

import argparse
import json
import math
import re
import socket
import sys
import time
from contextlib import ExitStack, closing, contextmanager
from pathlib import Path

POINT_FILE = Path(__file__).resolve().parent.parent / "nova2_points.json"
WORKFLOW = "ice_americano"

POINT_NAMES = (
    "HOME", "SAFE_CUP", "CUP_PRE", "CUP_RECEIVE",
    "SAFE_ICE", "ICE_PRE", "ICE_POS",
    "SAFE_COFFEE", "COFFEE_PRE", "COFFEE_POS",
    "SAFE_WINDOW", "WINDOW_PRE", "WINDOW_PLACE",
    "WINDOW_RELEASE", "WINDOW_EXIT", "WINDOW_CLEAR",
)


def read_config():
    with POINT_FILE.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_config(config):
    with POINT_FILE.open("w", encoding="utf-8") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
        f.write("\n")


def send_command(command, *args):
    """Send a command and check the V3 response status."""
    name = command.__name__
    reply = command(*args)
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*\{[^{}]*\}\s*,[^;]*;\s*", reply or "")
    if match is None:
        raise RuntimeError(f"{name} returned an invalid or incomplete response: {reply!r}")
    code = int(match.group(1))
    if code != 0:
        raise RuntimeError(f"{name} failed with error code {code}: {reply}")
    return reply


def read_vector(command):
    reply = send_command(command)
    values_text = re.search(r"\{([^}]*)\}", reply).group(1)
    values = [float(s) for s in values_text.split(",")]
    if not is_pose(values):
        raise RuntimeError(f"{command.__name__} must return six finite numbers: {reply}")
    return values


def is_pose(values):
    return isinstance(values, list) and len(values) == 6 and all(
        type(v) in (float, int) and math.isfinite(v) for v in values
    )


def validate_config(config):
    points = config.get("points", {})
    missing = []
    for name in POINT_NAMES:
        data = points.get(name) or {}
        if not is_pose(data.get("pose")):
            missing.append(f"{name}.pose")
        if name == "HOME" and not is_pose(data.get("joint")):
            missing.append(f"{name}.joint")
    if missing:
        raise ValueError("Teach or correct the following points:\n - " + "\n - ".join(missing))
    sf = config.get("speed_factor", 10)
    if type(sf) is not int or not (1 <= sf <= 100):
        raise ValueError("speed_factor must be an integer from 1 to 100.")
    tolerance = config.get("home_joint_tolerance_deg", 3)
    if type(tolerance) not in (float, int) or not (0 < tolerance <= 10):
        raise ValueError("home_joint_tolerance_deg must be in (0, 10]; 1-3 degrees is recommended.")
    for key in ("user_index", "tool_index"):
        idx = config.get(key, 0)
        if type(idx) is not int or not (0 <= idx <= 9):
            raise ValueError(f"{key} must be an integer from 0 to 9.")
    if config.get("safe_routes_physically_verified", {}).get(WORKFLOW) is not True:
        raise ValueError(
            "Verify every route at low speed in DobotStudio Pro, then set "
            f"safe_routes_physically_verified.{WORKFLOW} to true."
        )
    return points


@contextmanager
def connect(config, for_movement):
    # Import only when connecting; offline commands do not need the SDK.
    sdk_dir = Path(__file__).resolve().parent.parent / "TCP_IP_Python_V3"
    if str(sdk_dir) not in sys.path:
        sys.path.append(str(sdk_dir))
    import dobot_api as robot_api

    ip = config["robot_ip"]
    with ExitStack() as stack:
        previous_timeout = socket.getdefaulttimeout()
        try:
            socket.setdefaulttimeout(8)
            dashboard = stack.enter_context(closing(robot_api.DobotApiDashboard(ip, 29999)))
            move = stack.enter_context(closing(robot_api.DobotApiMove(ip, 30003))) if for_movement else None
        except Exception as exc:
            raise ConnectionError(f"Cannot connect to {ip}: {exc}") from exc
        finally:
            socket.setdefaulttimeout(previous_timeout)
        dashboard.socket_dobot.settimeout(90)
        if move is not None:
            move.socket_dobot.settimeout(90)
        send_command(dashboard.User, config.get("user_index", 0))
        send_command(dashboard.Tool, config.get("tool_index", 0))
        yield dashboard, move


def teach(config, name):
    with connect(config, for_movement=False) as (dashboard, _):
        angles = read_vector(dashboard.GetAngle)
        pose = read_vector(dashboard.GetPose)
        print(f"waypoint to record: {name}")
        print(f"  joint (deg): {angles}")
        print(f"  pose (mm/deg): {pose}")
        print("Confirm the Tool/User frames, TCP and payload are correct.")
        if input("Type SAVE to save; any other input cancels: ").strip() != "SAVE":
            print("Point was not saved.")
            return
        config.setdefault("points", {})[name] = {"joint": angles, "pose": pose}
        config["safe_routes_physically_verified"] = {
            name: False for name in config.get("safe_routes_physically_verified", {})
        }
        write_config(config)
        print(f"Saved to {POINT_FILE}")
        print("All workflow route verifications were reset. Verify the routes again before running.")


def operator_confirm(message):
    print("\n" + message)
    if input("Type Y to continue; any other input aborts: ").strip().upper() != "Y":
        raise RuntimeError("Operator aborted the task. No further commands will be sent.")


def wait_cup_drop_finished():
    """TODO: Replace manual confirmation with real feedback and a timeout."""
    operator_confirm("[Manual feedback] Confirm the cup is seated correctly in the fixture.")


def wait_ice_water_finished():
    """TODO: Replace manual confirmation with real feedback and a timeout."""
    operator_confirm(
        "[Manual feedback] Confirm ice and water dispensing have both finished, "
        "the outlet has stopped, and the cup is seated correctly."
    )


def wait_coffee_finished():
    """TODO: Replace manual confirmation with real feedback and a timeout."""
    operator_confirm("[Manual feedback] Confirm espresso dispensing has finished (espresso only, no extra hot water).")
    time.sleep(2.0)


# Machine layout and all full-arm paths require physical verification.
# Execute one segment at a time and wait for completion.
PLAN = [
    ("check_home", "HOME", "Verify the initial HOME joint angles without moving"),
    ("linear", "SAFE_CUP", "Move from HOME to the cup dispenser corridor waypoint"),
    ("linear", "CUP_PRE", "Approach the cup dispenser"),
    ("linear", "CUP_RECEIVE", "Move to the cup receiving position"),
    ("cup_signal", None, "Wait for cup dispensing to finish"),
    ("linear", "CUP_PRE", "Retract from the cup receiving position"),
    ("linear", "SAFE_CUP", "Return to the cup dispenser corridor waypoint"),
    ("linear", "SAFE_ICE", "Follow the corridor to the ice machine waypoint"),
    ("linear", "ICE_PRE", "Approach the ice and water outlet"),
    ("linear", "ICE_POS", "Move to the ice and water receiving position"),
    ("ice_signal", None, "Wait for both ice and water dispensing to finish"),
    ("linear", "ICE_PRE", "Retract from the ice and water receiving position"),
    ("linear", "SAFE_ICE", "Return to the ice machine corridor waypoint"),
    ("linear", "SAFE_COFFEE", "Follow the corridor to the coffee machine waypoint"),
    ("linear", "COFFEE_PRE", "Approach the coffee machine"),
    ("linear", "COFFEE_POS", "Move to the espresso dispensing position"),
    ("coffee_signal", None, "Wait for espresso dispensing to finish, then wait another 2 seconds"),
    ("linear", "COFFEE_PRE", "Retract from the coffee dispensing position"),
    ("linear", "SAFE_COFFEE", "Return to the coffee machine corridor waypoint"),
    ("linear", "SAFE_WINDOW", "Follow the corridor to the serving window waypoint"),
    ("linear", "WINDOW_PRE", "Approach the serving window"),
    ("linear", "WINDOW_PLACE", "Place the cup at the serving window"),
    ("linear", "WINDOW_RELEASE", "Lower the fixture to release support of the cup rim"),
    ("linear", "WINDOW_EXIT", "Withdraw the fixture horizontally through the opening"),
    ("linear", "WINDOW_CLEAR", "Move the fixture clear of the cup and window"),
    ("linear", "SAFE_WINDOW", "Return to the serving window corridor waypoint"),
    ("linear", "HOME", "Return to HOME along the verified route"),
]


def dry_run(config):
    print("Dry run: display the plan without connecting to the robot or peripherals.")
    print("A machine row near Y = 400 mm is not a verified collision boundary.")
    print("Routes physically verified:", config.get("safe_routes_physically_verified", {}).get(WORKFLOW, False))
    for i, (action, target, note) in enumerate(PLAN, 1):
        data = (config.get("points", {}).get(target) or {}) if target else {}
        present = is_pose(data.get("joint" if action == "check_home" else "pose"))
        state = (" [taught]" if present else " [not taught]") if target else ""
        print(f"{i:02d}. {note} | {action} {target or ''}{state}")


def verify_home(dashboard, points, tolerance_deg):
    current = read_vector(dashboard.GetAngle)
    taught = points["HOME"]["joint"]
    errors = [abs(a - b) for a, b in zip(current, taught)]
    if any(e > tolerance_deg for e in errors):
        raise RuntimeError(
            f"Robot is not at HOME: current joints={current}, taught HOME={taught}. "
            "Move to HOME manually in DobotStudio Pro before running."
        )
    print("Initial HOME joint angle check passed.")


def run(config, auto):
    points = validate_config(config)
    print("All linear segments and full-arm/fixture/cup paths must be verified at low speed.")

    with connect(config, for_movement=True) as (dashboard, move):
        # Verify the starting joints before enabling the robot.
        verify_home(dashboard, points, config.get("home_joint_tolerance_deg", 3))
        send_command(dashboard.SpeedFactor, config.get("speed_factor", 10))
        send_command(dashboard.EnableRobot)
        for idx, (action, name, note) in enumerate(PLAN, 1):
            print(f"\n[{idx}/{len(PLAN)}] {note}")
            if action == "check_home":
                continue
            if action == "cup_signal":
                wait_cup_drop_finished()
                continue
            if action == "ice_signal":
                wait_ice_water_finished()
                continue
            if action == "coffee_signal":
                wait_coffee_finished()
                continue
            if action != "linear":
                raise RuntimeError(f"Unknown action type: {action}")
            if not auto:
                operator_confirm(
                    f"Ready to send MovL to {name}. Confirm the cup, fixture "
                    "and all arm links have clearance along the entire path."
                )
            send_command(move.MovL, *points[name]["pose"])
            send_command(move.Sync)
            print("Completed:", name)
        print("\nMotion sequence completed.")
        # Closing connections does not guarantee that the robot has stopped.


def main():
    config = read_config()
    parser = argparse.ArgumentParser(description="Nova 2 iced Americano workflow")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--dry-run", action="store_true", help="display the plan without the SDK or robot")
    group.add_argument("--teach", choices=tuple(config["points"]), metavar="POINT", help="record current pose and joint angles")
    group.add_argument("--run", action="store_true", help="run with confirmation before each movement")
    parser.add_argument(
        "--auto", action="store_true",
        help="skip movement confirmations; cup, ice/water and espresso feedback still require manual confirmation",
    )
    args = parser.parse_args()
    if args.auto and not args.run:
        parser.error("--auto requires --run")
    if args.dry_run:
        dry_run(config)
    elif args.teach:
        teach(config, args.teach)
    else:
        run(config, args.auto)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"\nTask aborted; no further commands will be sent: {exc}", file=sys.stderr)
        print("Robot stop is not confirmed. Check its state and stop it if needed.", file=sys.stderr)
        sys.exit(1)
