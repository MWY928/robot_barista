#!/usr/bin/env python3
"""用途：示教保存静态点位，或移动到已保存点位。
主要入口：teach_point() 读取当前关节/位姿，输入 SAVE 后保存，不发送运动指令；
          move_point() 确认后使能并等待到位；默认 MovJ，linear=True 使用 MovL。
根目录运行：
  python teach_point.py HOME                 # 示教保存 HOME
  python teach_point.py --list               # 离线列出点位
  python teach_point.py --move HOME --speed 5 # 会运动：移动到已保存的 HOME
  python teach_point.py --move CUP_RECEIVE --linear --speed 5 # 会运动：直线测试精确点位
"""

import argparse
import re
import sys
import time
from pathlib import Path

from robot_common import (CONFIG_FILE, command_timeout, connect, enable_robot, is_pose, load_sdk,
                          read_config, read_vector, send_command, send_queued_command, write_config)
from trajectory_player import LiveFeedback, TrajectoryPlayer, state_errors, within


def teach_point(name, config_path=CONFIG_FILE):
    # raise error when point name is illegal
    if re.fullmatch(r"[A-Z][A-Z0-9_]*", name) is None:
        raise ValueError("Point names must start with A-Z and contain only A-Z, 0-9 or _.")
    # locate point config paths
    config_path = Path(config_path)
    config = read_config(config_path)

    #read the point from the robot 
    print("Keep the robot stationary while reading. Check Tool/User, TCP and payload.")
    with connect(config, for_movement=False) as (dashboard, _):
        joint = read_vector(dashboard.GetAngle)
        pose = read_vector(dashboard.GetPose)
    print(f"Waypoint: {name} ({'existing' if name in config.get('points', {}) else 'new'})")
    print(f"  joint (deg): {joint}")
    print(f"  pose (mm/deg): {pose}")
    #saving points
    if input("Type SAVE to save; any other input cancels: ").strip() != "SAVE":
        print("Point was not saved.")
        return False
    #   Preserve unrelated config edits made while the operator was confirming.
    latest = read_config(config_path)
    for key in ("robot_ip", "user_index", "tool_index"):
        if latest.get(key) != config.get(key):#check if other configs are match, if not match then cancle
            raise RuntimeError(f"{key} changed during teaching; point was not saved. Record again.")
    latest.setdefault("points", {}).setdefault(name, {}).update(joint=joint, pose=pose)
    latest["safe_routes_physically_verified"] = {
        workflow: False for workflow in latest.get("safe_routes_physically_verified", {})
    }
    write_config(latest, config_path)
    print(f"Saved to {config_path}")
    print("All workflow route verifications were reset. Verify routes again before running.")
    return True


def move_point(name, config_path=CONFIG_FILE, speed=None, *, linear=False):
    """Move to a saved point by MovJ or explicit MovL, then check arrival."""
    config = read_config(config_path)
    point = config.get("points", {}).get(name)
    if not isinstance(point, dict) or not is_pose(point.get("joint")) or not is_pose(point.get("pose")):
        raise ValueError(f"{name}: a saved point with six valid joint angles and pose values is required.")
    speed = config.get("speed_factor", 10) if speed is None else speed
    if type(speed) is not int or not 1 <= speed <= 100:
        raise ValueError("speed must be an integer from 1 to 100.")
    print(f"Move to saved point: {name}")
    print(f"  joint (deg): {point['joint']}")
    print(f"  pose (mm/deg): {point['pose']}")
    method = "MovL(pose=...)" if linear else "MovJ(joint=...)"
    print(f"{method} at speed factor {speed}%. Confirm the full arm/fixture path is clear.")
    if input("Type Y to enable and move; any other input cancels: ").strip().upper() != "Y":
        print("Movement cancelled.")
        return None
    with connect(config, for_movement=True) as (dashboard, move):
        with LiveFeedback(config) as feedback:
            player = TrajectoryPlayer(dashboard, move, config, feedback)
            initial = player.state(require_enabled=False)
            player.frames = initial["frames"]
            pose = read_vector(dashboard.GetPose)
            if not within(state_errors(initial, {"pose": pose, "joint": initial["joint"]}),
                          player.limits, arrival=True):
                raise RuntimeError("Dashboard and feedback poses differ; verify Tool/User/TCP conventions.")
            with command_timeout(move, player.limits["command_timeout_s"]):
                try:
                    send_command(dashboard.SpeedFactor, speed)
                    enable_robot(dashboard)
                    deadline = time.monotonic()+player.limits["endpoint_timeout_s"]
                    while not feedback.state()["enable_status"]:
                        if time.monotonic() >= deadline:
                            raise RuntimeError("Enabled feedback did not arrive.")
                        time.sleep(0.02)
                    player.state()
                    started = time.monotonic()
                    player.motion_started = True
                    if linear:
                        command_id = send_queued_command(
                            move.MovL, point["pose"], load_sdk().CoordinateType.CARTESIAN,
                            user=config.get("user_index", 0), tool=config.get("tool_index", 0), cp=0)
                    else:
                        command_id = send_queued_command(
                            move.MovJ, point["joint"], load_sdk().CoordinateType.JOINT, cp=0)
                    player.wait_for_target(point, after=started, timeout=player.limits["linear_timeout_s"],
                                           command_id=command_id)
                    player.assert_target(point)
                    print(f"Reached saved point {name}. Robot remains enabled; saved data was not changed.")
                    return player.state()
                except (Exception, KeyboardInterrupt):
                    player.stop_after_error()
                    raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("point", nargs="?", help="existing or new waypoint, e.g. HOME or LATTE_START")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--list", action="store_true", help="list shared waypoint names without connecting")
    action.add_argument("--move", metavar="POINT", help="enable and MovJ(joint=...) to a saved point, e.g. HOME")
    parser.add_argument("--speed", type=int, help="speed factor for --move (1-100; defaults to shared setting)")
    parser.add_argument("--linear", action="store_true", help="use MovL for --move, matching the recipe's point approach")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE, help="shared configuration path")
    args = parser.parse_args()
    if args.speed is not None and not args.move:
        parser.error("--speed requires --move")
    if args.linear and not args.move:
        parser.error("--linear requires --move")
    if args.point and (args.list or args.move):
        parser.error("use either a point name to teach, --list, or --move POINT")
    if args.list:
        if args.point:
            parser.error("use either a point name or --list")
        for name in read_config(args.config).get("points", {}):
            print(name)
    elif args.move:
        move_point(args.move, args.config, speed=args.speed, linear=args.linear)
    elif args.point:
        teach_point(args.point, args.config)
    else:
        parser.error("specify a point name, --list, or --move POINT")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Point operation aborted: {exc}", file=sys.stderr)
        sys.exit(1)
