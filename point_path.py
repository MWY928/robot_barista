#!/usr/bin/env python3
"""用途：手动录制多点路径，并逐点 MovL 回放，不使用 ServoJ 时间轨迹。
入口：record_point_path() 只读反馈录点；load_point_path() 离线校验；play_point_path() 会使能并运动。
根目录运行：
  python point_path.py record cup_route       # Enter 录一点；u+Enter 撤销；q+Enter 结束
    python point_path.py record cup_route --append  # 保留原点位，从末尾继续录制
  python point_path.py show cup_route         # 离线查看点位和直线段距离
    python point_path.py move cup_route --point 3 --speed 10  # 直接 MovL 到第3点，会运动
  python point_path.py play cup_route --speed 10  # 会运动；速度为全局百分比，逐点到位
    python point_path.py play cup_route --reverse  # 反向连续播放，无需确认，会运动
文件：point_paths/NAME.json，独立格式；不覆盖现有文件或修改原有轨迹。
"""

import argparse
import json
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from robot_common import (CONFIG_FILE, ROOT, command_timeout, connect, enable_robot, is_pose,
                          load_sdk, read_config, read_vector, send_command, send_queued_command,
                          validate_connection_config, write_config)
from trajectory_player import (LiveFeedback, TrajectoryPlayer, check_frames, describe_errors,
                               limits_for, state_errors, within)

POINT_PATH_DIR = ROOT / "point_paths"
POINT_PATH_SOURCE = "dobot_v4_manual_movel_points"


def point_path_file(name):
    if not isinstance(name, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name) is None:
        raise ValueError("Path name must contain letters, numbers, _ or -, starting with a letter or number.")
    return POINT_PATH_DIR / f"{name}.json"


def record_point_path(name, config=None, *, append=False):
    """Passively capture one stationary joint/pose pair per Enter key; save each edit."""
    config = read_config() if config is None else config
    validate_connection_config(config)
    settings = limits_for(config)
    path = point_path_file(name)
    data = {"schema_version": 1, "source": POINT_PATH_SOURCE, "name": name,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "robot_ip": config["robot_ip"], "units": {"joint": "degrees", "pose": "mm / degrees"},
            "feedback_frames": None, "complete": False, "points": []}
    if append:
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if (not isinstance(data, dict) or data.get("schema_version") != 1
                or data.get("source") != POINT_PATH_SOURCE):
            raise ValueError("Expected a manual MovL point path.")
        if data.get("robot_ip") != config["robot_ip"]:
            raise ValueError("Recorded robot IP differs from current config.")
        points = data.get("points")
        if not isinstance(points, list) or any(
                not isinstance(point, dict) or not is_pose(point.get("joint"))
                or not is_pose(point.get("pose")) for point in points):
            raise ValueError("Existing points contain invalid joint/pose data.")
        if points or data.get("feedback_frames") is not None:
            check_frames(data.get("feedback_frames"), config)
    original_count = len(data["points"])
    path.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the new name before connecting. Subsequent atomic writes belong to this take.
    if not append:
        with path.open("x", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
    else:
        print(f"Appending after {original_count} existing points; undo only removes new points.")
    print(f"Recording points to {path}; no enable, frame-selection or movement commands.")
    print("Prepare matching Tool/User and teaching/drag mode on the robot first.")
    print("Enter = save stationary point; u + Enter = undo; q + Enter = finish. Ctrl+C also finishes.")
    try:
        with LiveFeedback(config) as feedback:
            while True:
                key = input(f"Point {len(data['points'])+1:02d}> ").strip().lower()
                if key == "q":
                    break
                if key == "u":
                    if len(data["points"]) > original_count:
                        data["points"].pop()
                        write_config(data, path)
                        print(f"Removed last point; {len(data['points'])} retained.")
                    else:
                        print("No new points to undo; original points are preserved.")
                    continue
                if key:
                    print("Press Enter alone to record, u to undo, or q to finish.")
                    continue
                state = feedback.state()
                if data["feedback_frames"] is None:
                    data["feedback_frames"] = state["frames"]
                elif state["frames"] != data["feedback_frames"]:
                    raise RuntimeError("Tool/User definitions changed; take remains incomplete.")
                if max(map(abs, state["joint_velocity"])) > settings["settle_joint_speed_deg_s"]:
                    print("Arm is moving; stop and press Enter again. No point saved.")
                    continue
                data["points"].append({"joint": state["joint"], "pose": state["pose"]})
                write_config(data, path)
                print(f"Saved P{len(data['points']):02d}: pose={state['pose']}")
    except (KeyboardInterrupt, EOFError):
        print("\nFinishing point capture.")
    if len(data["points"]) < 2:
        raise ValueError(f"Need at least two points. Incomplete take retained at {path}; use a new name.")
    data["complete"] = True
    write_config(data, path)
    print(f"Saved {len(data['points'])} points. Recording did not move the robot.")
    return path


def load_point_path(name, config):
    """Validate the entire manual path before any robot connection; no time derivatives."""
    validate_connection_config(config)
    path = point_path_file(name)
    with path.open(encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict) or data.get("schema_version") != 1 or data.get("source") != POINT_PATH_SOURCE:
        raise ValueError("Expected a manual MovL point path, not a timed ServoJ recording.")
    # Completion is recording metadata, not a prerequisite for loading saved points.
    check_frames(data.get("feedback_frames"), config)
    points = data.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("A point path needs at least two points.")
    for index, point in enumerate(points, 1):
        if not isinstance(point, dict) or not is_pose(point.get("pose")) or not is_pose(point.get("joint")):
            raise ValueError(f"P{index:02d} requires six finite pose and joint values.")
    return data


def show_point_path(name, config):
    """Print points and straight-line segment lengths without connecting."""
    data = load_point_path(name, config)
    print(f"{name}: {len(data['points'])} points; controller-planned MovL, stop at every point.")
    previous = None
    for index, point in enumerate(data["points"], 1):
        distance = "" if previous is None else f"; segment={state_errors(previous, point)['position_mm']:.2f} mm"
        print(f"P{index:02d}: pose={point['pose']}; joint={point['joint']}{distance}")
        previous = point


def _confirm(message):
    print(message)
    if input("Type Y to continue; any other input aborts: ").strip().upper() != "Y":
        raise RuntimeError("Operator aborted the point-path run.")


def play_continuous(player, points, radius):
    """Reach the start exactly, queue blended intermediates, then verify the end."""
    player.move_linear(points[0])
    started = time.monotonic()
    command_id = None
    for index, point in enumerate(points[1:], 2):
        player.state()
        options = {"cp": 0} if index == len(points) else {"r": radius}
        print(f"[{index}/{len(points)}] Queued MovL {options}: {point['pose']}")
        player.motion_started = True
        with command_timeout(player.move, player.limits["command_timeout_s"]):
            command_id = send_queued_command(
                player.move.MovL, point["pose"], load_sdk().CoordinateType.CARTESIAN,
                user=player.config.get("user_index", 0),
                tool=player.config.get("tool_index", 0), **options)
    player.wait_for_target(points[-1], after=started, command_id=command_id,
                           timeout=player.limits["linear_timeout_s"] * (len(points) - 1))
    player.assert_target(points[-1])


def play_point_path(name, config=None, *, speed=None, reverse=False, step=False, point_index=None,
                    continuous=False, radius=5):
    """Enable and execute explicit MovL targets; reverse/step are debugging helpers."""
    config = read_config() if config is None else config
    data = load_point_path(name, config)
    if continuous and point_index is not None:
        raise ValueError("Continuous mode requires full-path playback.")
    if type(radius) is not int or radius <= 0:
        raise ValueError("Blend radius must be a positive integer in mm.")
    if continuous:
        print(f"CONTINUOUS: intermediate points may be bypassed; blend radius={radius} mm. "
              "Check clearance around rounded corners. Only endpoints are verified.")
    speed = config.get("speed_factor", 10) if speed is None else speed
    if type(speed) is not int or not 1 <= speed <= 100:
        raise ValueError("speed must be an integer percentage from 1 to 100.")
    if point_index is not None:
        if type(point_index) is not int or not 1 <= point_index <= len(data["points"]):
            raise ValueError(f"point must be between 1 and {len(data['points'])} (1-based).")
        if reverse or step:
            raise ValueError("Single-point movement does not support reverse/step.")
        points = [data["points"][point_index - 1]]
        print(f"Direct MovL target: {name} P{point_index:02d}; intermediate points will NOT be visited.")
    else:
        points = list(reversed(data["points"])) if reverse else data["points"]
    print(f"{name}: {len(points)} points, {'reverse' if reverse else 'forward'}, speed factor {speed}%.")
    print(f"Start pose: {points[0]['pose']}; start joints: {points[0]['joint']}")
    notify = _confirm if point_index is not None else print
    notify(
        "PHYSICAL MOTION: exit drag mode. "
        + ("Confirm the direct straight path from the CURRENT position to this target is clear; "
           if point_index is not None else
           "Position near the start; confirm every straight arm/cup path is clear; ")
        + "the robot will be enabled."
    )
    # Feedback already verifies active frame indices; each MovL carries explicit User/Tool.
    # Avoid queueing frame changes before EnableRobot, which disabled controllers may reject.
    with connect(config, for_movement=True, select_frames=False) as (dashboard, move):
        with LiveFeedback(config) as feedback:
            player = TrajectoryPlayer(dashboard, move, config, feedback)
            player.frames = data["feedback_frames"]
            initial = player.state(require_enabled=False)
            errors = state_errors(initial, points[0])
            if point_index is None and not within(errors, player.limits):
                raise RuntimeError("Not near the first point; position there manually: " + describe_errors(errors))
            if point_index is not None and (initial["running_status"] or
                    max(map(abs, initial["joint_velocity"])) > player.limits["settle_joint_speed_deg_s"]):
                raise RuntimeError("Robot must be stationary before direct point movement.")
            pose = read_vector(dashboard.GetPose)
            if not within(state_errors(initial, {"pose": pose, "joint": initial["joint"]}),
                          player.limits, arrival=True):
                raise RuntimeError("Dashboard and feedback poses differ; verify Tool/User/TCP conventions.")
            try:
                with command_timeout(dashboard, player.limits["command_timeout_s"]):
                    send_command(dashboard.SpeedFactor, speed)
                    enable_robot(dashboard)
                deadline = time.monotonic()+player.limits["endpoint_timeout_s"]
                while not player.state(require_enabled=False)["enable_status"]:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Enabled feedback did not arrive.")
                    time.sleep(0.02)
                if continuous:
                    play_continuous(player, points, radius)
                else:
                    for index, point in enumerate(points, 1):
                        print(f"[{index}/{len(points)}] MovL to pose={point['pose']}")
                        player.move_linear(point)
                return player.state()
            except (Exception, KeyboardInterrupt):
                player.stop_after_error()
                raise


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("record", "show", "play", "move"))
    parser.add_argument("name", help="point path name under point_paths/")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--speed", type=int, help="play/move: global speed percentage, defaults to speed_factor")
    parser.add_argument("--point", type=int, help="move only: target point number, starting at 1")
    parser.add_argument("--append", action="store_true", help="record: 保留已有点位并在末尾追加录制")
    parser.add_argument("--reverse", action="store_true", help="play only: reverse point order for debugging")
    parser.add_argument("--step", action="store_true", help="兼容旧命令；play 不再逐点确认，此参数忽略")
    parser.add_argument("--continuous", action="store_true", help="play: 首尾精确到位，中间点圆滑连续运动")
    parser.add_argument("--radius", type=int, help="连续模式过渡半径 mm，正整数，默认5")
    args = parser.parse_args()
    if args.append and args.action != "record":
        parser.error("--append requires record")
    if args.continuous and args.action != "play":
        parser.error("--continuous requires play")
    if args.radius is not None and (not args.continuous or args.radius <= 0):
        parser.error("--radius requires --continuous and a positive integer")
    if args.action not in ("play", "move") and args.speed is not None:
        parser.error("--speed requires play or move")
    if args.action != "play" and (args.reverse or args.step):
        parser.error("--reverse and --step require play")
    if args.action == "move" and (args.point is None or args.point < 1):
        parser.error("move requires --point N (N >= 1)")
    if args.action != "move" and args.point is not None:
        parser.error("--point requires move")
    config = read_config(args.config)
    if args.action == "record":
        record_point_path(args.name, config, append=args.append)
    elif args.action == "show":
        show_point_path(args.name, config)
    else:
        play_point_path(args.name, config, speed=args.speed, reverse=args.reverse, step=args.step,
                    point_index=args.point, continuous=args.continuous,
                    radius=args.radius if args.radius is not None else 5)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Point path aborted: {exc}", file=sys.stderr)
        sys.exit(1)
