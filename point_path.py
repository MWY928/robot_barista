#!/usr/bin/env python3
"""用途：手动录制多点路径，并逐点 MovL 回放，不使用 ServoJ 时间轨迹。
入口：record_point_path() 只读反馈录点；load_point_path() 离线校验；play_point_path() 会使能并运动。
根目录运行：
  python point_path.py record cup_route       # Enter 录一点；u+Enter 撤销；q+Enter 结束
  python point_path.py show cup_route         # 离线查看点位和直线段距离
  python point_path.py play cup_route --speed 10  # 会运动；速度为全局百分比，逐点到位
  python point_path.py play cup_route --reverse --step  # 反向逐点调试，会运动
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
                          read_config, read_vector, send_command, validate_connection_config, write_config)
from trajectory_player import (LiveFeedback, TrajectoryPlayer, check_frames, describe_errors,
                               limits_for, state_errors, within)

POINT_PATH_DIR = ROOT / "point_paths"
POINT_PATH_SOURCE = "dobot_v4_manual_movel_points"


def point_path_file(name):
    if not isinstance(name, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name) is None:
        raise ValueError("Path name must contain letters, numbers, _ or -, starting with a letter or number.")
    return POINT_PATH_DIR / f"{name}.json"


def record_point_path(name, config=None):
    """Passively capture one stationary joint/pose pair per Enter key; save each edit."""
    config = read_config() if config is None else config
    validate_connection_config(config)
    settings = limits_for(config)
    path = point_path_file(name)
    data = {"schema_version": 1, "source": POINT_PATH_SOURCE, "name": name,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "robot_ip": config["robot_ip"], "units": {"joint": "degrees", "pose": "mm / degrees"},
            "feedback_frames": None, "complete": False, "points": []}
    path.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the new name before connecting. Subsequent atomic writes belong to this take.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
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
                    if data["points"]:
                        data["points"].pop()
                        write_config(data, path)
                        print(f"Removed last point; {len(data['points'])} retained.")
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
    if data.get("complete") is not True:
        raise ValueError("Point capture is incomplete; this file cannot be replayed.")
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


def play_point_path(name, config=None, *, speed=None, reverse=False, step=False):
    """Enable and execute explicit MovL targets; reverse/step are debugging helpers."""
    config = read_config() if config is None else config
    data = load_point_path(name, config)
    speed = config.get("speed_factor", 10) if speed is None else speed
    if type(speed) is not int or not 1 <= speed <= 100:
        raise ValueError("speed must be an integer percentage from 1 to 100.")
    points = list(reversed(data["points"])) if reverse else data["points"]
    print(f"{name}: {len(points)} points, {'reverse' if reverse else 'forward'}, speed factor {speed}%.")
    print(f"Start pose: {points[0]['pose']}; start joints: {points[0]['joint']}")
    _confirm("PHYSICAL MOTION: position near this start and exit drag mode. "
             "Confirm every straight arm/cup path is clear; the robot will be enabled.")
    # Feedback already verifies active frame indices; each MovL carries explicit User/Tool.
    # Avoid queueing frame changes before EnableRobot, which disabled controllers may reject.
    with connect(config, for_movement=True, select_frames=False) as (dashboard, move):
        with LiveFeedback(config) as feedback:
            player = TrajectoryPlayer(dashboard, move, config, feedback)
            player.frames = data["feedback_frames"]
            initial = player.state(require_enabled=False)
            errors = state_errors(initial, points[0])
            if not within(errors, player.limits):
                raise RuntimeError("Not near the first point; position there manually: " + describe_errors(errors))
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
                for index, point in enumerate(points, 1):
                    print(f"[{index}/{len(points)}] MovL to pose={point['pose']}")
                    if step:
                        _confirm(f"Move to point {index:02d}?")
                    player.move_linear(point)
                return player.state()
            except (Exception, KeyboardInterrupt):
                player.stop_after_error()
                raise


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("record", "show", "play"))
    parser.add_argument("name", help="point path name under point_paths/")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--speed", type=int, help="play only: global speed percentage, defaults to speed_factor")
    parser.add_argument("--reverse", action="store_true", help="play only: reverse point order for debugging")
    parser.add_argument("--step", action="store_true", help="play only: confirm every MovL")
    args = parser.parse_args()
    if args.action != "play" and (args.speed is not None or args.reverse or args.step):
        parser.error("--speed, --reverse and --step require play")
    config = read_config(args.config)
    if args.action == "record":
        record_point_path(args.name, config)
    elif args.action == "show":
        show_point_path(args.name, config)
    else:
        play_point_path(args.name, config, speed=args.speed, reverse=args.reverse, step=args.step)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Point path aborted: {exc}", file=sys.stderr)
        sys.exit(1)
