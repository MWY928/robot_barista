"""Offline inspection and command-line support for segmented recipes."""

import argparse

from robot_common import is_pose, validate_connection_config
from trajectory_player import describe_errors, limits_for, load_trajectory, state_errors, within


def inspect_workflow(config, plan, workflow, require_verified=False, strict=True, report=True):
    """Visit all segments in execution order, including their junctions."""
    errors, trajectories = [], {}
    limits = limits_for(config)
    validate_connection_config(config)
    home = config.get("points", {}).get("HOME", {})
    if not isinstance(home, dict) or not is_pose(home.get("pose")) or not is_pose(home.get("joint")):
        errors.append("HOME requires both pose and joint data.")
    speed = config.get("speed_factor", 10)
    if type(speed) is not int or not 1 <= speed <= 100:
        errors.append("speed_factor must be an integer from 1 to 100.")
    tolerance = config.get("home_joint_tolerance_deg", 3)
    if type(tolerance) not in (int, float) or not 0 < tolerance <= 10:
        errors.append("home_joint_tolerance_deg must be in (0, 10].")
    if require_verified and config.get("safe_routes_physically_verified", {}).get(workflow) is not True:
        errors.append(f"safe_routes_physically_verified.{workflow} is not true.")
    previous = None
    for index, (action, name, note) in enumerate(plan, 1):
        if report:
            print(f"{index:02d}. {note} | {action} {name or ''}")
        if action != "trajectory":
            continue
        try:
            trajectory = load_trajectory(name, config, require_verified=require_verified)
            trajectories[name] = trajectory
            if report:
                print(f"    {trajectory.path.name}: {len(trajectory.samples)} samples, "
                      f"{trajectory.duration:.2f}s recorded, "
                      f"verified={trajectory.data.get('replay_verified') is True}")
            if previous is not None:
                junction = state_errors(previous.last, trajectory.first)
                if previous.frames != trajectory.frames:
                    raise ValueError(f"{previous.name} -> {name}: Tool/User frame definitions differ.")
                if not within(junction, limits):
                    raise ValueError(f"{previous.name} -> {name}: junction exceeds start margins: "
                                     + describe_errors(junction))
                if report:
                    print("    Junction: " + describe_errors(junction))
            previous = trajectory
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(f"{name}: {exc}")
            previous = None  # Do not compare non-adjacent segments across a missing recording.
            if report:
                print(f"    ERROR: {exc}")
    if errors:
        if strict:
            raise ValueError("Workflow trajectory check failed:\n - " + "\n - ".join(errors))
        if report:
            print(f"Preflight incomplete: {len(errors)} issue(s). No robot connection was made.")
    elif report:
        print("All trajectory files and junctions passed offline checks. No robot connection was made.")
    return trajectories


def recipe_arguments(description):
    parser = argparse.ArgumentParser(description=description)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--dry-run", action="store_true", help="preview the segmented workflow offline")
    group.add_argument("--check-trajectories", action="store_true",
                       help="strictly inspect all recordings and segment junctions without connecting")
    group.add_argument("--check-steps", action="store_true",
                       help="MOVE the robot to commission each segment with confirmation; allows unverified takes")
    group.add_argument("--teach", metavar="POINT", help="alias for teach_point.py POINT")
    group.add_argument("--run", action="store_true", help="execute the verified segmented recipe")
    parser.add_argument("--auto", action="store_true", help="skip motion confirmations in --run only")
    args = parser.parse_args()
    if args.auto and not args.run:
        parser.error("--auto requires --run")
    return args


def operator_confirm(message):
    print("\n" + message)
    if input("Type Y to continue; any other input aborts: ").strip().upper() != "Y":
        raise RuntimeError("Operator aborted the task.")
