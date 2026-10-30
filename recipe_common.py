"""用途：两份咖啡配方共用的离线校验、命令行参数和人工确认。
主要入口：inspect_workflow() 离线校验轨迹、精确点位和 MovL 衔接；recipe_arguments() 解析参数；
          operator_confirm() 等待人工确认，本身不控制机械臂。
本文件供配方导入；从根目录运行配方，例如：
  python Americano/nova2_coffee_v1_1.py --check-trajectories # 离线检查，不运动
"""

import argparse

from robot_common import is_pose, validate_connection_config
from trajectory_player import describe_errors, limits_for, load_trajectory, state_errors, within


def inspect_workflow(config, plan, workflow, require_verified=False, strict=True, report=True):
    """Check the actual mixed trajectory/point plan before any connection."""
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
    previous, previous_name, frames = None, None, None
    for index, (action, name, note) in enumerate(plan, 1):
        if report:
            print(f"{index:02d}. {note} | {action} {name or ''}")
        if action not in ("check_home", "entry", "trajectory", "point", "return_home"):
            continue
        try:
            if action in ("check_home", "point", "return_home"):
                point = config.get("points", {}).get(name)
                if not isinstance(point, dict) or not is_pose(point.get("pose")) or not is_pose(point.get("joint")):
                    raise ValueError(f"Point {name} requires taught pose and joint data; use teach_point.py {name}.")
                target, target_name = point, name
            else:
                if name not in trajectories:
                    trajectories[name] = load_trajectory(name, config, require_verified=require_verified)
                trajectory = trajectories[name]
                if frames is not None and frames != trajectory.frames:
                    raise ValueError(f"{name}: Tool/User frame definitions differ from the workflow.")
                frames = trajectory.frames
                target, target_name = trajectory.first, f"{name}.start"
                if report:
                    print(f"    {trajectory.path.name}: {len(trajectory.samples)} samples, "
                          f"{trajectory.duration:.2f}s recorded, "
                          f"verified={trajectory.data.get('replay_verified') is True}")
            if previous is not None:
                junction = state_errors(previous, target)
                # Explicit entry/point/return actions are planned MovL routes.
                # A trajectory without an entry still requires a nearby start.
                if action == "trajectory" and not within(junction, limits):
                    raise ValueError(f"{previous_name} -> {target_name}: junction exceeds start margins: "
                                     + describe_errors(junction) + "; add/commission an explicit entry MovL.")
                if report:
                    label = "Start junction" if action == "trajectory" else "Planned MovL"
                    print(f"    {label}: {previous_name} -> {target_name}: " + describe_errors(junction))
            if action == "trajectory":
                previous, previous_name = trajectory.last, f"{name}.end"
            else:
                previous, previous_name = target, target_name
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
