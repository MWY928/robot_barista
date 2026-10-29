#!/usr/bin/env python3
"""Record a stationary waypoint in the shared configuration."""

import argparse
import re
import sys
from pathlib import Path

from robot_common import POINT_FILE, connect, read_config, read_vector, write_config


def teach_point(name, config_path=POINT_FILE):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("point", nargs="?", help="existing or new waypoint, e.g. HOME or LATTE_START")
    parser.add_argument("--list", action="store_true", help="list shared waypoint names without connecting")
    parser.add_argument("--config", type=Path, default=POINT_FILE, help="shared configuration path")
    args = parser.parse_args()
    if args.list:
        if args.point:
            parser.error("use either a point name or --list")
        for name in read_config(args.config).get("points", {}):
            print(name)
    elif args.point:
        teach_point(args.point, args.config)
    else:
        parser.error("specify a point name or --list")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Teaching aborted: {exc}", file=sys.stderr)
        sys.exit(1)
