"""Shared configuration and Dobot V3 connection helpers."""

import argparse
import json
import math
import os
import re
import socket
import sys
import tempfile
from contextlib import ExitStack, closing, contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONFIG_FILE = ROOT / "robot_config.json"


def read_config(path=CONFIG_FILE):
    with Path(path).open(encoding="utf-8") as f:
        return json.load(f)


def write_config(config, path=CONFIG_FILE):
    """Replace a config atomically so a failed write preserves the old file."""
    path = Path(path)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         delete=False, suffix=".tmp") as f:
            temp_path = Path(f.name)
            json.dump(config, f, indent=2, ensure_ascii=False, allow_nan=False)
            f.write("\n")
        os.replace(temp_path, path)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def is_pose(values):
    return isinstance(values, list) and len(values) == 6 and all(
        type(v) in (float, int) and math.isfinite(v) for v in values
    )


def send_command(command, *args):
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
    values = [float(s) for s in re.search(r"\{([^}]*)\}", reply).group(1).split(",")]
    if not is_pose(values):
        raise RuntimeError(f"{command.__name__} must return six finite numbers: {reply}")
    return values


def enable_robot(dashboard):
    """Request robot enable and reject unsuccessful or incomplete responses."""
    return send_command(dashboard.EnableRobot)


def load_sdk():
    sdk_dir = str(ROOT / "TCP_IP_Python_V3")
    if sdk_dir not in sys.path:
        sys.path.insert(0, sdk_dir)
    import dobot_api
    return dobot_api


def validate_connection_config(config):
    if not isinstance(config.get("robot_ip"), str) or not config["robot_ip"].strip():
        raise ValueError("robot_ip must be a non-empty address.")
    for key in ("user_index", "tool_index"):
        index = config.get(key, 0)
        if type(index) is not int or not 0 <= index <= 9:
            raise ValueError(f"{key} must be an integer from 0 to 9.")


@contextmanager
def connect(config, for_movement=False):
    validate_connection_config(config)
    sdk = load_sdk()
    ip = config["robot_ip"]
    with ExitStack() as stack:
        previous_timeout = socket.getdefaulttimeout()
        try:
            socket.setdefaulttimeout(8)
            dashboard = stack.enter_context(closing(sdk.DobotApiDashboard(ip, 29999)))
            move = stack.enter_context(closing(sdk.DobotApiMove(ip, 30003))) if for_movement else None
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


def main():
    parser = argparse.ArgumentParser(description="Shared robot utilities")
    parser.add_argument("--enable", action="store_true", required=True,
                        help="send EnableRobot using the dashboard connection")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE,
                        help="shared configuration path")
    args = parser.parse_args()
    config = read_config(args.config)
    with connect(config, for_movement=False) as (dashboard, _):
        enable_robot(dashboard)
    print("Robot enable command accepted.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Enable failed: {exc}", file=sys.stderr)
        sys.exit(1)
