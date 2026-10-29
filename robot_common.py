"""Shared configuration and Dobot V4 connection helpers (controller 4.6.x)."""

import argparse
import json
import math
import os
import re
import sys
import tempfile
from contextlib import ExitStack, contextmanager
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

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


def send_command(command, *args, **kwargs):
    name = command.__name__
    try:
        reply = command(*args, **kwargs)
    except Exception as exc:
        # V4 SDK connection exceptions do not inherit from OSError.
        raise RuntimeError(f"{name} command failed: {exc}") from exc
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*\{[^{}]*\}\s*,[^;]*;\s*", reply or "")
    if match is None:
        raise RuntimeError(f"{name} returned an invalid or incomplete response: {reply!r}")
    code = int(match.group(1))
    if code != 0:
        raise RuntimeError(f"{name} failed with error code {code}: {reply}")
    return reply


def send_queued_command(command, *args, **kwargs):
    """Return the V4 queue ID; an acknowledgement alone is not arrival."""
    reply = send_command(command, *args, **kwargs)
    result = re.search(r"\{\s*(\d+)\s*\}", reply)
    if result is None:
        raise RuntimeError(f"{command.__name__} did not return a V4 queue ID: {reply}")
    return int(result.group(1))


@contextmanager
def command_timeout(api, seconds):
    """Temporarily change the shared V4 command connection's receive timeout."""
    connection = api.connection
    # The SDK exposes set_timeout(), but currently has no public timeout getter.
    previous = connection._receive_timeout
    command_socket = connection._socket
    previous_socket_timeout = command_socket.gettimeout()
    connection.set_timeout(receive_timeout=seconds)
    # In this SDK set_timeout() only stores defaults, even after connecting.
    command_socket.settimeout(seconds)
    try:
        yield
    finally:
        connection.set_timeout(receive_timeout=previous)
        command_socket.settimeout(previous_socket_timeout)


def read_vector(command):
    reply = send_command(command)
    values = [float(s) for s in re.search(r"\{([^}]*)\}", reply).group(1).split(",")]
    if not is_pose(values):
        raise RuntimeError(f"{command.__name__} must return six finite numbers: {reply}")
    return values


def enable_robot(dashboard):
    """Request robot enable and reject unsuccessful or incomplete responses."""
    return send_command(dashboard.EnableRobot)


@lru_cache(maxsize=1)
def load_sdk():
    """Load the V4 feature/v4-optimization SDK, locally unpacked or installed."""
    candidates = [ROOT / "TCP_IP_Python_V4", ROOT / "TCP-IP-Python-V4"]
    candidates.extend(sorted(path for path in ROOT.glob("TCP-IP-Python-V4-*") if path.is_dir()))
    for candidate in candidates:
        roots = [candidate, *sorted(candidate.glob("*/"))]
        sdk_root = next((path for path in roots if (path / "dobot_sdk" / "__init__.py").is_file()), None)
        if sdk_root is not None:
            if str(sdk_root) not in sys.path:
                sys.path.insert(0, str(sdk_root))
            break
    try:
        from dobot_sdk.api.motion import CoordinateType, Motion
        from dobot_sdk.api.robot_control import RobotControl
        from dobot_sdk.protocol.data_types import MyType
        from v4_connection import CommandConnection
    except ModuleNotFoundError as exc:
        if exc.name == "dobot_sdk":
            raise ImportError(
                "V4 SDK not found. Unpack Dobot-Arm/TCP-IP-Python-V4 "
                "(feature/v4-optimization, containing dobot_sdk/) in the project root, "
                "or install that package. See README.md."
            ) from exc
        raise
    return SimpleNamespace(DobotConnection=CommandConnection, RobotControl=RobotControl,
                           Motion=Motion, CoordinateType=CoordinateType, MyType=MyType)


def validate_connection_config(config):
    if not isinstance(config.get("robot_ip"), str) or not config["robot_ip"].strip():
        raise ValueError("robot_ip must be a non-empty address.")
    for key in ("user_index", "tool_index"):
        index = config.get(key, 0)
        if type(index) is not int or not 0 <= index <= 9:
            raise ValueError(f"{key} must be an integer from 0 to 9.")


@contextmanager
def connect(config, for_movement=False, *, select_frames=True):
    validate_connection_config(config)
    sdk = load_sdk()
    ip = config["robot_ip"]
    with ExitStack() as stack:
        connection = sdk.DobotConnection(ip, 29999)
        stack.callback(connection.disconnect)
        connection.set_timeout(connect_timeout=8, receive_timeout=90)
        try:
            connection.connect()
        except Exception as exc:
            raise ConnectionError(f"Cannot connect to {ip}: {exc}") from exc
        # V4 sends both control and motion through 29999, using one connection/lock.
        dashboard = sdk.RobotControl(connection)
        move = sdk.Motion(connection) if for_movement else None
        if select_frames:
            send_command(dashboard.User, config.get("user_index", 0))
            send_command(dashboard.Tool, config.get("tool_index", 0))
        yield dashboard, move


def main():
    parser = argparse.ArgumentParser(description="Shared robot utilities")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--enable", action="store_true",
                        help="send EnableRobot using the dashboard connection")
    action.add_argument("--status", action="store_true",
                        help="read RobotMode/GetErrorID without selecting frames or enabling")
    action.add_argument("--request-control", action="store_true",
                        help="request V4 TCP control mode while the robot is disabled")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE,
                        help="shared configuration path")
    args = parser.parse_args()
    config = read_config(args.config)
    if args.status or args.request_control:
        with connect(config, for_movement=False, select_frames=False) as (dashboard, _):
            with command_timeout(dashboard, 5):
                if args.request_control:
                    print("RequestControl reply:", send_command(dashboard.RequestControl))
                else:
                    # GetErrorID can contain nested lists; preserve the raw diagnostic reply.
                    print("RobotMode reply:", dashboard.RobotMode())
                    print("GetErrorID reply:", dashboard.GetErrorID())
        return
    with connect(config, for_movement=False) as (dashboard, _):
        enable_robot(dashboard)
    print("Robot enable command accepted.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Robot operation failed: {exc}", file=sys.stderr)
        sys.exit(1)
