#!/usr/bin/env python3
"""Passively record a timed joint/TCP trajectory from Dobot V3 feedback."""

import argparse
import json
import math
import re
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from robot_common import CONFIG_FILE, ROOT, is_pose, load_sdk, read_config, validate_connection_config

FEEDBACK_PORT = 30004
MAGIC = 0x123456789ABCDEF


class FeedbackReader:
    """Read complete frames even when TCP fragments or combines packets."""

    def __init__(self, connection, dtype):
        self.connection = connection
        self.dtype = dtype
        if dtype.itemsize != 1440:
            raise ValueError(f"Expected a V3 1440-byte feedback layout; got {dtype.itemsize}.")

    def read(self):
        import numpy as np
        data = bytearray()
        packet_deadline = time.perf_counter() + self.connection.gettimeout()
        while len(data) < self.dtype.itemsize:
            remaining = packet_deadline - time.perf_counter()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for a complete feedback packet.")
            self.connection.settimeout(remaining)
            chunk = self.connection.recv(self.dtype.itemsize - len(data))
            if not chunk:
                raise ConnectionError("Feedback connection closed before a complete packet arrived.")
            data.extend(chunk)
        frame = np.frombuffer(data, dtype=self.dtype, count=1)[0]
        if int(frame["len"]) != self.dtype.itemsize or int(frame["test_value"]) != MAGIC:
            raise ValueError("Invalid V3 feedback header. Check firmware/SDK compatibility.")
        return frame


def frame_values(frame, config):
    joint = frame["q_actual"].tolist()
    pose = frame["tool_vector_actual"].tolist()
    user_frame = frame["user[6]"].tolist()
    tool_frame = frame["tool[6]"].tolist()
    if not all(is_pose(value) for value in (joint, pose, user_frame, tool_frame)):
        raise ValueError("Feedback contains an invalid joint, pose or coordinate frame.")
    user, tool = int(frame["user"]), int(frame["tool"])
    if user != config.get("user_index", 0) or tool != config.get("tool_index", 0):
        raise ValueError(
            f"Feedback Tool/User ({tool}/{user}) differs from config "
            f"({config.get('tool_index', 0)}/{config.get('user_index', 0)}). "
            "Select matching frames before recording. The recorder does not change them."
        )
    return joint, pose, {"user_index": user, "tool_index": tool,
                         "user_frame": user_frame, "tool_frame": tool_frame}


def capture(connection, dtype, config, duration, rate, timeout, name):
    """Drain every feedback packet; retain samples at up to the requested rate."""
    reader = FeedbackReader(connection, dtype)
    started = time.perf_counter()
    deadline = started + duration
    samples = []
    frames = None
    last_timestamp = None
    received = 0
    first_sample_time = None
    next_sample_time = started
    reason, error = "duration", None
    try:
        while True:
            now = time.perf_counter()
            if now >= deadline:
                break
            connection.settimeout(min(timeout, deadline - now))
            try:
                frame = reader.read()
            except socket.timeout:
                if time.perf_counter() >= deadline:
                    break
                raise TimeoutError("No complete feedback packet arrived within the timeout.")
            received_at = time.perf_counter()
            # A final packet can arrive just beyond the requested duration.
            if received_at >= deadline:
                break
            joint, pose, active_frames = frame_values(frame, config)
            if frames is None:
                frames = active_frames
            elif frames != active_frames:
                raise ValueError("Tool/User frame definitions changed during recording.")
            timestamp = int(frame["time_stamp"])
            if last_timestamp is not None and timestamp < last_timestamp:
                raise ValueError("Controller timestamp moved backwards; recording stopped.")
            received += 1
            if timestamp == last_timestamp:
                continue
            last_timestamp = timestamp
            if received_at < next_sample_time:
                continue
            if first_sample_time is None:
                first_sample_time = received_at
            samples.append({
                "t": received_at - first_sample_time,
                "controller_timestamp_raw": timestamp,
                "joint": joint,
                "pose": pose,
                "robot_mode": int(frame["robot_mode"]),
                "drag_status": int(frame["drag_status"]),
                "error_status": int(frame["error_status"]),
            })
            # Never sleep here: the feedback stream must be drained continuously.
            next_sample_time = received_at + 1.0 / rate
    except KeyboardInterrupt:
        reason = "interrupted"
    except (OSError, ValueError, RuntimeError) as exc:
        reason, error = "error", str(exc)
    elapsed = samples[-1]["t"] if samples else 0.0
    intervals = [b["t"] - a["t"] for a, b in zip(samples, samples[1:])]
    result = {
        "schema_version": 1,
        "name": name,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "dobot_v3_feedback_30004",
        "robot_ip": config["robot_ip"],
        "configured_frames": {key: config.get(key, 0) for key in ("user_index", "tool_index")},
        "feedback_frames": frames,
        "units": {"t": "seconds (host monotonic receive time, first sample = 0)",
                  "joint": "degrees", "pose": "mm / degrees",
                  "controller_timestamp_raw": "controller value, unconverted"},
        "pose_source": "tool_vector_actual, as reported by controller; no frame conversion",
        "requested_duration_s": duration,
        "requested_rate_hz": rate,
        "sample_count": len(samples),
        "received_frame_count": received,
        "recorded_duration_s": elapsed,
        "average_rate_hz": (len(samples) - 1) / elapsed if elapsed > 0 else 0.0,
        "max_sample_gap_s": max(intervals, default=0.0),
        "stop_reason": reason,
        "error": error,
        "has_multiple_samples": len(samples) >= 2,
        "replay_verified": False,
        "samples": samples,
    }
    return result


def save_recording(recording, path):
    """Use exclusive creation: an existing take is never overwritten."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as f:
        json.dump(recording, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="take name, e.g. latte_art_001 (use a new name for each take)")
    parser.add_argument("--duration", type=float, default=30, help="maximum recording time, seconds (default: 30)")
    parser.add_argument("--rate", type=float, default=50, help="maximum retained samples/second (default: 50)")
    parser.add_argument("--timeout", type=float, default=2, help="feedback timeout, seconds (default: 2)")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--output", type=Path, help="output JSON; default: trajectories/NAME.json")
    args = parser.parse_args()
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.name) is None:
        parser.error("name must contain only letters, numbers, _ or -, starting with a letter or number")
    for key, maximum in (("duration", 600), ("rate", 125), ("timeout", 30)):
        value = getattr(args, key)
        if not math.isfinite(value) or not 0 < value <= maximum:
            parser.error(f"--{key} must be in (0, {maximum}]")
    output = args.output or ROOT / "trajectories" / f"{args.name}.json"
    if output.exists():
        raise FileExistsError(f"{output} already exists. Choose a new take name or output path.")
    config = read_config(args.config)
    validate_connection_config(config)
    dtype = load_sdk().MyType
    print("Passive recording only: no enable, movement, drag-mode or frame-selection commands.")
    print("Set up teaching/drag mode and matching Tool/User/TCP using the robot controls first.")
    print(f"Recording up to {args.duration:g}s at up to {args.rate:g} Hz. Ctrl+C ends and saves the take.")
    input("Press Enter when ready to connect and start recording: ")
    with socket.create_connection((config["robot_ip"], FEEDBACK_PORT), timeout=8) as connection:
        print("Recording started. Perform the trajectory now.")
        result = capture(connection, dtype, config, args.duration, args.rate, args.timeout, args.name)
    if not result["samples"]:
        raise RuntimeError(result["error"] or "No samples captured; no output file was written.")
    save_recording(result, output)
    print(f"Saved {result['sample_count']} samples to {output} ({result['stop_reason']}).")
    print(f"Average {result['average_rate_hz']:.1f} Hz; maximum sample gap {result['max_sample_gap_s']:.3f}s.")
    if result["error"]:
        raise RuntimeError(f"Partial take saved; recording failed: {result['error']}")
    if not result["has_multiple_samples"]:
        raise RuntimeError("Take saved, but fewer than two samples cannot describe a trajectory.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Recording ended: {exc}", file=sys.stderr)
        sys.exit(1)
