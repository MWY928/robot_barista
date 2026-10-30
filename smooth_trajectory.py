#!/usr/bin/env python3
"""用途：离线重分配轨迹时间、降低指令加速度，生成新副本，不连接机械臂。
处理：只调整时间戳，保留全部原始关节/位姿配对及状态，不做几何滤波或重采样，
并用 trajectory_player.load_trajectory() 的同一套校验验证新轨迹。
根目录运行（输出新文件，不覆盖原轨迹）：
  python smooth_trajectory.py pick_cup pick_cup_smooth
"""

import argparse
import copy
import json
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

from robot_common import CONFIG_FILE, ROOT, read_config, write_config
import trajectory_player

DEFAULT_RATE = 50.0
DEFAULT_SIGMA = 3.0
MAX_STRETCH = 10.0


def build_samples(original, sigma, stretch, rate):
    """Time warp only; legacy sigma/rate arguments do not filter geometry.

    Derivative of the mapping before stretching is in [1, 2].
    Every original paired sample and status field is preserved.
    """
    duration = original[-1]["t"]
    samples = copy.deepcopy(original)
    for sample in samples:
        t = sample["t"]
        sample["t"] = stretch * (1.5*t + duration/(4*math.pi)*math.sin(2*math.pi*t/duration))
    return samples


def make_recording(original_data, samples, name):
    data = dict(original_data)
    gaps = [b["t"] - a["t"] for a, b in zip(samples, samples[1:])]
    data.update({
        "name": name,
        "samples": samples,
        "sample_count": len(samples),
        "recorded_duration_s": samples[-1]["t"],
        "average_rate_hz": (len(samples) - 1) / samples[-1]["t"] if samples[-1]["t"] > 0 else 0,
        "max_sample_gap_s": max(gaps) if gaps else 0,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "replay_verified": False,
    })
    return data


def retime_recording(original_data, config, name=None):
    """Reject corrupt raw captures first; never repair errors or large gaps."""
    trajectory_player.validate_recording(original_data, config, check_dynamics=False)
    last_error = None
    for step in range(181):
        stretch = 1 + step * 0.05
        samples = build_samples(original_data["samples"], None, stretch, None)
        if max(b["t"]-a["t"] for a, b in zip(samples, samples[1:])) > trajectory_player.limits_for(config)["max_sample_gap_s"]:
            break
        data = make_recording(original_data, samples, name or original_data.get("name", "recording"))
        data["time_processing"] = {"method": "monotone_cosine_time_warp_v1",
                                   "uniform_stretch": stretch,
                                   "source_duration_s": original_data["samples"][-1]["t"],
                                   "validated_playback_speed": trajectory_player.limits_for(config)["playback_speed"]}
        data["units"] = dict(data.get("units", {}), t="seconds (retimed; raw backup retains receive times)")
        try:
            trajectory_player.validate_recording(data, config)
        except ValueError as exc:
            last_error = exc
            continue
        return data
    raise ValueError(f"Cannot retime within unchanged dynamics/gap limits: {last_error or 'gap bound'}")


def smooth_trajectory(source_name, output_name, config_path=CONFIG_FILE,
                      sigma=DEFAULT_SIGMA, rate=DEFAULT_RATE):
    config = read_config(config_path)
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", output_name) is None:
        raise ValueError("Invalid output name.")
    if output_name in config.get("trajectories", {}):
        raise ValueError("Output name already configured; choose a new name.")
    entry = config.get("trajectories", {}).get(source_name)
    if not isinstance(entry, dict) or not isinstance(entry.get("file"), str):
        raise ValueError(f"No file configured for trajectory {source_name!r}.")
    source_path = Path(entry["file"])
    source_path = source_path if source_path.is_absolute() else ROOT / source_path
    with source_path.open(encoding="utf-8") as f:
        original_data = json.load(f)
    original = original_data.get("samples")
    if not isinstance(original, list) or len(original) < 2:
        raise ValueError(f"{source_name}: recording needs at least two samples.")

    relative = Path("trajectories") / f"{output_name}.json"
    output_path = ROOT / relative
    if output_path.exists():
        raise ValueError(f"Refusing to overwrite existing file: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    from record_trajectory import save_recording
    data = retime_recording(original_data, config, output_name)
    save_recording(data, output_path)
    chosen = (data["recorded_duration_s"]/original[-1]["t"], len(data["samples"]), data["recorded_duration_s"])

    if output_name not in config.get("trajectories", {}):
        config.setdefault("trajectories", {})[output_name] = {"file": str(relative)}
        write_config(config, config_path)
    return chosen, output_path


def main():
    parser = argparse.ArgumentParser(description="Smooth a recorded trajectory offline and validate the copy")
    parser.add_argument("source", help="existing trajectory name in robot_config.json")
    parser.add_argument("output", help="new trajectory name; creates trajectories/<output>.json")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--sigma", type=float, default=DEFAULT_SIGMA,
                        help="deprecated compatibility option; no geometry filtering")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE,
                        help="deprecated compatibility option; no resampling")
    args = parser.parse_args()
    if args.sigma <= 0 or not 1 <= args.rate <= 125:
        raise ValueError("sigma must be positive and rate must be 1..125 Hz.")
    (stretch, count, duration), path = smooth_trajectory(
        args.source, args.output, args.config, args.sigma, args.rate)
    print(f"Retimed trajectory saved: {path}")
    print(f"Time stretch: {stretch}x, duration: {duration:.2f}s, samples: {count}")
    print("Validated with the same offline checks as the player; replay_verified=False.")
    print("First physical run still needs --commissioning and a careful operator.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print(f"Smoothing aborted: {exc}", file=sys.stderr)
        sys.exit(1)
