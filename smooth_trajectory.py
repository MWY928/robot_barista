#!/usr/bin/env python3
"""用途：离线重分配轨迹时间、降低指令加速度，生成新副本，不连接机械臂。
处理：调整时间戳，保留全部原始关节/位姿配对及状态；必要时成对线性补点，不做几何滤波，
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
MAX_OUTPUT_SAMPLES = 100000
ADAPTIVE_TARGET_SPEEDUP = 1.5


def paired_resample(source, stretch, max_gap, max_samples=MAX_OUTPUT_SAMPLES):
    """Keep every original vertex; split only stretched intervals, never raw gaps.

    Caller must structurally validate the original first. Synthetic pose is an
    approximate linear companion of joint interpolation, NOT forward kinematics.
    No synthetic controller timestamp/status is claimed to be measured feedback.
    """
    pieces = [max(1, math.ceil((b["t"]-a["t"])*stretch/(max_gap*.99)))
              for a, b in zip(source, source[1:])]
    if 1 + sum(pieces) > max_samples:
        raise ValueError("Cannot retime: output sample budget exceeded")
    result = [copy.deepcopy(source[0])]
    for index, (a, b, count) in enumerate(zip(source, source[1:], pieces)):
        start, end = a["t"]*stretch, b["t"]*stretch
        for part in range(1, count):
            fraction = part/count
            result.append({
                "t": start + (end-start)*fraction,
                "joint": [x+(y-x)*fraction for x, y in zip(a["joint"], b["joint"])],
                "pose": [x+(y-x)*fraction for x, y in zip(a["pose"], b["pose"])],
                "interpolation": {"source_segment": index, "fraction": fraction,
                                  "pose_approximate_not_fk": True},
            })
        sample = copy.deepcopy(b)
        sample["t"] = end
        result.append(sample)
    return result


def retime_paired_recording(original_data, config, name=None, *, max_stretch=MAX_STRETCH,
                           stretch_step=.025, max_samples=MAX_OUTPUT_SAMPLES):
    """Bounded ascending uniform-time search, with unchanged player validation.

    First passing grid candidate, not a continuous/global optimum: servo phase
    and resampling make acceptance non-monotone. At most 1001 candidates,
    100000 samples and the player's 100000 servo cycles per candidate.
    Internal pauses and all original paired vertices are retained, not smoothed.
    """
    trajectory_player.validate_recording(original_data, config, check_dynamics=False)
    if (not math.isfinite(max_stretch) or not 1 <= max_stretch <= MAX_STRETCH
            or not math.isfinite(stretch_step) or stretch_step <= 0
            or type(max_samples) is not int or not 2 <= max_samples <= MAX_OUTPUT_SAMPLES):
        raise ValueError("Invalid retiming bounds")
    steps = math.ceil((max_stretch-1)/stretch_step)
    if steps > 1000:
        raise ValueError("Cannot retime: search budget exceeded")
    limits = trajectory_player.limits_for(config)
    source = original_data["samples"]
    last_error = "no candidate"
    for step in range(steps+1):
        stretch = min(max_stretch, 1 + step*stretch_step)
        if math.ceil(source[-1]["t"]*stretch/limits["playback_speed"] /
                     limits["servo_period_s"]) > 100000:
            last_error = "servo cycle budget exceeded"
            break
        samples = paired_resample(source, stretch, limits["max_sample_gap_s"], max_samples)
        data = make_recording(copy.deepcopy(original_data), samples,
                              name or original_data.get("name", "recording"))
        data["time_processing"] = {
            "method": "uniform_time_stretch_paired_linear_v1",
            "uniform_stretch": stretch, "search_step": stretch_step,
            "source_duration_s": source[-1]["t"], "source_sample_count": len(source),
            "validated_playback_speed": limits["playback_speed"],
            "pose_interpolation": "approximate linear paired pose; NOT FK",
            "path_guarantee": "original vertices and piecewise linear joint path only; no jerk or collision guarantee",
        }
        data["pose_source"] = "Original recorded vertices; inserted poses approximate paired linear interpolation, NOT FK"
        data["units"] = dict(data.get("units", {}), t="seconds (retimed playback timebase)")
        try:
            trajectory_player.validate_recording(data, config)
        except ValueError as exc:
            last_error = str(exc)
            continue
        return data
    raise ValueError(f"Cannot retime within unchanged limits and computational bounds: {last_error}")


def adaptive_retime_recording(original_data, config, name=None, *, target_speedup=ADAPTIVE_TARGET_SPEEDUP):
    """Retiming that spends time on corners and high-speed/stop transitions.

    This changes timestamps only.  Every recorded vertex remains, and any
    required gap repair uses paired linear joint/pose samples.  Candidates are
    always checked by the player's complete validator, including the fixed
    ServoJ grid and start/stop acceleration checks.
    """
    config = copy.deepcopy(config)
    playback_speed = trajectory_player.limits_for(config)["playback_speed"]
    if playback_speed > 1:
        return fast_retime_recording(original_data, config, name)
    trajectory_player.validate_recording(original_data, config, check_dynamics=False)
    if not math.isfinite(target_speedup) or target_speedup <= 1:
        raise ValueError("target_speedup must be greater than 1")
    source = original_data["samples"]
    old_duration = source[-1]["t"]
    # A dimensionless difficulty score: velocity, corner change, and paired
    # TCP/orientation motion.  The floor keeps quiet segments near 1.5x.
    scores = []
    velocities = []
    for a, b in zip(source, source[1:]):
        dt = b["t"] - a["t"]
        joint = [abs(y-x)/dt for x, y in zip(a["joint"], b["joint"])]
        state = trajectory_player.state_errors(a, b)
        velocities.append(joint)
        scores.append(max(max(joint)/30, state["position_mm"]/100,
                            state["orientation_deg"]/30))
    for i in range(1, len(scores)):
        scores[i] = max(scores[i], min(1.5, max(abs(velocities[i][j]-velocities[i-1][j])
                                                for j in range(6))/90))
    max_score = max(1., max(scores))
    weights = [1. + .5 * min(1., score/max_score) for score in scores]
    # Search the fastest global multiplier that passes all unchanged limits.
    base = [((b["t"]-a["t"]) * weights[i]) for i, (a, b) in enumerate(zip(source, source[1:]))]
    last_error = "no candidate"
    desired_ratio = 1 / target_speedup
    candidates = sorted((x/1000 for x in range(350, 1001, 5)),
                        key=lambda value: abs(value * sum(base) / old_duration - desired_ratio))
    for multiplier in candidates:
        samples = copy.deepcopy(source)
        t = 0.
        samples[0]["t"] = 0.
        for interval, sample in zip(base, samples[1:]):
            t += interval * multiplier
            sample["t"] = t
        if t / old_duration > 1.05:  # do not accidentally make a slower take
            continue
        data = make_recording(copy.deepcopy(original_data), samples, name or original_data.get("name", "recording"))
        data["time_processing"] = {
            "method": "adaptive_segment_time_retiming_v1",
            "target_speedup": target_speedup,
            "validated_playback_speed": playback_speed,
            "target_duration_ratio": 1 / target_speedup,
            "duration_ratio": t / old_duration,
            "difficulty_weights": "joint/tcp/orientation speed and adjacent velocity change",
            "all_original_vertices_retained": True,
            "pose_interpolation": "none unless paired gap repair is required; NOT FK",
        }
        try:
            trajectory_player.validate_recording(data, config)
        except ValueError as exc:
            last_error = str(exc)
            continue
        return data
    raise ValueError(f"Cannot adaptively retime within unchanged limits: {last_error}")


def fast_retime_recording(original_data, config, name=None):
    """Fastest passing bounded grid candidate, not a global optimum.

    Keep every vertex, including stationary samples. Use speed-derived local
    durations and enlarge corner intervals for acceleration; full validation
    remains authoritative (including paired interpolation and servo phase).
    """
    trajectory_player.validate_recording(original_data, config, check_dynamics=False)
    limits = trajectory_player.limits_for(config)
    speed = limits["playback_speed"]
    source = original_data["samples"]
    delta = [[y-x for x, y in zip(a["joint"], b["joint"])]
             for a, b in zip(source, source[1:])]
    durations = []
    for a, b, dq in zip(source, source[1:], delta):
        errors = trajectory_player.state_errors(a, b)
        durations.append(max(.001, max(map(abs, dq))/limits["max_joint_speed_deg_s"],
                             errors["position_mm"]/limits["max_cartesian_speed_mm_s"],
                             errors["orientation_deg"]/limits["max_orientation_speed_deg_s"]))
    for _ in range(100):
        factors = [1.] * len(durations)
        for i in range(1, len(durations)):
            acceleration = max(abs(delta[i][j]/durations[i]-delta[i-1][j]/durations[i-1])
                               for j in range(6))/((durations[i]+durations[i-1])/2)
            factor = math.sqrt(acceleration/limits["max_joint_acceleration_deg_s2"])
            factors[i] = max(factors[i], factor)
            factors[i-1] = max(factors[i-1], factor)
        if max(factors) <= 1.000001:
            break
        durations = [dt*f for dt, f in zip(durations, factors)]
    last_error = "no candidate"
    for step in range(901):
        multiplier = 1. + step*.01
        samples = copy.deepcopy(source)
        samples[0]["t"] = 0.
        t = 0.
        for sample, dt in zip(samples[1:], durations):
            t += dt * multiplier * speed
            sample["t"] = t
        samples = paired_resample(samples, 1., limits["max_sample_gap_s"])
        data = make_recording(copy.deepcopy(original_data), samples,
                              name or original_data.get("name", "recording"))
        data["time_processing"] = {
            "method": "speed_corner_adaptive_grid_v2", "validated_playback_speed": speed,
            "multiplier": multiplier, "grid_step": .01,
            "optimality": "first passing ascending grid candidate; not global optimum",
            "source_duration_s": source[-1]["t"], "source_sample_count": len(source),
            "all_original_vertices_retained": True,
            "pose_interpolation": "paired linear approximation; NOT FK",
        }
        try:
            trajectory_player.validate_recording(data, config)
        except ValueError as exc:
            last_error = str(exc)
            continue
        return data
    raise ValueError(f"Cannot retime at playback_speed={speed:g} within unchanged limits "
                     f"and bounded search (not a proof of physical impossibility): {last_error}")


def dynamics_report(data, config):
    """Offline maxima, using the same actual ServoJ target grid as the player.

    TCP/orientation are recorded/approximate pose metrics, not measured execution.
    Validation always runs first, including initial/final rest commands.
    """
    trajectory = trajectory_player.validate_recording(data, config)
    limits = trajectory_player.limits_for(config)
    report = dict(sample_joint_speed_deg_s=0., sample_joint_acceleration_deg_s2=0.,
                  tcp_speed_mm_s=0., orientation_speed_deg_s=0.,
                  servo_joint_speed_deg_s=0., servo_joint_acceleration_deg_s2=0.)
    previous_velocity = None
    previous_dt = None
    for a, b in zip(trajectory.samples, trajectory.samples[1:]):
        dt = (b["t"]-a["t"])/limits["playback_speed"]
        velocity = [(y-x)/dt for x,y in zip(a["joint"], b["joint"])]
        report["sample_joint_speed_deg_s"] = max(report["sample_joint_speed_deg_s"], *map(abs, velocity))
        if previous_velocity is not None:
            acceleration = max(abs(x-y) for x,y in zip(velocity, previous_velocity))/((dt+previous_dt)/2)
            report["sample_joint_acceleration_deg_s2"] = max(report["sample_joint_acceleration_deg_s2"], acceleration)
        errors = trajectory_player.state_errors(a, b)
        report["tcp_speed_mm_s"] = max(report["tcp_speed_mm_s"], errors["position_mm"]/dt)
        report["orientation_speed_deg_s"] = max(report["orientation_speed_deg_s"], errors["orientation_deg"]/dt)
        previous_velocity, previous_dt = velocity, dt
    period = limits["servo_period_s"]
    count = math.ceil(trajectory.duration/limits["playback_speed"]/period)
    previous, velocity = trajectory.first["joint"], [0.]*6
    for index in range(1, count+2):
        joint = trajectory.joint_at(index*period*limits["playback_speed"])
        next_velocity = [(b-a)/period for a,b in zip(previous, joint)]
        report["servo_joint_speed_deg_s"] = max(report["servo_joint_speed_deg_s"], *map(abs, next_velocity))
        report["servo_joint_acceleration_deg_s2"] = max(
            report["servo_joint_acceleration_deg_s2"],
            max(abs(a-b)/period for a,b in zip(velocity, next_velocity)))
        previous, velocity = joint, next_velocity
    report.update(duration_s=trajectory.duration,
                  playback_duration_s=trajectory.duration/limits["playback_speed"],
                  sample_count=len(trajectory.samples),
                  servo_cycles=count, max_sample_gap_s=max(b-a for a,b in zip(trajectory.times, trajectory.times[1:])))
    return report


def idle_bounds(samples):
    """Contiguous endpoint-relative idle only; never inspect/remove interior pauses."""
    def near(a, b):
        errors = trajectory_player.state_errors(a, b)
        return (errors["joint_deg"] <= .1 and errors["position_mm"] <= 1
                and errors["orientation_deg"] <= .1)
    first, last = 0, len(samples) - 1
    while first < last and near(samples[first + 1], samples[0]):
        first += 1
    while last > 0 and near(samples[last - 1], samples[-1]):
        last -= 1
    return first, last


def trim_idle_recording(original_data, config, name=None, *, retime=False):
    """Search conservative buffers; validate the entire fixed-period output.

    Keep exact endpoint states and an uninterrupted interior slice. Only collapse
    idle intervals between the original endpoints and that slice. No filtering,
    synthetic poses, speed changes or limit relaxation. Failed captures are
    rejected BEFORE removing anything. Stationary takes remain intact.
    """
    trajectory_player.validate_recording(original_data, config, check_dynamics=False)
    source = original_data["samples"]
    first, last = idle_bounds(source)
    duration = source[-1]["t"]
    # Start with 0.20 source seconds and at least two samples on each side;
    # expand toward the untouched original if a splice fails validation.
    last_error = None
    for attempt in range(22):
        buffer = .2 * 1.5 ** attempt
        left, right = first, last
        if first >= last or attempt == 21:
            left, right = 1, len(source) - 2
        else:
            while left > 1 and (source[first]["t"] - source[left]["t"] < buffer
                                or first - left < 2):
                left -= 1
            while right < len(source) - 2 and (source[right]["t"] - source[last]["t"] < buffer
                                              or right - last < 2):
                right += 1
            left = max(1, left)
            right = min(len(source) - 2, right)
        indices = [0] + list(range(left, right + 1)) + [len(source) - 1]
        samples = copy.deepcopy([source[i] for i in indices])
        # Preserve the original endpoint-adjacent interval, not the removed gap.
        removed_start = source[left]["t"] - source[1]["t"]
        removed_end = source[-2]["t"] - source[right]["t"]
        for sample, index in zip(samples, indices):
            sample.setdefault("source_t", source[index]["t"])
            sample["t"] = (0 if index == 0 else source[index]["t"] - removed_start)
        samples[-1]["t"] -= removed_end
        data = make_recording(copy.deepcopy(original_data), samples,
                              name or original_data.get("name", "recording"))
        data["idle_trim"] = {
            "method": "endpoint_idle_inner_removal_v1",
            "source_name": original_data.get("name"),
            "source_created_at_utc": original_data.get("created_at_utc"),
            "source_duration_s": duration,
            "source_sample_count": len(source),
            "retained_source_indices": indices,
            "thresholds": {"joint_deg": .1, "position_mm": 1., "orientation_deg": .1},
            "removed_leading_s": removed_start, "removed_trailing_s": removed_end,
            "validated_playback_speed": trajectory_player.limits_for(config)["playback_speed"],
        }
        try:
            if retime:
                data = retime_recording(data, config, name)
            else:
                trajectory_player.validate_recording(data, config)
        except ValueError as exc:
            last_error = exc
            continue
        start, end = idle_bounds(data["samples"])
        data["idle_trim"].update(
            remaining_leading_s=data["samples"][start]["t"],
            remaining_trailing_s=data["samples"][-1]["t"] - data["samples"][end]["t"])
        return data
    raise ValueError(f"Cannot trim within unchanged dynamics/gap limits: {last_error}")


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
    # Preserve successful legacy recorder results; only use bounded paired
    # resampling when timestamp-only retiming cannot pass the unchanged checks.
    return retime_paired_recording(original_data, config, name)


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
    data = adaptive_retime_recording(original_data, config, output_name)
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
