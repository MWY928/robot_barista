"""用途：校验、对齐起点并回放已录制的 V4 轨迹。
主要入口：
  load_trajectory()：离线读取并校验轨迹，不连接机械臂。
  single_trajectory()：连接、确认、使能并执行单段；align_only=True 只对齐起点。
  TrajectoryPlayer.check_start()：只检查起点；ensure_start()：可能执行 MovL 对齐。
  TrajectoryPlayer.play()：自动对齐、ServoJ 回放并等待实际到位。
根目录运行（以下两条都会运动；--commissioning 允许试跑未验证轨迹）：
  python trajectory_player.py pick_cup --commissioning --align-only
  python trajectory_player.py pick_cup --commissioning
"""

import argparse
import bisect
import copy
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

from record_trajectory import FEEDBACK_PORT, RECORDING_SOURCE, FeedbackReader, frame_values
from robot_common import (CONFIG_FILE, ROOT, command_timeout, connect, enable_robot, is_pose,
                          load_sdk, read_config, read_vector, send_command,
                          send_queued_command, validate_connection_config)

# Commission these values on the actual workstation before production use.
DEFAULT_LIMITS = {
    "start_position_margin_mm": 5.0,
    "start_orientation_margin_deg": 3.0,
    "start_joint_margin_deg": 5.0,
    "arrival_position_mm": 0.5,
    "arrival_orientation_deg": 0.5,
    "arrival_joint_deg": 1.0,
    "feedback_max_age_s": 0.25,
    "endpoint_timeout_s": 5.0,
    "linear_timeout_s": 60.0,
    "command_timeout_s": 2.0,
    "settle_joint_speed_deg_s": 0.5,
    "servo_period_s": 0.04,
    "playback_speed": 0.35,
    "max_command_lag_s": 0.04,
    "max_tracking_error_deg": 5.0,
    "max_sample_gap_s": 0.2,
    "max_joint_speed_deg_s": 30.0,
    "max_joint_acceleration_deg_s2": 90.0,
    "max_cartesian_speed_mm_s": 100.0,
    "max_orientation_speed_deg_s": 30.0,
    "servo_lookahead_time": 50.0,
    "servo_gain": 500.0,
}


def limits_for(config):
    limits = dict(DEFAULT_LIMITS)
    limits.update(config.get("trajectory_settings", {}))
    for key, value in limits.items():
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"trajectory_settings.{key} must be a positive finite number.")
    if not 0.02 <= limits["servo_period_s"] <= 0.1:
        raise ValueError("servo_period_s must be between 0.02 and 0.1 for this V4 player.")
    if limits["playback_speed"] > 1:
        raise ValueError("playback_speed must be <= 1 (no faster than the recorded timing).")
    if limits["max_command_lag_s"] > limits["servo_period_s"]:
        raise ValueError("max_command_lag_s must not exceed one servo period.")
    if not 20 <= limits["servo_lookahead_time"] <= 100 or not 200 <= limits["servo_gain"] <= 1000:
        raise ValueError("ServoJ lookahead/gain is outside the bundled SDK's range.")
    for small, large in (("arrival_position_mm", "start_position_margin_mm"),
                         ("arrival_orientation_deg", "start_orientation_margin_deg"),
                         ("arrival_joint_deg", "start_joint_margin_deg")):
        if limits[small] > limits[large]:
            raise ValueError(f"{small} must not exceed {large}.")
    return limits


def rotation_matrix(angles):
    """Dobot Rx/Ry/Rz in degrees, R = Rz * Ry * Rx."""
    x, y, z = map(math.radians, angles)
    cx, cy, cz = math.cos(x), math.cos(y), math.cos(z)
    sx, sy, sz = math.sin(x), math.sin(y), math.sin(z)
    return ((cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx),
            (sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx),
            (-sy, cy*sx, cy*cx))


def orientation_error(a, b):
    ra, rb = rotation_matrix(a), rotation_matrix(b)
    trace = sum(ra[i][j] * rb[i][j] for i in range(3) for j in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1) / 2))))


def state_errors(current, target):
    # Do not wrap joint differences: a different revolution is a different arm state.
    return {
        "position_mm": math.sqrt(sum((a-b)**2 for a, b in zip(current["pose"][:3], target["pose"][:3]))),
        "orientation_deg": orientation_error(current["pose"][3:], target["pose"][3:]),
        "joint_deg": max(abs(a-b) for a, b in zip(current["joint"], target["joint"])),
    }


def within(errors, limits, arrival=False):
    keys = (("arrival_position_mm", "arrival_orientation_deg", "arrival_joint_deg") if arrival else
            ("start_position_margin_mm", "start_orientation_margin_deg", "start_joint_margin_deg"))
    return all(errors[key] <= limits[limit] + 1e-9 for key, limit in
               zip(("position_mm", "orientation_deg", "joint_deg"), keys))


def describe_errors(errors):
    return ", ".join(f"{key}={value:.3f}" for key, value in errors.items())


def check_frames(frames, config):
    if not isinstance(frames, dict):
        raise ValueError("Recording must contain feedback_frames from the trajectory recorder.")
    for key in ("user_index", "tool_index"):
        if type(frames.get(key)) is not int or frames[key] != config.get(key, 0):
            raise ValueError(f"Recording {key} does not match shared configuration.")
    if not all(is_pose(frames.get(key)) for key in ("user_frame", "tool_frame")):
        raise ValueError("Recording coordinate-frame definitions are invalid.")


class Trajectory:
    def __init__(self, name, path, data):
        self.name, self.path, self.data = name, path, data
        self.samples = data["samples"]
        self.times = [sample["t"] for sample in self.samples]
        self.first, self.last = self.samples[0], self.samples[-1]
        self.duration = self.times[-1]
        self.frames = data["feedback_frames"]

    def joint_at(self, recorded_time):
        if recorded_time <= 0:
            return list(self.first["joint"])
        if recorded_time >= self.duration:
            return list(self.last["joint"])
        index = bisect.bisect_right(self.times, recorded_time) - 1
        a, b = self.samples[index:index+2]
        fraction = (recorded_time-a["t"]) / (b["t"]-a["t"])
        return [x + fraction*(y-x) for x, y in zip(a["joint"], b["joint"])]


def load_trajectory(name, config, require_verified=False):
    """Validate the complete take before allowing any robot connection."""
    entry = config.get("trajectories", {}).get(name)
    if not isinstance(entry, dict) or not isinstance(entry.get("file"), str):
        raise ValueError(f"No file configured for trajectory {name!r}.")
    path = Path(entry["file"])
    path = path if path.is_absolute() else ROOT / path
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    return validate_recording(data, config, name, path, require_verified)


def validate_recording(data, config, name="recording", path=None,
                       require_verified=False, *, check_dynamics=True):
    """Shared in-memory validation. Structural-only mode never authorizes replay."""
    limits = limits_for(config)
    if not isinstance(data, dict):
        raise ValueError(f"{name}: recording must be a JSON object.")
    if data.get("schema_version") != 1 or data.get("source") != RECORDING_SOURCE:
        raise ValueError(f"{name}: expected a schema-1 V4 feedback recording; rerecord V3 takes on V4.")
    if data.get("stop_reason") not in ("duration", "interrupted") or data.get("error") is not None:
        raise ValueError(f"{name}: failed/partial-error recordings cannot be replayed.")
    check_frames(data.get("feedback_frames"), config)
    if require_verified and data.get("replay_verified") is not True:
        raise ValueError(f"{name}: replay_verified must be true after physical validation.")
    samples = data.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        raise ValueError(f"{name}: a trajectory needs at least two samples.")
    if data.get("sample_count") != len(samples):
        raise ValueError(f"{name}: sample_count does not match samples.")
    previous_velocity = None
    previous_dt = None
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict) or not is_pose(sample.get("joint")) or not is_pose(sample.get("pose")):
            raise ValueError(f"{name}: invalid sample {index}.")
        t = sample.get("t")
        if type(t) not in (int, float) or not math.isfinite(t) or t < 0:
            raise ValueError(f"{name}: invalid timestamp at sample {index}.")
        if sample.get("error_status", 0) != 0:
            raise ValueError(f"{name}: controller error was recorded at sample {index}.")
        if index == 0:
            if t != 0:
                raise ValueError(f"{name}: first sample must have t=0.")
            continue
        previous = samples[index-1]
        gap = t-previous["t"]
        if not 0 < gap <= limits["max_sample_gap_s"]:
            raise ValueError(f"{name}: non-increasing time or excessive sample gap at {index}.")
        if not check_dynamics:
            continue
        dt = gap / limits["playback_speed"]
        velocity = [(b-a)/dt for a, b in zip(previous["joint"], sample["joint"])]
        if max(map(abs, velocity)) > limits["max_joint_speed_deg_s"]:
            raise ValueError(f"{name}: joint speed limit exceeded at sample {index}.")
        distance = math.sqrt(sum((a-b)**2 for a, b in zip(previous["pose"][:3], sample["pose"][:3])))
        if distance/dt > limits["max_cartesian_speed_mm_s"]:
            raise ValueError(f"{name}: TCP speed limit exceeded at sample {index}.")
        if orientation_error(previous["pose"][3:], sample["pose"][3:])/dt > limits["max_orientation_speed_deg_s"]:
            raise ValueError(f"{name}: orientation speed limit exceeded at sample {index}.")
        if previous_velocity is not None:
            acceleration = max(abs(a-b) for a, b in zip(velocity, previous_velocity)) / ((dt+previous_dt)/2)
            if acceleration > limits["max_joint_acceleration_deg_s2"]:
                raise ValueError(f"{name}: joint acceleration limit exceeded at sample {index}.")
        previous_velocity, previous_dt = velocity, dt
    trajectory = Trajectory(name, path, data)
    if not check_dynamics:
        return trajectory
    # Validate the actual fixed-period targets too, including start and end at rest.
    period = limits["servo_period_s"]
    count = math.ceil(trajectory.duration / limits["playback_speed"] / period)
    if count > 100000:
        raise ValueError(f"{name}: playback exceeds 100000 servo cycles.")
    previous_joint = trajectory.first["joint"]
    velocity = [0.0] * 6
    for index in range(1, count+2):
        joint = trajectory.joint_at(index*period*limits["playback_speed"])
        next_velocity = [(b-a)/period for a, b in zip(previous_joint, joint)]
        if max(map(abs, next_velocity)) > limits["max_joint_speed_deg_s"]:
            raise ValueError(f"{name}: resampled joint speed exceeds the limit.")
        acceleration = [abs(a-b)/period for a, b in zip(velocity, next_velocity)]
        joint_index = max(range(6), key=acceleration.__getitem__)
        if acceleration[joint_index] > limits["max_joint_acceleration_deg_s2"]:
            phase = "start" if index == 1 else "stop" if index > count else "path"
            raise ValueError(
                f"{name}: resampled acceleration/start/stop exceeds the limit: "
                f"J{joint_index+1}={acceleration[joint_index]:.3f} deg/s^2 > "
                f"{limits['max_joint_acceleration_deg_s2']:.3f} deg/s^2 "
                f"(cycle={index}, playback_t={index*period:.3f}s, phase={phase}, "
                f"playback_speed={limits['playback_speed']:g}). "
                "Try a lower playback_speed or rerecord with smoother motion.")
        previous_joint, velocity = joint, next_velocity
    return trajectory


class LiveFeedback:
    """Continuously drain feedback and fail closed on stale or invalid state."""

    def __init__(self, config):
        self.config = config
        self.max_age = limits_for(config)["feedback_max_age_s"]
        self.condition = threading.Condition()
        self.stopping = threading.Event()
        self.latest = None
        self.error = None

    def __enter__(self):
        dtype = load_sdk().MyType
        self.socket = socket.create_connection((self.config["robot_ip"], FEEDBACK_PORT), timeout=8)
        self.socket.settimeout(self.max_age)
        self.reader = FeedbackReader(self.socket, dtype)
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()
        try:
            self.state()
        except BaseException:
            self.__exit__(*sys.exc_info())
            raise
        return self

    def _receive(self):
        timestamp = None
        try:
            while not self.stopping.is_set():
                self.socket.settimeout(self.max_age)
                frame = self.reader.read()
                current = int(frame["TimeStamp"])
                if timestamp is not None and current < timestamp:
                    raise RuntimeError("Live controller timestamp moved backwards.")
                if current == timestamp:
                    continue  # Duplicates must not make an old state appear fresh.
                timestamp = current
                joint, pose, frames = frame_values(frame, self.config)
                velocity = frame["QDActual"].tolist()
                if not is_pose(velocity):
                    raise ValueError("Live feedback contains invalid joint velocities.")
                state = {"joint": joint, "pose": pose, "frames": frames,
                         "joint_velocity": velocity,
                         "received_at": time.monotonic(),
                         "error_status": int(frame["ErrorStatus"]),
                         "enable_status": int(frame["EnableStatus"]),
                         "drag_status": int(frame["DragStatus"]),
                         "current_command_id": int(frame["CurrentCommandId"]),
                         "running_status": int(frame["RunningStatus"])}
                with self.condition:
                    self.latest = state
                    self.condition.notify_all()
        except Exception as exc:
            if not self.stopping.is_set():
                with self.condition:
                    self.error = exc
                    self.condition.notify_all()

    def state(self):
        with self.condition:
            self.condition.wait_for(lambda: self.latest is not None or self.error is not None,
                                    timeout=self.max_age)
            if self.error:
                raise RuntimeError(f"Feedback failed: {self.error}") from self.error
            if self.latest is None or time.monotonic()-self.latest["received_at"] > self.max_age:
                raise RuntimeError("Robot feedback is stale or unavailable.")
            if self.latest["error_status"]:
                raise RuntimeError("Robot reports an error; playback stopped.")
            return copy.deepcopy(self.latest)

    def __exit__(self, *exc):
        self.stopping.set()
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        self.thread.join(timeout=1)


class TrajectoryPlayer:
    def __init__(self, dashboard, move, config, feedback):
        self.dashboard, self.move, self.config, self.feedback = dashboard, move, config, feedback
        self.limits = limits_for(config)
        self.frames = None
        self.motion_started = False

    def state(self, require_enabled=True):
        state = self.feedback.state()
        if self.frames is not None and state["frames"] != self.frames:
            raise RuntimeError("Live Tool/User definitions differ from the recording.")
        if state["drag_status"]:
            raise RuntimeError("Exit drag mode before executing a recipe.")
        if require_enabled and not state["enable_status"]:
            raise RuntimeError("Robot is not enabled.")
        return state

    def use_frames(self, trajectory):
        self.frames = trajectory.frames
        self.state(require_enabled=False)

    def check_home(self, home):
        state = self.state(require_enabled=False)
        # Confirm GetPose/HOME and feedback use the same pose convention before any motion.
        pose = read_vector(self.dashboard.GetPose)
        if not within(state_errors(state, {"pose": pose, "joint": state["joint"]}), self.limits, arrival=True):
            raise RuntimeError("Dashboard and feedback poses differ; verify Tool/User/TCP conventions.")
        errors = state_errors(state, home)
        home_limits = dict(self.limits, arrival_joint_deg=self.config.get("home_joint_tolerance_deg", 3))
        if not within(errors, home_limits, arrival=True):
            raise RuntimeError("Robot is not at HOME: " + describe_errors(errors))

    def assert_target(self, target):
        errors = state_errors(self.state(), target)
        if not within(errors, self.limits, arrival=True):
            raise RuntimeError("Expected transition start was not reached: " + describe_errors(errors))

    def wait_for_target(self, target, after=0.0, timeout=None, command_id=None):
        deadline = time.monotonic()+(timeout or self.limits["endpoint_timeout_s"])
        while True:
            state = self.state()
            errors = state_errors(state, target)
            settled = max(map(abs, state["joint_velocity"])) <= self.limits["settle_joint_speed_deg_s"]
            # V4 has no Sync(): require the issued command to have executed and
            # motion to be idle, as well as checking fresh actual joints/TCP.
            queue_finished = (command_id is None or
                              (state["current_command_id"] >= command_id and not state["running_status"]))
            if (state["received_at"] >= after and queue_finished and settled and
                    within(errors, self.limits, arrival=True)):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("Endpoint was not reached: " + describe_errors(errors))
            time.sleep(min(0.02, max(0, deadline-time.monotonic())))

    def move_linear(self, target):
        self.state()
        started = time.monotonic()
        self.motion_started = True
        with command_timeout(self.move, self.limits["command_timeout_s"]):
            command_id = send_queued_command(
                self.move.MovL, target["pose"], load_sdk().CoordinateType.CARTESIAN,
                user=self.config.get("user_index", 0), tool=self.config.get("tool_index", 0), cp=0)
            self.wait_for_target(target, after=started, timeout=self.limits["linear_timeout_s"],
                                 command_id=command_id)
            self.assert_target(target)

    def check_start(self, trajectory, require_enabled=False):
        """Read live state and reject mismatched frames or excessive start error."""
        self.frames = trajectory.frames
        state = self.state(require_enabled=require_enabled)
        errors = state_errors(state, trajectory.first)
        if not within(errors, self.limits):
            raise RuntimeError(f"{trajectory.name}: start mismatch exceeds margin: " + describe_errors(errors))
        return errors

    def ensure_start(self, trajectory):
        """Check, correct a small error, and confirm actual arrival before playback."""
        errors = self.check_start(trajectory, require_enabled=True)
        if not within(errors, self.limits, arrival=True):
            print("Aligning to segment start: " + describe_errors(errors))
            self.move_linear(trajectory.first)
        # Check again even if no correction was needed.
        if not within(state_errors(self.state(), trajectory.first), self.limits, arrival=True):
            raise RuntimeError(f"{trajectory.name}: start alignment could not be confirmed.")

    def play(self, trajectory):
        self.ensure_start(trajectory)
        speed, period = self.limits["playback_speed"], self.limits["servo_period_s"]
        count = math.ceil(trajectory.duration / speed / period)
        started = time.monotonic()
        with command_timeout(self.move, period+self.limits["max_command_lag_s"]):
            for index in range(count):
                deadline = started+index*period
                delay = deadline-time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                now = time.monotonic()
                if now-deadline > self.limits["max_command_lag_s"]:
                    raise RuntimeError("ServoJ scheduling deadline missed; no catch-up burst will be sent.")
                state = self.state()
                expected = trajectory.joint_at(max(0, (now-started)*speed))
                if max(abs(a-b) for a, b in zip(state["joint"], expected)) > self.limits["max_tracking_error_deg"]:
                    raise RuntimeError("Trajectory tracking error exceeded the joint margin.")
                target = trajectory.joint_at((index+1)*period*speed)
                self.motion_started = True
                command_id = send_queued_command(
                    self.move.ServoJ, target, t=period,
                    aheadtime=self.limits["servo_lookahead_time"], gain=self.limits["servo_gain"])
                if time.monotonic()-deadline > period+self.limits["max_command_lag_s"]:
                    raise RuntimeError("ServoJ acknowledgement was too late.")
            # Let the last servo interval finish, then check actual state once per segment.
            delay = started+count*period-time.monotonic()
            if delay > 0:
                time.sleep(delay)
        self.wait_for_target(trajectory.last, after=time.monotonic(), command_id=command_id)
        self.assert_target(trajectory.last)

    def stop_after_error(self):
        if self.motion_started:
            try:
                with command_timeout(self.dashboard, self.limits["command_timeout_s"]):
                    send_command(self.dashboard.Stop)
                print("Stop request accepted after workflow interruption.", file=sys.stderr)
            except Exception as exc:
                print(f"Stop request failed; check the robot and stop it manually: {exc}", file=sys.stderr)


def _confirm_motion(message):
    print(message)
    if input("Type Y to continue; any other input aborts: ").strip().upper() != "Y":
        raise RuntimeError("Operator aborted the single-trajectory run.")


def single_trajectory(name, config=None, *, commissioning=False, align_only=False):
    """Run one real recorded segment, including connection, checks and alignment.

    Raises on invalid data, a bad start, feedback faults or failed arrival.
    commissioning=True allows a take whose replay_verified flag is false.
    This function moves real hardware, asks for confirmation and does not
    change verification flags or automatically disable the robot on exit.
    """
    config = read_config() if config is None else config
    validate_connection_config(config)
    trajectory = load_trajectory(name, config, require_verified=not commissioning)
    speed = config.get("speed_factor", 10)
    if type(speed) is not int or not 1 <= speed <= 100:
        raise ValueError("speed_factor must be an integer from 1 to 100.")
    print(f"Recorded trajectory: {name} ({trajectory.path})")
    print(f"Start pose (mm/deg): {trajectory.first['pose']}")
    print(f"Start joints (deg): {trajectory.first['joint']}")
    _confirm_motion("PHYSICAL MOTION: position near this start and exit drag mode. "
                    "Confirm the alignment path is clear; the robot will be enabled.")
    with connect(config, for_movement=True) as (dashboard, move):
        with LiveFeedback(config) as feedback:
            player = TrajectoryPlayer(dashboard, move, config, feedback)
            # Reject large start errors before enabling, then recheck before motion.
            player.check_start(trajectory)
            state = player.state(require_enabled=False)
            dashboard_pose = read_vector(dashboard.GetPose)
            if not within(state_errors(state, {"pose": dashboard_pose, "joint": state["joint"]}),
                          player.limits, arrival=True):
                raise RuntimeError("Dashboard and feedback poses differ; verify Tool/User/TCP conventions.")
            try:
                send_command(dashboard.SpeedFactor, speed)
                enable_robot(dashboard)
                deadline = time.monotonic()+player.limits["endpoint_timeout_s"]
                while not feedback.state()["enable_status"]:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Enabled feedback did not arrive.")
                    time.sleep(0.02)
                if align_only:
                    player.ensure_start(trajectory)
                    print("Start reached and checked.")
                else:
                    _confirm_motion("Align to the start and play this segment once? "
                                    "Confirm the complete arm/cup path is clear.")
                    player.play(trajectory)
                    print("Recorded endpoint reached and checked.")
                return player.state()
            except (Exception, KeyboardInterrupt):
                player.stop_after_error()
                raise


def main():
    parser = argparse.ArgumentParser(description="Run one recorded trajectory using shared runtime checks")
    parser.add_argument("name", help="logical trajectory name in robot_config.json")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--commissioning", action="store_true", help="allow an unverified take for physical validation")
    parser.add_argument("--align-only", action="store_true", help="check and align to the recorded start, then finish")
    args = parser.parse_args()
    single_trajectory(args.name, read_config(args.config), commissioning=args.commissioning,
                      align_only=args.align_only)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Single trajectory aborted: {exc}", file=sys.stderr)
        print("Check the actual robot state; disconnecting does not guarantee stopping.", file=sys.stderr)
        sys.exit(1)
