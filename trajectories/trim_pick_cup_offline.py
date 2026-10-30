"""One-shot offline derivation; preserves source samples without interpolation."""

import copy
import hashlib
import io
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import trajectory_player


def main():
    source = ROOT / "trajectories/pick_cup_smooth.json"
    target = source.with_name("pick_cup_smooth_trimmed.json")
    name = "pick_cup_smooth_trimmed"
    raw = source.read_bytes()
    original = json.loads(raw)
    config = json.loads((ROOT / "robot_config.json").read_text())
    if target.exists() or name in config["trajectories"]:
        raise RuntimeError("Refusing to overwrite an existing output or logical name")
    duration = original["samples"][-1]["t"]
    cutoff = duration - 3.0
    result = copy.deepcopy(original)
    samples = [s for s in result["samples"] if s["t"] <= cutoff]
    if len(samples) < 2:
        raise ValueError("Insufficient samples after trimming")
    result.update(
        name=name,
        samples=samples,
        sample_count=len(samples),
        recorded_duration_s=samples[-1]["t"],
        average_rate_hz=(len(samples)-1) / samples[-1]["t"],
        max_sample_gap_s=max(b["t"]-a["t"] for a, b in zip(samples, samples[1:])),
        has_multiple_samples=True,
        replay_verified=False,
    )
    result["provenance"] = {
        "operation": "trim_recorded_timeline_tail_without_interpolation",
        "derived_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_file": source.name,
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "source_sample_count": len(original["samples"]),
        "source_recorded_duration_s": duration,
        "requested_removed_recorded_seconds": 3.0,
        "requested_cutoff_s": cutoff,
        "actual_cutoff_s": samples[-1]["t"],
        "actual_removed_recorded_seconds": duration-samples[-1]["t"],
        "removed_sample_count": len(original["samples"])-len(samples),
        "selection": "original samples with t <= source final timestamp - 3.0",
        "preserved_acquisition_metadata": [
            "created_at_utc", "requested_duration_s", "requested_rate_hz",
            "received_frame_count", "stop_reason",
        ],
        "acquisition_metadata_note": "These fields describe the original recording, not a new acquisition.",
        "offline_validation": {
            "validator": "trajectory_player.load_trajectory",
            "trajectory_settings": trajectory_player.limits_for(config),
            "includes_fixed_period_start_and_stop_at_rest": True,
            "passed": True,
            "physical_replay_performed": False,
        },
    }
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    trial = copy.deepcopy(config)
    trial["trajectories"][name] = {"file": str(target)}
    original_open = Path.open

    def candidate_open(path, *args, **kwargs):
        if path == target:
            return io.StringIO(text)
        return original_open(path, *args, **kwargs)

    with patch("socket.socket", side_effect=AssertionError("Network forbidden")):
        with patch.object(Path, "open", candidate_open):
            trajectory_player.load_trajectory(name, trial)
        with target.open("x", encoding="utf-8") as stream:
            stream.write(text)
        loaded = trajectory_player.load_trajectory(name, trial)
    assert source.read_bytes() == raw
    assert loaded.samples == original["samples"][:len(samples)]
    print(json.dumps({"output": str(target), "sample_count": len(samples),
                      "duration": loaded.duration, "validation": "PASS",
                      "source_unchanged": True}, indent=2))


if __name__ == "__main__":
    main()