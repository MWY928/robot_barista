# Robot Barista

Python workflows for a DOBOT Nova 2 coffee workstation. Each recipe combines shared recorded motion segments with dispenser completion checks. Only the planned HOME entry/return connections and small start corrections use `MovL`.

## Project layout

- `nova2_points.json`: shared robot settings, HOME, trajectory file mapping, playback limits, and workflow verification flags.
- `record_trajectory.py`: passive recording of timed joint/TCP feedback.
- `trajectory_player.py`: recording validation, start alignment, continuous `ServoJ` playback, live tracking, and endpoint checks.
- `recipe_common.py`: offline workflow inspection and shared CLI helpers.
- `Americano/`, `IceAmericano/`: recipe `PLAN` definitions, dispenser waits, and `check_trajectories()` functions.
- `teach_point.py`, `robot_common.py`: shared point teaching, connections, and enabling.
- `trajectories/`: shared recordings and recording instructions.
- `TCP_IP_Python_V3/`: bundled [Dobot SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V3), including its license.

## Setup

Use Python 3.8+ and install NumPy:

```powershell
python -m pip install numpy
```

Configure the TCP, payload, Tool/User frames, compatible TCP/IP control mode, and actual `robot_ip`. Dashboard, motion, and live feedback use ports `29999`, `30003`, and `30004`. All three must work for recipe execution. Check controller compatibility with the bundled V3 `ServoJ` API before commissioning.

Run commands from the repository root. To teach HOME or explicitly enable the robot:

```powershell
python teach_point.py HOME
python robot_common.py --enable
```

Teaching reads a stationary pose and joints; enter `SAVE` to update the shared JSON. It resets all workflow verification flags. New waypoint names are supported, and `--list` lists existing points. Recipe `--teach POINT` commands remain aliases. Segmented recipes require only HOME in the point table; other existing taught points are preserved for future use.

## Record shared segments

Use one recording per movement between station waiting positions:

```powershell
python record_trajectory.py pick_cup --duration 30 --rate 50
```

Set up teaching/drag mode and matching Tool/User/TCP using the robot controls first. Press Enter, wait for `Recording started`, then perform the movement. Start and end with the arm at rest. `Ctrl+C` finishes and saves early; the duration also limits the recording. Operate dispensers separately so their waiting time does not become part of a transport segment.

Recordings default to `trajectories/NAME.json`. Existing files are not overwritten; use another take name and update the corresponding `trajectories` entry in the shared JSON. See [segment recording notes](trajectories/README.md) for the required names and boundaries.

Each file includes time-stamped `joint` and `pose` samples, Tool/User definitions, capture status, and `replay_verified`. Time `t` is host receive time; the original controller timestamp is retained separately. `--rate` is a maximum retained rate, not a guaranteed sampling frequency. The recorder sends no enable, motion, or drag-mode commands.

## Inspect, commission, and run

The following uses one recipe script; the other scripts expose the same commands:

```powershell
# Preview the plan and report missing recordings without connecting
python Americano/nova2_coffee_v1_1.py --dry-run

# Strict offline checks; return an error for missing/invalid files or bad junctions
python Americano/nova2_coffee_v1_1.py --check-trajectories

# PHYSICAL MOTION: commission each step with explicit operator confirmation
python Americano/nova2_coffee_v1_1.py --check-steps
```

`check_trajectories(config)` follows the recipe's actual order, checks every take and adjacent endpoints, and does not connect to hardware. It checks timestamps, gaps, frame metadata, controller errors, and configured motion limits. Each recipe keeps this function as its inspection entry point.

`--check-steps` **moves the robot** and allows takes whose verification flags are still false. It retains file, margin, live-feedback, timing, and endpoint checks; it requires `Y` before each motion step and waits for manual dispenser confirmation. Use it to physically verify the full arm/fixture/cup path, cup orientation, and HOME connections. It never automatically marks a take or workflow as verified.

After successful physical validation, set `replay_verified` to `true` in each validated recording and set `safe_routes_physically_verified[WORKFLOW]` to `true` in the shared JSON. Manually place the robot at HOME, then execute:

```powershell
python Americano/nova2_coffee_v1_1.py --run
```

Normal runs check all required files and verification flags before connecting or enabling. `--run --auto` skips motion prompts only; dispenser confirmations remain manual. Run one recipe at a time. Rerecording or changing frames, limits, replay timing, hardware, or payload requires clearing the affected verification flags and commissioning again.

## Playback behavior

`trajectory_settings` in the shared JSON applies to all recipes. Initial values are commissioning defaults, not validated workstation limits:

| Setting | Default | Purpose |
| --- | --- | --- |
| `start_position_margin_mm` | 5 | Maximum position error allowed for a start correction |
| `start_orientation_margin_deg` | 3 | Maximum rotation-angle error allowed for a start correction |
| `start_joint_margin_deg` | 5 | Maximum error of any joint; revolutions are not wrapped |
| `arrival_position_mm` / `arrival_orientation_deg` / `arrival_joint_deg` | 0.5 / 0.5 / 1 | Required arrival tolerances |
| `playback_speed` | 0.25 | Recorded-time playback ratio, at most 1 |
| `servo_period_s` | 0.04 | Fixed command period in seconds |

Before every segment, the player checks the actual live pose, joints, and coordinate frames. A start already within arrival tolerance needs no correction. A larger error within all three start margins triggers a `MovL` correction and another actual-state check. Any exceeded margin aborts the workflow.

Playback interpolates recorded joint samples onto the fixed command period and sends `ServoJ` targets with a timed schedule. It does not stop at every recorded point. A final segment check requires the arm to reach the recorded endpoint and settle. The initial HOME connection and final return are separately planned `MovL` moves, not large automatic start corrections.

Live stale feedback, controller errors, changed frames, excessive tracking error, and missed command deadlines abort execution. After motion has been issued, an interruption triggers a best-effort `ResetRobot` stop request. Always check the actual robot state; network disconnection and stop acknowledgements do not guarantee a physical stop.

This is host-driven V3 playback, not a controller-native `StartPath` importer or a hard real-time system. Resampling and servo following can change the exact path between samples. Validate timing, response, and clearances on the actual robot. The pose convention is checked against dashboard feedback at HOME; no automatic frame conversion or collision avoidance is provided. For very sensitive pouring or latte-art work, further controller/timing validation is required. `speed_factor` is sent for ordinary motion; use `playback_speed` to set this player's timing rather than assuming the global override governs servo motion.

## Extend the workstation

1. Add recordings to the shared trajectory mapping, reusing a segment only when its frames, start/end arm states, fixture, payload, and cup state are appropriate.
2. Copy a recipe directory under the repository root, select a unique `WORKFLOW`, and add its verification entry with value `false`.
3. Arrange `trajectory` and dispenser-wait actions in `PLAN`, preserving the HOME entry and return. Keep the recipe's `check_trajectories()` wrapper so it checks the same order that runs.
4. Add handlers for new device actions. Replace manual `wait_*_finished()` functions with device startup, completion feedback, and timeout handling when integrating real dispensers.
5. Run offline inspection and commission the complete route, including junction corrections and HOME connections.

Hardware-free tests:

```powershell
python -m unittest discover -s tests -v
```
