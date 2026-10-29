# Robot Barista

Python workflows for a DOBOT Nova 2 coffee workstation. Each recipe combines shared recorded motion segments with dispenser completion checks. Only the planned HOME entry/return connections and small start corrections use `MovL`.

## Project layout

- `robot_config.json`: shared robot settings, HOME, trajectory file mapping, playback limits, and workflow verification flags.
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

To physically inspect a saved waypoint, move the robot to it:

```powershell
python teach_point.py --move HOME
# Optional lower speed factor:
python teach_point.py --move HOME --speed 5
```

This reads `points.HOME` from `robot_config.json`, displays the saved values, and asks for `Y` before enabling and moving. `JointMovJ` restores the saved six joint angles so you can inspect the recorded arm configuration in physical space. The complete joint-motion path must be clear; it is not a TCP straight-line move or obstacle-avoiding route. Actual joints and TCP arrival are checked using live feedback. The command does not overwrite the saved point and leaves the robot enabled. Use the same command with another saved point name.

## Record shared segments

Recipe scripts already reserve the segment names and their execution order in `PLAN`. The shared JSON already maps these names to files:

| Segment name | Default file | Movement to record |
| --- | --- | --- |
| `pick_cup` | `trajectories/pick_cup.json` | Open approach position to cup receiving position |
| `cup_to_coffee` | `trajectories/cup_to_coffee.json` | Cup receiving position to coffee outlet |
| `cup_to_ice` | `trajectories/cup_to_ice.json` | Cup receiving position to ice/water outlet |
| `ice_to_coffee` | `trajectories/ice_to_coffee.json` | Ice/water outlet to coffee outlet |
| `coffee_to_window` | `trajectories/coffee_to_window.json` | Coffee outlet through cup placement, release, and fixture withdrawal |

Record the names needed by your recipe. Using a reserved name saves to its default file, so the existing recipe can find it without editing its script. Both recipes reference `pick_cup` and `coffee_to_window`; validate shared segments for each recipe's actual cup/payload conditions.

For example, record the pickup segment:

```powershell
python record_trajectory.py pick_cup --duration 30 --rate 50
```

1. Set up teaching/drag mode and matching Tool/User/TCP using the robot controls. Position the arm at the intended segment start.
2. Run the command, press Enter, and wait for `Recording started` before moving.
3. Perform the complete movement, including any withdrawal, corridor, or approach needed. Start and end with the arm at rest.
4. Press `Ctrl+C` once the movement is complete, or let `--duration` end recording. Samples are saved automatically; no `SAVE` input is required for trajectories.
5. Record the next segment from the preceding segment's endpoint, matching both tool pose and joint configuration. Operate the dispenser between recordings; the recipe's callback handles the equipment wait during execution.

Repeat with the other required names, for example:

```powershell
python record_trajectory.py cup_to_ice --duration 30 --rate 50
python record_trajectory.py ice_to_coffee --duration 30 --rate 50
python record_trajectory.py coffee_to_window --duration 30 --rate 50
```

The first approach position must be reachable from HOME by the separately verified entry `MovL`. Finish `coffee_to_window` with the fixture clear of the cup/window so the planned return to HOME is possible.

Recordings default to `trajectories/NAME.json`. Existing files are not overwritten; use another take name and update the corresponding `trajectories` entry in the shared JSON. See [segment recording notes](trajectories/README.md) for the required names and boundaries.

For example, after recording `pick_cup_002`, keep the recipe's logical name and change only its file mapping:

```json
"pick_cup": {"file": "trajectories/pick_cup_002.json"}
```

Recording does not automatically register a new logical segment or mark it as verified. After recording, follow the inspection and commissioning steps below.

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

Other programs can run a recorded segment through the public function in the existing player module:

```python
from trajectory_player import single_trajectory

# PHYSICAL MOTION: validate an unverified recording, align and play it once.
final_state = single_trajectory("pick_cup", commissioning=True)

# Use align_only=True to finish after checking/correcting the start position.
# For an already verified take, leave commissioning at its default False.
```

The same entry point is available from the terminal:

```powershell
python trajectory_player.py pick_cup --commissioning --align-only
python trajectory_player.py pick_cup --commissioning
```

`single_trajectory()` loads the actual mapped recording, checks live state before enabling, confirms motion with the operator, aligns within the shared margins, plays, and checks the endpoint. Position near the recorded start first; larger errors raise an exception. It returns the final feedback state and leaves verification flags unchanged. Enable state is not automatically cleared on exit.

The call chain is `single_trajectory()` -> `check_start()` -> `ensure_start()` -> `play()` -> endpoint check. These are runtime checks used by application code. `test_reject_unsafe_start` and the other `test_*` functions under `tests/` are independent unittest checks of that behavior; they are not called to operate the robot. No separate trial script is needed.

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

## Recipe and device integration hooks

The hooks live in the recipe scripts: [Americano](Americano/nova2_coffee_v1_1.py) and [IceAmericano](IceAmericano/nova2_ice_coffee_v1_1.py). They currently use manual confirmation, so recording the trajectories alone does not start the dispensers automatically.

| Location | Role / extension point |
| --- | --- |
| `PLAN` | Ordered `(action, segment_name, description)` entries; reserves each trajectory's position and each equipment-wait step |
| `run()` | Dispatches actions to trajectory playback or a device callback; add a branch here for a new action type |
| `wait_cup_drop_finished()` | Called after `pick_cup` reaches its endpoint; add cup-dispenser startup and completion/seating feedback here |
| `wait_ice_water_finished()` | Ice recipe only; called after `cup_to_ice`; add ice/water startup and feedback confirming both have finished |
| `wait_coffee_finished()` | Called after arrival at the coffee outlet; select the recipe's drink and wait for completion; retain or deliberately adjust the existing 2-second finishing delay |
| `check_trajectories(config)` | Offline inspection in the same order as `PLAN`; keep this wrapper in each new recipe |

For example, `('ice_signal', None, '...')` in `PLAN` reaches the `ice_signal` branch in `run()`, which calls `wait_ice_water_finished()`. The preceding trajectory must complete and pass its endpoint check before this callback runs. Callback failures abort the workflow before the next segment.

When replacing a manual callback, send the device start command, wait for actual completion with a bounded timeout, and raise an exception on failure. The following shows the integration pattern; `ice_machine` and its methods are placeholders for your device adapter, not bundled APIs:

```python
def wait_ice_water_finished():
    ice_machine.start_ice_water()
    if not ice_machine.wait_until_finished(timeout=30):
        raise RuntimeError("Ice/water dispensing did not finish in time")
```

Put reusable device communication in a shared adapter module and call it from the recipe callbacks. Equipment waits remain outside the recorded motion files.

## Extend the workstation

1. Add recordings to the shared trajectory mapping, reusing a segment only when its frames, start/end arm states, fixture, payload, and cup state are appropriate.
2. Copy a recipe directory under the repository root, select a unique `WORKFLOW`, and add its verification entry with value `false`.
3. Arrange `trajectory` and dispenser-wait actions in `PLAN`, preserving the HOME entry and return. Keep the recipe's `check_trajectories()` wrapper so it checks the same order that runs.
4. Add handlers for new device actions. Replace manual `wait_*_finished()` functions with device startup, completion feedback, and timeout handling when integrating real dispensers.
5. Run offline inspection and commission the complete route, including junction corrections and HOME connections.

`tests/test_trajectory_player.py` contains seven hardware-free checks of program logic: loading, start alignment, unsafe-start rejection, confirmed arrival, single-segment playback, aborts, and workflow preflight. These checks use synthetic data, not your recorded trajectories; use the recipe scripts with `--check-trajectories` and `--check-steps` to inspect and physically validate your recorded files. Separate recording/teaching and enable tests remain in their own files.

Run the focused playback checks or the complete hardware-free suite:

```powershell
python -m unittest discover -s tests -p test_trajectory_player.py -v
python -m unittest discover -s tests -v
```
