# Robot Barista

Python workflows for a DOBOT Nova 2 coffee workstation. The robot carries a cup between dispensing stations and a serving window, following taught waypoints and waiting for each dispensing step to finish.

All recipes share [nova2_points.json](nova2_points.json) for waypoints and robot settings. Each recipe script defines its own sequence in `PLAN`. Movement uses `MovL`, with `Sync` after each segment. Dispensers currently require manual operation and completion confirmation.

## Project layout

- `nova2_points.json`: shared waypoints, robot IP, Tool/User indices, speed, and route verification flags.
- `teach_point.py`: shared waypoint teaching, including new point names.
- `record_trajectory.py`: passive recording of timed joint and TCP samples for tasks such as latte art.
- `robot_common.py`: shared configuration, connection, command, and robot-enable helpers.
- `trajectories/`: recorded takes, created when a recording is saved.
- `Americano/` and `IceAmericano/`: recipe scripts and notes.
- `TCP_IP_Python_V3/`: [Dobot SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V3), including its license.

## Setup and use

Use Python 3.8 or later and install the SDK dependency:

```powershell
python -m pip install numpy
```

Configure the TCP, payload, Tool/User frames, and compatible TCP/IP control mode in DobotStudio Pro. Set `robot_ip` in the shared JSON to the controller's actual address and check network connectivity. The scripts connect to dashboard port `29999` and movement port `30003`; controller firmware must be compatible with the bundled SDK.

Run commands from the repository root. The following uses one recipe as an example:

```powershell
# Preview the plan without connecting to hardware
python Americano/nova2_coffee_v1_1.py --dry-run

# Manually position the robot, then record a waypoint
python teach_point.py HOME
```

Teaching reads the current joints and tool pose without issuing movement commands. Keep the robot stationary while reading, then type `SAVE` to update the shared JSON. Use `python teach_point.py --list` to list points, or supply a new name such as `LATTE_START` to create one. Existing recipe `--teach POINT` commands also delegate to this shared implementation.

Each waypoint stores `pose = [x, y, z, rx, ry, rz]` in mm/degrees and `joint = [j1, ..., j6]` in degrees. Motion requires a pose for every waypoint used by the recipe; `HOME` also requires joint angles for the startup check. A `null` value means the point has not been taught. Keep Tool/User and TCP settings consistent between teaching and execution.

Before running, teach the required points and physically verify the entire route at low speed. Set `safe_routes_physically_verified[WORKFLOW]` to `true` in the JSON only for the verified recipe; `WORKFLOW` is defined in its script. Teaching any point resets all recipe verification flags. Manual changes to points or the workstation also require resetting the flags and verifying the affected routes again.

Manually place the robot at `HOME`, then run:

```powershell
python Americano/nova2_coffee_v1_1.py --run
```

Execution checks the HOME joints before enabling the robot and asks for `Y` before each movement. Add `--auto` to skip movement confirmations after verification; dispenser confirmations remain manual. Run only one recipe at a time.

To send an enable request separately, using the shared robot settings:

```powershell
python robot_common.py --enable
```

This connects to the dashboard, selects the configured Tool/User, and sends `EnableRobot`. It does not issue movement commands. Recipe scripts call the same `enable_robot(dashboard)` helper after their HOME check. Point teaching and trajectory recording do not automatically enable the robot. The SDK's default enable/payload behavior is unchanged; verify the TCP and payload before use.

## Recording a trajectory

Use the trajectory recorder to capture a complete movement over time rather than a single stationary waypoint:

```powershell
python record_trajectory.py latte_art_001 --duration 30 --rate 50
```

Set up the TCP, payload, and teaching/drag mode using the robot controls first. Ensure the active Tool/User indices match `nova2_points.json` and that feedback is available in that mode. Press Enter in the terminal to connect, wait for `Recording started`, then perform the movement. Recording ends after the requested duration; `Ctrl+C` ends it early and saves the samples already captured. Use a new take name for each attempt.

The recorder only receives port `30004` feedback. It sends no commands to enable, move, or switch the robot into drag mode, and it does not change coordinate frames or shared waypoints. It can observe movement performed through the robot controls without running a recipe.

Output defaults to `trajectories/latte_art_001.json`; use `--output PATH` to choose another file. Existing files are never overwritten. Each sample contains:

- `t`: seconds since the first retained sample, measured at reception on the host.
- `joint` and `pose`: actual joints and TCP pose from the same feedback packet.
- `controller_timestamp_raw`: the original controller timestamp without unit conversion.
- Robot mode, drag status, and error status reported by the controller.

The file also stores frame indices and definitions, units, sample count, average rate, maximum sample gap, and the reason recording ended. Tool/User mismatches, frame changes, invalid packets, and network errors stop capture; an error after capturing data saves a partial take marked with `stop_reason: "error"`. No file is saved if no samples were captured.

`--rate` is an upper bound for retained samples, not a guaranteed control frequency. Inspect the actual timestamps and gaps before processing a take. The pose is the controller's `tool_vector_actual`, without conversion; verify its frame convention against the controller before using it elsewhere.

These files are recording data, not executable motion programs. Trajectory replay, resampling, and latte-art motion control are future integration steps; repeated `MovL`/`Sync` calls do not reproduce the recorded timing or smoothness.

## Adding a recipe or device

1. Copy an existing recipe script into a directory directly under the repository root, preserving the shared config and SDK paths.
2. Choose a unique `WORKFLOW` name and add its entry to `safe_routes_physically_verified` with a value of `false`.
3. Define the action order in `PLAN` and update `POINT_NAMES` to include the waypoints that recipe requires. Reuse existing points where the physical location and tool pose match; add new points to the shared JSON when needed.
4. For new device actions, add a handler in `run()`. Replace or extend the `wait_*_finished()` placeholders with device commands, completion feedback, and timeouts when integrating real dispensers.
5. Preview the plan, teach any new points, and verify every movement segment before enabling the recipe.

Waypoint reuse does not verify new connections between points. `MovL` does not provide full-arm collision avoidance, and cup orientation must be checked along the route. Closing a connection or aborting the script does not guarantee the robot has stopped.

Hardware-free checks for the shared teaching and recording tools:

```powershell
python -m unittest discover -s tests -v
```
