DOBOT Nova 2 - segmented Americano workflow

Shared configuration: ../robot_config.json
Player: ../trajectory_player.py

Plan:
  Verify HOME -> MovL to pick_cup start -> pick_cup
  -> wait for cup -> cup_to_coffee -> wait for coffee (plus 2 seconds)
  -> coffee_to_window (including release and fixture withdrawal) -> MovL HOME

From the repository root:
  python teach_point.py HOME
  python record_trajectory.py pick_cup --duration 30 --rate 50
  python record_trajectory.py cup_to_coffee --duration 30 --rate 50
  python record_trajectory.py coffee_to_window --duration 30 --rate 50

Inspect all segments in recipe order without connecting:
  python Americano/nova2_coffee_v1_1.py --dry-run
  python Americano/nova2_coffee_v1_1.py --check-trajectories
  The script's check_trajectories(config) function is also available for development.

PHYSICAL commissioning, with a Y prompt before each motion:
  python Americano/nova2_coffee_v1_1.py --check-steps
  Allows unverified takes but retains file, margin, live-state and playback checks.
  Each segment must begin/end at rest. Verify HOME connections and full-arm clearance.
  Neither commissioning nor offline inspection automatically changes verification flags.

After physical validation:
  Set replay_verified=true in each take.
  Set safe_routes_physically_verified.americano=true in the shared configuration.
  Manually position at HOME, then:
    python Americano/nova2_coffee_v1_1.py --run
  --run --auto skips motion prompts only; cup/coffee confirmations remain manual.

Dispenser operation is currently manual. Reuse takes only with compatible frames,
start/end arm states, fixture, cup and payload conditions. See root README.md and
trajectories/README.md for shared margins, timing, recording and extension details.
