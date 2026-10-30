DOBOT Nova 2 - segmented iced Americano workflow

Shared configuration: ../robot_config.json
Player: ../trajectory_player.py

Plan:
  Verify HOME -> MovL to pick_cup start -> pick_cup
  -> MovL CUP_RECEIVE -> wait for cup -> MovL cup_to_ice start -> cup_to_ice
  -> MovL ICE_POS -> wait for ice and water -> MovL ice_to_coffee start -> ice_to_coffee
  -> MovL COFFEE_POS -> wait for espresso (plus 2 seconds) -> MovL coffee_to_window start
  -> coffee_to_window (including release and fixture withdrawal) -> MovL HOME

Shared with Americano where physically compatible: pick_cup and coffee_to_window.
Ice and water are assumed to be dispensed at one receiving position.

From the repository root, record the required segments using:
  python record_trajectory.py NAME --duration 30 --rate 50
Use NAME = pick_cup, cup_to_ice, ice_to_coffee or coffee_to_window.
Teach HOME with python teach_point.py HOME.
Also teach CUP_RECEIVE, ICE_POS and COFFEE_POS with the same command.
Test a precise linear approach with python teach_point.py --move ICE_POS --linear --speed 5.

Inspect all segments in recipe order without connecting:
  python IceAmericano/nova2_ice_coffee_v1_1.py --dry-run
  python IceAmericano/nova2_ice_coffee_v1_1.py --check-trajectories
  The script's check_trajectories(config) function is also available for development.

PHYSICAL commissioning, with a Y prompt before each motion:
  python IceAmericano/nova2_ice_coffee_v1_1.py --check-steps
  Allows unverified takes but retains file, margin, live-state and playback checks.
  Verify each segment, its corrections, and HOME connections with the actual cup/payload.
  Neither commissioning nor offline inspection automatically changes verification flags.

After physical validation:
  Set replay_verified=true in each take.
  Set safe_routes_physically_verified.ice_americano=true in the shared configuration.
  Manually position at HOME, then:
    python IceAmericano/nova2_ice_coffee_v1_1.py --run
  --run --auto skips motion prompts only; cup/ice/water/espresso confirmations remain manual.

Operate dispensers manually; select espresso only, without additional hot water.
See root README.md and trajectories/README.md for shared settings and recording details.
