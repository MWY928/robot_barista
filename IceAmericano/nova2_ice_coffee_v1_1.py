#!/usr/bin/env python3
"""用途：执行冰美式流程：接杯 → 冰水 → 浓缩咖啡 → 出杯 → HOME。
主要入口：PLAN 定义顺序；check_trajectories()/dry_run() 离线检查/预览；
          run() 使能并执行流程；wait_*_finished() 目前人工确认设备完成。
根目录运行：
  python IceAmericano/nova2_ice_coffee_v1_1.py --dry-run            # 离线预览
  python IceAmericano/nova2_ice_coffee_v1_1.py --check-trajectories # 离线严格校验
  python IceAmericano/nova2_ice_coffee_v1_1.py --check-steps        # 会运动：逐步试跑，允许未验证轨迹
  python IceAmericano/nova2_ice_coffee_v1_1.py --run                # 会运动：执行已验证流程
"""

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_common import connect, enable_robot, read_config, send_command
from teach_point import teach_point
from recipe_common import inspect_workflow, operator_confirm, recipe_arguments
from trajectory_player import LiveFeedback, TrajectoryPlayer

WORKFLOW = 'ice_americano'
POINT_NAMES = ("HOME",)

# Segments stop at station waiting points. Files are shared in robot_config.json.
PLAN = [
    ('check_home', 'HOME', 'Verify the starting HOME pose and joints'),
    ('entry', 'pick_cup', 'MovL from HOME to the first pick-cup sample'),
    ('trajectory', 'pick_cup', 'Follow the shared cup-pickup trajectory'),
    ('cup_signal', None, 'Wait for cup dispensing to finish'),
    ('trajectory', 'cup_to_ice', 'Carry the cup to the ice/water outlet'),
    ('ice_signal', None, 'Wait for ice and water dispensing to finish'),
    ('trajectory', 'ice_to_coffee', 'Carry the cup from ice machine to coffee machine'),
    ('coffee_signal', None, 'Wait for coffee dispensing to finish, then wait 2 seconds'),
    ('trajectory', 'coffee_to_window', 'Place and release the cup, then withdraw the fixture'),
    ('return_home', 'HOME', 'MovL from the cleared window position to HOME'),
]


def wait_cup_drop_finished():
    """TODO: Replace manual confirmation with real feedback and a timeout."""
    operator_confirm("[Manual feedback] Confirm the cup is seated correctly in the fixture.")


def wait_ice_water_finished():
    """TODO: Replace manual confirmation with real feedback and a timeout."""
    operator_confirm(
        "[Manual feedback] Confirm ice and water dispensing have both finished, "
        "the outlet has stopped, and the cup is seated correctly."
    )


def wait_coffee_finished():
    """TODO: Replace manual confirmation with real feedback and a timeout."""
    operator_confirm("[Manual feedback] Confirm espresso dispensing has finished (espresso only, no extra hot water).")
    time.sleep(2.0)


def check_trajectories(config, require_verified=False):
    """Check each segment and junction in this recipe's actual execution order."""
    return inspect_workflow(config, PLAN, WORKFLOW, require_verified=require_verified)


def dry_run(config):
    print("Offline preview: no robot or dispenser connections.")
    return inspect_workflow(config, PLAN, WORKFLOW, strict=False)


def run(config, auto=False, commissioning=False):
    # Every file/junction is checked before connecting, enabling or moving.
    trajectories = check_trajectories(config, require_verified=not commissioning)
    home = config["points"]["HOME"]
    if commissioning:
        auto = False
        operator_confirm("STEP CHECK MODE: the robot WILL MOVE. Check each segment and HOME connection at low speed.")
    with connect(config, for_movement=True) as (dashboard, move):
        with LiveFeedback(config) as feedback:
            player = TrajectoryPlayer(dashboard, move, config, feedback)
            player.use_frames(trajectories["pick_cup"])
            player.check_home(home)
            send_command(dashboard.SpeedFactor, config.get("speed_factor", 10))
            enable_robot(dashboard)
            # Wait briefly for enabled state to appear in fresh feedback.
            deadline = time.monotonic()+player.limits["endpoint_timeout_s"]
            while not feedback.state()["enable_status"]:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Enable request succeeded but enabled feedback did not arrive.")
                time.sleep(0.02)
            try:
                for index, (action, name, note) in enumerate(PLAN, 1):
                    print(f"\n[{index}/{len(PLAN)}] {note}")
                    if action == "check_home":
                        player.check_home(home)
                    elif action == "cup_signal":
                        wait_cup_drop_finished()
                    elif action == "coffee_signal":
                        wait_coffee_finished()
                    elif action == "ice_signal":
                        wait_ice_water_finished()
                    elif action in ("entry", "trajectory", "return_home"):
                        if not auto:
                            operator_confirm(f"Ready: {note}. Confirm the complete arm/cup path is clear.")
                        if action == "entry":
                            # A planned, verified connection, independent of small-error start margins.
                            player.check_home(home)
                            player.move_linear(trajectories[name].first)
                        elif action == "trajectory":
                            player.play(trajectories[name])
                        else:
                            player.assert_target(trajectories["coffee_to_window"].last)
                            player.move_linear(home)
                    else:
                        raise RuntimeError(f"Unknown workflow action: {action}")
            except (Exception, KeyboardInterrupt):
                player.stop_after_error()
                raise
    print("Workflow completed." if not commissioning else
          "Step check completed. Review the results before marking takes and workflow as verified.")


def main():
    args = recipe_arguments(__doc__)
    config = read_config()
    if args.dry_run:
        dry_run(config)
    elif args.check_trajectories:
        check_trajectories(config)
    elif args.teach:
        teach_point(args.teach)
    else:
        run(config, auto=args.auto, commissioning=args.check_steps)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"Task aborted: {exc}", file=sys.stderr)
        print("Check the actual robot state; stopping is not guaranteed by disconnecting.", file=sys.stderr)
        sys.exit(1)
