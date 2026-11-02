#!/usr/bin/env python3
"""点位美式/拿铁流程；默认离线预览，只有 --execute 才连接设备并运动。"""

import argparse
import math
import sys
import time
from pathlib import Path

import coffee_machine as coffee
import cup_dispenser as cups
from point_path import load_point_path, play_point_path
from robot_common import CONFIG_FILE, read_config
from trajectory_player import limits_for, state_errors, within

PATHS = ("pick_cup", "cup2coffee", "coffee2window")


def remaining(deadline):
    value = deadline - time.monotonic()
    if value <= 0:
        raise TimeoutError("设备阶段等待超时；中止，不执行下一步")
    return value


class SingleShotDispenser(cups.CupDispenser):
    """保留现有协议解析，但禁止默认三次重发，并限制串口读写时间。"""

    deadline = 0.0

    def _transact(self, cmd, ins, payload=b"", retries=1):
        budget = min(1.0, remaining(self.deadline) / 3)
        self.ser.timeout = budget
        self.ser.write_timeout = budget
        result = super()._transact(cmd, ins, payload, retries=1)
        remaining(self.deadline)
        return result


class BoundedCoffee(coffee.CoffeeMachine):
    deadline = 0.0

    def transact(self, mi, mp=0, data=b"", wait_ack_timeout=3.0, wait_resp_timeout=10.0):
        budget = remaining(self.deadline) / 4
        self.ser.write_timeout = min(1.0, budget)
        result = super().transact(mi, mp, data,
                                  wait_ack_timeout=min(wait_ack_timeout, budget),
                                  wait_resp_timeout=min(wait_resp_timeout, budget))
        remaining(self.deadline)
        return result


def wait_cup(dev, timeout):
    dev.deadline = time.monotonic() + timeout
    if not dev.query_status().ready:
        raise RuntimeError("落杯机未就绪")
    if not dev.dispense(drink_id=1):
        raise RuntimeError("落杯命令被拒绝")
    # dispense() 的设置成功不是完成；与现有 dispense_and_wait 一致，
    # 完成依据是后续状态查询 work_state=0，而不是杜撰独立完成帧。
    while True:
        remaining(dev.deadline)
        status = dev.query_status()
        remaining(dev.deadline)
        if status.work_state == 2 or status.motor_fault or status.drawer_open or status.cup_missing:
            raise RuntimeError(f"落杯机异常: {status}")
        if status.work_state == 0:
            return
        if status.work_state not in (1, 3):
            raise RuntimeError(f"未知落杯状态: {status}")
        time.sleep(min(0.3, remaining(dev.deadline)))


def drink_recipe(drink, milk_qty):
    if drink == "americano":
        return 5, dict(water_ml=80, hot_water_ml=180)
    if drink == "latte":
        return 15, dict(water_ml=40, milk_quantity=milk_qty, milk_temp=0,
                        milk_percent=100, milk_sequence=1, foam_sequence=0,
                        milk_output="coffee")
    raise ValueError("drink 必须为 americano 或 latte")


def make_coffee(dev, timeout, drink="americano", milk_qty=100):
    dev.deadline = time.monotonic() + timeout
    product_type, kwargs = drink_recipe(drink, milk_qty)
    dev.do_product(product_type, **kwargs)
    ready_status = 1 if drink == "latte" else 2
    observed_busy = False  # 必须是本次请求之后采样的状态。
    while True:
        remaining(dev.deadline)
        status = dev.get_machine_status().module_status(0)
        remaining(dev.deadline)
        if status == 0:
            observed_busy = True
        elif status == ready_status and observed_busy:
            return
        elif status not in (1, 2, 3):
            raise RuntimeError(f"MP0 异常状态: {status}")
        time.sleep(min(1.0, remaining(dev.deadline)))


def preflight(config, speed, radius, cup_timeout, coffee_timeout,
              drink="americano", milk_qty=100):
    if type(speed) is not int or not 1 <= speed <= 100:
        raise ValueError("speed 必须为 1..100 的整数")
    if type(radius) is not int or radius <= 0:
        raise ValueError("radius 必须为正整数 mm")
    for value in (cup_timeout, coffee_timeout):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("超时必须为有限正数")
    product_type, kwargs = drink_recipe(drink, milk_qty)
    recipe = coffee.product_data(product_type, **kwargs)
    paths = [load_point_path(name, config) for name in PATHS]
    limits = limits_for(config)
    for previous, current in zip(paths, paths[1:]):
        if previous["feedback_frames"] != current["feedback_frames"]:
            raise ValueError("路径 Tool/User 定义不一致")
        if not within(state_errors(previous["points"][-1], current["points"][0]), limits):
            raise ValueError("相邻路径首尾超出 point_path 起点容差；请先修正点位")
    return recipe


def run(config, *, execute=False, speed=None, continuous=False, radius=5,
        cup_timeout=30.0, coffee_timeout=180.0,
        cup_port=cups.DEFAULT_PORT, coffee_port=coffee.DEFAULT_PORT,
        drink="americano", milk_qty=100):
    speed = config.get("speed_factor", 10) if speed is None else speed
    recipe = preflight(config, speed, radius, cup_timeout, coffee_timeout, drink, milk_qty)
    ready_status = 1 if drink == "latte" else 2
    print(f"pick_cup → 落杯并查询待机 → cup2coffee → {drink} ({len(recipe)} bytes) "
          f"→ MP0 0 后 {ready_status} → 等待2秒 → coffee2window")
    if drink == "latte":
        print(f"拿铁：咖啡水40ml，奶时间{milk_qty / 10:g}秒；总量260ml需实测标定。")
    print(f"speed={speed}%, continuous={continuous}, radius={radius}mm")
    if not execute:
        print("离线校验通过；未连接设备。--execute 会执行真实运动及出品。")
        return

    def play(name, *, speed=speed):
        play_point_path(name, config, speed=speed, continuous=continuous, radius=radius)

    # 所有文件和配方校验完成后才允许打开设备。任何异常直接退出，不重试。
    with SingleShotDispenser(port=cup_port) as dispenser, BoundedCoffee(port=coffee_port) as machine:
        play(PATHS[0],speed=40)
        wait_cup(dispenser, cup_timeout)
        time.sleep(1.0)
        play(PATHS[1],speed=40)
        make_coffee(machine, coffee_timeout, drink, milk_qty)
        time.sleep(2.0)
        play(PATHS[2],speed=20)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drink", choices=("americano", "latte"), default="americano")
    parser.add_argument("--milk-qty", type=int, default=100,
                        help="拿铁出奶时间，单位0.1秒；默认100=10秒，非毫升")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--speed", type=int)
    parser.add_argument("--continuous", action="store_true")
    parser.add_argument("--radius", type=int, default=5)
    parser.add_argument("--cup-timeout", type=float, default=30.0)
    parser.add_argument("--coffee-timeout", type=float, default=180.0)
    parser.add_argument("--cup-port", default=cups.DEFAULT_PORT)
    parser.add_argument("--coffee-port", default=coffee.DEFAULT_PORT)
    args = vars(parser.parse_args(argv))
    config = read_config(args.pop("config"))
    run(config, **args)


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f"流程中止: {exc}。不重试、不继续；请人工核实设备和机械臂状态。", file=sys.stderr)
        sys.exit(1)