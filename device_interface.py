#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三台设备的统一调用接口。

示例::

    from device_interface import BaristaDevices

    with BaristaDevices() as devices:
        devices.coffee.espresso(water_ml=40)
        devices.ice.status()
        devices.cups.status()
        devices.cups.drop()

底层协议和状态对象仍分别由 coffee_machine.py、ice_maker.py、
cup_dispenser.py 提供；本模块只负责统一连接生命周期和常用操作名称。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from coffee_machine import (
    BEAN_HOPPER_LEFT,
    CoffeeMachine,
    MachineStatus,
    PRODUCT_TYPES,
    SIDE_LEFT,
)
from cup_dispenser import CupDispenser, DispenserStatus
from ice_maker import IceMaker, IceMakerStatus


@dataclass
class CoffeeDevice:
    """咖啡机的易用封装。"""

    client: CoffeeMachine

    def status(self) -> MachineStatus:
        return self.client.get_machine_status()

    def version(self) -> tuple[int, int, int]:
        return self.client.get_api_version()

    def espresso(self, water_ml: int = 40, **kwargs: Any) -> None:
        self.product("espresso", water_ml=water_ml, **kwargs)

    def coffee(self, water_ml: int = 120, **kwargs: Any) -> None:
        self.product("coffee", water_ml=water_ml, **kwargs)

    def americano(self, water_ml: int = 40, hot_water_ml: int = 120,
                  **kwargs: Any) -> None:
        self.product("americano", water_ml=water_ml,
                     hot_water_ml=hot_water_ml, **kwargs)

    def latte(self, water_ml: int = 40, milk_quantity: int = 100,
              **kwargs: Any) -> None:
        self.product("latte", water_ml=water_ml,
                     milk_quantity=milk_quantity, **kwargs)

    def milkfoam(self, milk_quantity: int = 100,
                 milk_output: str = "arm", **kwargs: Any) -> None:
        self.product("milkfoam", milk_quantity=milk_quantity,
                     milk_output=milk_output, **kwargs)

    def hotwater(self, water_ml: int = 150, **kwargs: Any) -> None:
        self.product("hotwater", water_ml=water_ml, **kwargs)

    def product(self, product: str | int, *, side: int = SIDE_LEFT,
                water_ml: int = 40, bean_hopper: int = BEAN_HOPPER_LEFT,
                **kwargs: Any) -> None:
        if isinstance(product, str):
            try:
                product_type = PRODUCT_TYPES[product]
            except KeyError as exc:
                raise ValueError(f"不支持的咖啡类型: {product}") from exc
        else:
            product_type = product
        self.client.do_product(product_type, side=side, water_ml=water_ml,
                               bean_hopper=bean_hopper, **kwargs)

    def rinse(self, side: int = SIDE_LEFT) -> None:
        self.client.do_rinse(side)

    def stop(self, module: int = 0) -> None:
        self.client.stop(module)

    def clean(self) -> None:
        self.client.start_cleaning()


@dataclass
class IceDevice:
    """制冰机的易用封装。"""

    client: IceMaker

    def status(self) -> IceMakerStatus:
        return self.client.query_status()

    def poll(self) -> tuple[int, int]:
        return self.client.poll()

    def error(self) -> tuple[int, str]:
        return self.client.query_error()

    def ice(self) -> bool:
        return self.client.take_ice()

    def water(self) -> bool:
        return self.client.take_water()

    def ice_water(self) -> bool:
        return self.client.take_ice_water()

    def stop(self) -> bool:
        return self.client.stop()

    def power(self, on: bool) -> bool:
        return self.client.power(on)

    def ice_timed(self, seconds: float) -> bool:
        return self.client.take_ice_timed(seconds)

    def water_timed(self, seconds: float) -> bool:
        return self.client.take_water_timed(seconds)

    def ice_water_timed(self, seconds: float) -> bool:
        return self.client.take_ice_water_timed(seconds)

    def ice_weighed(self, grams: int) -> bool:
        return self.client.take_ice_weighed(grams)

    def water_weighed(self, grams: int) -> bool:
        return self.client.take_water_weighed(grams)

    def ice_water_weighed(self, grams: int) -> bool:
        return self.client.take_ice_water_weighed(grams)

    def ensure_ready(self) -> IceMakerStatus:
        return self.client.ensure_ready()


@dataclass
class CupDevice:
    """落杯机的易用封装。"""

    client: CupDispenser

    def status(self) -> DispenserStatus:
        return self.client.query_status()

    def params(self) -> dict[str, Any]:
        return self.client.query_params()

    def drop(self, drink_id: int = 1, timeout: float = 30.0) -> DispenserStatus:
        return self.client.dispense_and_wait(drink_id=drink_id, timeout=timeout)


class BaristaDevices:
    """统一管理咖啡机、制冰机和落杯机。

    设备在进入 ``with`` 时打开，退出时自动关闭。也可以手动调用
    ``open()`` / ``close()``，适合长期运行的服务程序。
    """

    def __init__(self, *, coffee_port: Optional[str] = None,
                 ice_port: Optional[str] = None,
                 cup_port: Optional[str] = None,
                 timeout: float = 2.0):
        coffee_kwargs = {} if coffee_port is None else {"port": coffee_port}
        ice_kwargs = {} if ice_port is None else {"port": ice_port}
        cup_kwargs = {} if cup_port is None else {"port": cup_port}
        self._options = (coffee_kwargs, ice_kwargs, cup_kwargs)
        self._timeout = timeout
        self.coffee: CoffeeDevice
        self.ice: IceDevice
        self.cups: CupDevice
        self._clients: list[Any] = []
        self._opened = False

    def open(self) -> "BaristaDevices":
        if self._opened:
            return self
        coffee_kwargs, ice_kwargs, cup_kwargs = self._options
        try:
            coffee = CoffeeMachine(timeout=self._timeout, **coffee_kwargs)
            self._clients.append(coffee)
            coffee.open()
            ice = IceMaker(timeout=self._timeout, **ice_kwargs)
            self._clients.append(ice)
            cups = CupDispenser(timeout=self._timeout, **cup_kwargs)
            self._clients.append(cups)
            self.coffee = CoffeeDevice(coffee)
            self.ice = IceDevice(ice)
            self.cups = CupDevice(cups)
        except BaseException:
            self.close()
            raise
        self._opened = True
        return self

    def close(self) -> None:
        clients, self._clients = self._clients, []
        self._opened = False
        try:
            error = None
            for client in reversed(clients):
                try:
                    client.close()
                except Exception as exc:
                    error = exc
            if error is not None:
                raise error
        finally:
            for name in ("coffee", "ice", "cups"):
                if hasattr(self, name):
                    delattr(self, name)

    def __enter__(self) -> "BaristaDevices":
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()


__all__ = ["BaristaDevices", "CoffeeDevice", "IceDevice", "CupDevice"]
