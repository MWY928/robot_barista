#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Eversys Cameo 单物理咖啡出口 RCC/串口协议控制库
依据: Eversys-API V05.00.00 (Eversys-API-1.pdf)

协议要点
--------
- 接口: RS232, 波特率 115200, 8N1
- 帧结构:
    SOH(0x01) | PIP PIE PN SA DA | MI MP(lo hi) DL(lo hi) | Data... | CRC(lo hi) | EOT(0x04)
  - PIP = 0x00 (未用)
    - PIE = 0x68 Command/Response / 0x6C Request(查询) / 0x6A ACK / 0x6B NACK
    - ACK/NACK 仅含五字节包头，无数据头及 CRC。
  - PN  = 报文序号(0..255 循环), ACK/NACK 中的 PN 与被确认的报文一致
  - SA  = 0x42 (上位机/Remote), DA = 0x41 (咖啡机 e'4x)
    - MI  = 命令号, MP = 报文参数(逻辑模块 0=left 1=right，不代表物理双嘴), DL = 数据字节数(小端)
- CRC16: MODBUS (poly 0xA001, init 0xFFFF, 反射), 小端存放;
         对"转义前"的 PIP..Data 全部字节计算 (已用文档 GetStatus 示例帧验证)
- 字节转义 (stuffing & shifting): 以下字节在帧内须替换为 DLE(0x10) + (byte ^ 0x40):
    0x00 0x01 0x02 0x03 0x04 0x0A 0x0D 0x10 0x17
  仅 SOH / EOT 不转义。
- 查询流程: 上位机发 Request -> 机器回 ACK -> 机器发 Response -> 上位机回 ACK
- 动作流程: 上位机发 Command -> 机器回 ACK/NACK；ACK 仅确认收包，不表示动作完成。

常用命令号 (MI)
---------------
  0  Get API Version
  1  Get Machine Status        (文档建议 1 秒轮询)
    2  Do Product                (默认逻辑 MP0，可覆盖为 MP1)
    3  Do Rinse                  (默认逻辑 MP0，可覆盖为 MP1)
  4  Start Cleaning
  11 Stop                      (MP: 0=全部, 1..7=指定模块)
  12 Get Requests

用法
----
  python3 coffee_machine.py version
  python3 coffee_machine.py status
    python3 coffee_machine.py espresso            # 默认 MP0，示例参数，执行会出品
    python3 coffee_machine.py rinse --side left   # MP0，执行会冲洗
  python3 coffee_machine.py stop                # 停止全部
  python3 coffee_machine.py clean
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import os
import struct
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import serial

# 使用已验证的咖啡机硬件标识，避免升级/插拔后 ttyUSB 编号变化。
# 更换适配器时通过 --port 显式指定；不回退到其他设备。
DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Prolific_Technology_Inc._USB-Serial_Controller_A8CLb2AC569-if00-port0"
)

# ---------------- 协议常量 ----------------

SOH = 0x01
EOT = 0x04
DLE = 0x10
SHIFT_XOR = 0x40

# 需要转义的特殊字符
SPECIAL_CHARS = frozenset((0x00, 0x01, 0x02, 0x03, 0x04, 0x0A, 0x0D, 0x10, 0x17))

# PIE 报文类型
PIE_COMMAND = 0x68
PIE_REQUEST = 0x6C
PIE_ACK = 0x6A
PIE_NACK = 0x6B

ADDR_REMOTE = 0x42   # 上位机
ADDR_MACHINE = 0x41  # 咖啡机

# 命令号 MI
CMD_GET_API_VERSION = 0
CMD_GET_STATUS = 1
CMD_DO_PRODUCT = 2
CMD_DO_RINSE = 3
CMD_START_CLEANING = 4
CMD_STOP = 11
CMD_GET_REQUESTS = 12

REQUEST_COMMANDS = frozenset((CMD_GET_API_VERSION, CMD_GET_STATUS, CMD_GET_REQUESTS))

# 协议逻辑模块名，兼容 left/right；不是物理双嘴。
SIDE_LEFT = 0
SIDE_RIGHT = 1

# 产品类型 ProductType_t (常用, 详见文档 6.3.14)
PRODUCT_TYPES = {
    "ristretto": 1,
    "espresso": 2,
    "coffee": 3,
    "americano": 5,
    "milkfoam": 17,
    "milk-foam": 17,
    "cappuccino": 13,
    "latte": 15,  # LatteMacchiato，不是通用拿铁配方
    "hotwater": 8,
}

# 豆仓 BeanHopper_t
BEAN_HOPPER_LEFT = 1
BEAN_HOPPER_RIGHT = 0
BEAN_HOPPER_MIX = 2


class CoffeeMachineError(Exception):
    pass


class CoffeeMachineTimeout(CoffeeMachineError):
    pass


class CoffeeMachineNack(CoffeeMachineError):
    pass


# ---------------- 基础编解码 ----------------

def crc16_modbus(data: bytes) -> int:
    """MODBUS CRC16 (poly 0xA001, init 0xFFFF), 返回 16 位整数。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def stuff(data: bytes) -> bytes:
    """DLE 转义: 特殊字符 -> DLE + (b ^ 0x40)"""
    out = bytearray()
    for b in data:
        if b in SPECIAL_CHARS:
            out.append(DLE)
            out.append(b ^ SHIFT_XOR)
        else:
            out.append(b)
    return bytes(out)


def unstuff(data: bytes) -> bytes:
    """DLE 反转义。"""
    out = bytearray()
    i = 0
    while i < len(data):
        b = data[i]
        if b == DLE:
            if i + 1 >= len(data):
                raise CoffeeMachineError("转义序列不完整: 末尾出现孤立 DLE")
            out.append(data[i + 1] ^ SHIFT_XOR)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


def build_frame(pn: int, mi: int, mp: int = 0, data: bytes = b"") -> bytes:
    """构造上位机帧，查询使用 Request，其余使用 Command。"""
    body = bytes((
        0x00,            # PIP
        PIE_REQUEST if mi in REQUEST_COMMANDS else PIE_COMMAND,
        pn & 0xFF,       # PN
        ADDR_REMOTE,     # SA
        ADDR_MACHINE,    # DA
        mi, mp & 0xFF, (mp >> 8) & 0xFF,  # MI, MP (16-bit)
        len(data) & 0xFF, (len(data) >> 8) & 0xFF,  # DL 小端
    )) + data
    crc = crc16_modbus(body)
    raw = body + bytes((crc & 0xFF, crc >> 8))
    return bytes((SOH,)) + stuff(raw) + bytes((EOT,))


@dataclass
class Frame:
    pie: int
    pn: int
    sa: int
    da: int
    mi: int
    mp: int
    data: bytes


def parse_frame(body: bytes) -> Frame:
    """解析已反转义的帧体 (SOH/EOT/CRC 已剥离并校验)。

    ACK/NACK 帧只有 5 字节包头 (PIP PIE PN SA DA)，无 MI/MP/DL/Data。
    """
    if len(body) < 5:
        raise CoffeeMachineError(f"帧过短: {len(body)} 字节")
    if body[1] in (PIE_ACK, PIE_NACK):
        if len(body) != 5:
            raise CoffeeMachineError("ACK/NACK 长度异常")
        return Frame(pie=body[1], pn=body[2], sa=body[3], da=body[4],
                     mi=-1, mp=0, data=b"")
    if len(body) < 10:
        raise CoffeeMachineError("数据帧头不完整")
    if len(body) != 10 + int.from_bytes(body[8:10], "little"):
        raise CoffeeMachineError("数据帧长度与 DL 不符")
    return Frame(
        pie=body[1], pn=body[2], sa=body[3], da=body[4],
        mi=body[5], mp=int.from_bytes(body[6:8], "little"),
        data=body[10:],
    )


# ---------------- 机器状态 ----------------

@dataclass
class MachineStatus:
    """Get Machine Status 响应的简化解析 (完整字段见文档 5.2.2)。"""
    machine_status: int          # 字节0: MachineStatus_t 标志，不是就绪状态
    coffee_left: int
    coffee_right: int
    steam_left: int
    steam_right: int
    water: int
    action_status: int           # 兼容旧字段：实际是字节6 coffee_left process
    raw: bytes

    @property
    def ready(self) -> bool:
        """兼容字段：默认咖啡逻辑 MP0 完全 Ready，不代表整机就绪。"""
        return self.module_ready(SIDE_LEFT)

    def module_status(self, module: int = SIDE_LEFT) -> int:
        """模块索引 0/1=咖啡、2/3=蒸汽、4=热水；低4位状态。"""
        if module not in range(5):
            raise ValueError("模块索引必须为 0..4")
        return (self.coffee_left, self.coffee_right, self.steam_left,
                self.steam_right, self.water)[module] & 0x0F

    def module_action(self, module: int = SIDE_LEFT) -> int:
        self.module_status(module)  # 验证索引
        return (self.coffee_left, self.coffee_right, self.steam_left,
                self.steam_right, self.water)[module] >> 4

    def module_ready(self, module: int = SIDE_LEFT) -> bool:
        return self.module_status(module) == 1

    @property
    def just_reset(self) -> bool:
        return bool(self.machine_status & 0x01)

    @property
    def has_info_messages(self) -> bool:
        return bool(self.machine_status & 0x04)

    @property
    def has_product_dump(self) -> bool:
        return bool(self.machine_status & 0x10)

    @property
    def edit_mode(self) -> bool:
        return bool(self.machine_status & 0x20)

    @property
    def steam_boiler_water_level(self) -> bool:
        return bool(self.machine_status & 0x40)

    @property
    def standby(self) -> bool:
        return bool(self.machine_status & 0x80)

    @property
    def coffee_left_process(self) -> int:
        return self.action_status

    @property
    def has_requests(self) -> bool:
        return bool(self.machine_status & 0x02)


# ---------------- 主类 ----------------

def product_data(product_type: int, water_ml: int = 40,
                 bean_hopper: int = 1, cake_mm10: int = 80,
                 tamping: int = 92, pre_infusion: int = 10, relax: int = 10,
                 milk_quantity: int = 0, milk_temp: int = 0,
                 milk_percent: int = 0, *, milk_sequence=None,
                 foam_sequence: int = 0, hot_water_ml=None, bypass=None,
                 americano_temp=None, americano_sequence=None,
                 milk_output: Optional[str] = None) -> bytes:
    """纯离线编码。扩展默认值仅为示例，不是厂商推荐配方。

    Milk Quantity 单位 0.1s；Milk Percent 是奶比例而非泡沫百分比。
    原文没有明确规定其与泡沫百分比的换算，不能保证成品体积比例。
    milk_output: None 保留旧长度；coffee/arm 显式编码38字节的 P28=0/1。
    arm 仅支持含奶产品且奶时间非零，不改变逻辑 MP。
    """
    def integer(name, value, low, high, choices=None):
        if (type(value) is not int or not low <= value <= high
                or (choices is not None and value not in choices)):
            raise ValueError(f"{name} 必须为整数 {choices if choices else f'{low}..{high}'}")

    integer('product_type', product_type, 1, 17,
            (1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14, 15, 16, 17))
    for name, value, low, high in (
            ('water_ml', water_ml, 10, 1000), ('bean_hopper', bean_hopper, 0, 2),
            ('cake_mm10', cake_mm10, 30, 320), ('pre_infusion', pre_infusion, 0, 100),
            ('relax', relax, 0, 100), ('milk_quantity', milk_quantity, 0, 3000),
            ('milk_temp', milk_temp, 0, 3), ('milk_percent', milk_percent, 0, 100),
            ('foam_sequence', foam_sequence, 0, 1)):
        integer(name, value, low, high)
    integer('tamping', tamping, 64, 120, (64, 92, 120))
    if milk_sequence is None:
        milk_sequence = 3 if product_type in (16, 17) else 0
    integer('milk_sequence', milk_sequence, 0, 4)
    if product_type in (16, 17) and (milk_quantity == 0 or milk_sequence != 3):
        raise ValueError('单独奶/奶泡需要 milk_quantity=1..3000 (0.1s) 和 milk_sequence=3')
    if milk_output is not None and milk_output not in ('coffee', 'arm'):
        raise ValueError('milk_output 必须为 None、coffee 或 arm')
    if milk_output == 'arm' and (product_type not in (12, 13, 14, 15, 16, 17)
                                 or milk_quantity == 0):
        raise ValueError('milk_output=arm 仅适用于含奶产品12..17且 milk_quantity=1..3000 (0.1s)')
    extensions = (hot_water_ml, bypass, americano_temp, americano_sequence)
    if product_type != 5 and any(v is not None for v in extensions):
        raise ValueError('补水/bypass/混合温度/美式顺序仅适用于 americano=5')
    # 旧位置参数保留：非奶产品的旧奶字段仍按原格式编码，由设备忽略。
    if product_type == 5 and (milk_quantity or milk_temp or milk_percent or milk_sequence or foam_sequence):
        raise ValueError('americano 不接受奶参数；不自动拆分动作')
    data = struct.pack('<BBHBHBBBBHBBBBB', product_type,
                       2 if product_type == 8 else 0, water_ml, bean_hopper,
                       cake_mm10, tamping, pre_infusion, relax, 0,
                       milk_quantity, milk_temp, milk_percent, milk_sequence, 0, foam_sequence)
    if product_type == 5 or milk_output is not None:
        hot_water_ml = 120 if hot_water_ml is None else hot_water_ml
        bypass = 0 if bypass is None else bypass
        americano_temp = 70 if americano_temp is None else americano_temp
        americano_sequence = 1 if americano_sequence is None else americano_sequence
        for name, value, low, high in (
                ('hot_water_ml', hot_water_ml, 10, 1000), ('bypass', bypass, 0, 100),
                ('americano_temp', americano_temp, 30, 80),
                ('americano_sequence', americano_sequence, 0, 1)):
            integer(name, value, low, high)
        # P16..29: H B B B H H H B H H B B B B.
        # P20 原表写 unsigned byte (16 bit)，按16位及35/38总长解析。
        data += struct.pack('<HBBBHHHBHHBBBB', 1, americano_temp, 0, 0,
                            10, 100, 100, 0, 20, hot_water_ml, bypass,
                            0, int(milk_output == 'arm'), americano_sequence)
    return data

class CoffeeMachine:
    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = 115200,
                 timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._pn = 0
        self._pn_path: Optional[Path] = None
        self._rx_buf = bytearray()  # 接收缓冲 (一次 read 可能含多帧)

    # -- 连接 --
    def open(self):
        if self.ser and self.ser.is_open:
            return self
        self.ser = serial.Serial(self.port, self.baudrate,
                                 bytesize=8, parity="N", stopbits=1,
                                 timeout=self.timeout, exclusive=True)
        try:
            # realpath 将 /dev/serial/by-id 等别名映射至同一串口。
            identity = os.path.realpath(self.port)
            key = hashlib.sha256(os.fsencode(identity)).hexdigest()
            root = os.environ.get("XDG_STATE_HOME")
            base = Path(root) if root and os.path.isabs(root) else Path.home() / ".local/state"
            directory = base / "coffee_robot"
            directory.mkdir(parents=True, exist_ok=True, mode=0o700)
            self._pn_path = directory / (key + ".pn")
            try:
                text = self._pn_path.read_text(encoding="ascii").strip()
            except FileNotFoundError:
                self._pn = 0  # 仅首次使用允许从 0 开始。
            else:
                if not text.isascii() or not text.isdecimal() or not 0 <= int(text) <= 255:
                    raise ValueError("下一 PN 必须为 0..255 的整数")
                self._pn = int(text)
            self._rx_buf.clear()
        except (OSError, ValueError) as exc:
            self.close()
            raise CoffeeMachineError(f"无法读取 PN 状态 {self._pn_path}: {exc}") from exc
        return self

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def _next_pn(self) -> int:
        pn = self._pn
        next_pn = (pn + 1) & 0xFF
        if self._pn_path is not None:
            temporary = None
            try:
                # 先持久化预留号，再发送；发送失败/超时也不能复用该号。
                with tempfile.NamedTemporaryFile(mode="w", encoding="ascii",
                                                 dir=self._pn_path.parent,
                                                 delete=False) as output:
                    temporary = output.name
                    output.write(f"{next_pn}\n")
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(temporary, self._pn_path)
                temporary = None
                fd = os.open(self._pn_path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            except OSError as exc:
                # replace 后的 fsync 也可能失败，必须重新打开并读取状态。
                self.close()
                raise CoffeeMachineError(f"无法保存 PN 状态 {self._pn_path}: {exc}") from exc
            finally:
                if temporary is not None:
                    try:
                        os.unlink(temporary)
                    except OSError:
                        pass
        self._pn = next_pn
        return pn

    # -- 底层收发 --
    def _extract_frame(self) -> Optional[Frame]:
        """尝试从接收缓冲区提取一帧。成功则消费并返回 Frame，否则返回 None。"""
        try:
            start = self._rx_buf.index(SOH)
        except ValueError:
            self._rx_buf.clear()  # 没有 SOH，全是噪声
            return None
        if start > 0:
            del self._rx_buf[:start]  # 丢弃 SOH 前的噪声
        try:
            end = self._rx_buf.index(EOT, 1)
        except ValueError:
            return None  # 帧未收完，等待更多数据
        raw = unstuff(bytes(self._rx_buf[1:end]))
        del self._rx_buf[:end + 1]
        if len(raw) >= 2 and raw[1] in (PIE_ACK, PIE_NACK):
            return parse_frame(raw)
        if len(raw) < 3:
            raise CoffeeMachineError("帧内容过短")
        body, crc_bytes = raw[:-2], raw[-2:]
        expect = crc16_modbus(body)
        got = crc_bytes[0] | (crc_bytes[1] << 8)
        if expect != got:
            raise CoffeeMachineError(
                f"CRC 校验失败: 期望 {expect:04X}, 实际 {got:04X}")
        return parse_frame(body)

    def _read_one_frame(self, deadline: float) -> Optional[Frame]:
        """在 deadline 前读一帧（先查缓冲区，不足再读串口）。"""
        while time.monotonic() < deadline:
            f = self._extract_frame()
            if f is not None:
                return f
            old_timeout = self.ser.timeout
            try:
                self.ser.timeout = max(0, min(0.05, deadline - time.monotonic()))
                chunk = self.ser.read(self.ser.in_waiting or 1)
            finally:
                self.ser.timeout = old_timeout
            if chunk:
                self._rx_buf.extend(chunk)
        return self._extract_frame()

    def _send_ack(self, pn: int):
        ack = bytes((0x00, PIE_ACK, pn, ADDR_REMOTE, ADDR_MACHINE))
        frame = bytes((SOH,)) + stuff(ack) + bytes((EOT,))
        self.ser.write(frame)

    def transact(self, mi: int, mp: int = 0, data: bytes = b"",
                 wait_ack_timeout: float = 3.0, wait_resp_timeout: float = 10.0) -> Frame:
        """
        Command 返回匹配的 ACK 帧，仅确认收包，不代表执行成功或动作完成。
        Request 等 ACK 后继续等待 Response，并回 ACK，返回 Response 帧。
        NACK 或超时抛出异常，不自动重发；无 ACK 时动作是否执行未知。
        """
        if not self.ser or not self.ser.is_open:
            raise CoffeeMachineError("串口未打开")
        pn = self._next_pn()
        self.ser.reset_input_buffer()
        self._rx_buf.clear()
        self.ser.write(build_frame(pn, mi, mp, data))

        # 1) 等 ACK
        ack_received = False
        ack_deadline = time.monotonic() + wait_ack_timeout
        while time.monotonic() < ack_deadline:
            f = self._read_one_frame(ack_deadline)
            if f is None:
                break
            if (f.sa, f.da) != (ADDR_MACHINE, ADDR_REMOTE):
                continue
            if f.pie == PIE_ACK and f.pn == pn:
                if mi not in REQUEST_COMMANDS:
                    return f
                ack_received = True
                break
            if f.pie == PIE_NACK and f.pn == pn:
                raise CoffeeMachineNack(f"机器返回 NACK (MI={mi})")
        if not ack_received:
            raise CoffeeMachineTimeout(f"等待 ACK 超时 (MI={mi})")

        # 2) 等响应帧 (PIE=Response 且 MI 匹配)
        resp_deadline = time.monotonic() + wait_resp_timeout
        while time.monotonic() < resp_deadline:
            f = self._read_one_frame(resp_deadline)
            if f is None:
                break
            if (f.sa, f.da) != (ADDR_MACHINE, ADDR_REMOTE):
                continue
            if f.pie == PIE_COMMAND and f.mi == mi:
                self._send_ack(f.pn)
                return f
        raise CoffeeMachineTimeout(f"等待响应超时 (MI={mi})")

    # -- 高层命令 --
    def get_api_version(self) -> tuple[int, int, int]:
        """返回 (major, minor, revision), 如 (1, 2, 0) = V01.02.00"""
        f = self.transact(CMD_GET_API_VERSION)
        if len(f.data) < 3:
            raise CoffeeMachineError(f"API 版本响应长度异常: {len(f.data)}")
        return f.data[0], f.data[1], f.data[2]

    def get_machine_status(self) -> MachineStatus:
        f = self.transact(CMD_GET_STATUS)
        d = f.data
        if len(d) < 7:
            raise CoffeeMachineError(f"状态响应长度异常: {len(d)}")
        return MachineStatus(
            machine_status=d[0],
            coffee_left=d[1], coffee_right=d[2],
            steam_left=d[3], steam_right=d[4],
            water=d[5], action_status=d[6],
            raw=d,
        )

    def do_rinse(self, side: int = SIDE_LEFT):
        """请求冲洗，ACK 非完成。side: 0/1 为逻辑模块，不代表物理双嘴。"""
        self.transact(CMD_DO_RINSE, mp=side)

    def stop(self, module: int = 0):
        """请求停止，ACK 非完成。module: 0=全部, 1=左咖啡 2=右咖啡 3=左蒸汽 4=右蒸汽 5=热水 6=左奶 7=右奶"""
        self.transact(CMD_STOP, mp=module)

    def start_cleaning(self):
        """请求启动清洗，ACK 非完成（可能需要人工步骤，见文档 5.6）。"""
        self.transact(CMD_START_CLEANING)

    def do_product(self, product_type: int, side: int = SIDE_LEFT,
                   water_ml: int = 40, bean_hopper: int = BEAN_HOPPER_LEFT,
                   cake_mm10: int = 80, tamping: int = 92,
                   pre_infusion: int = 10, relax: int = 10,
                   milk_quantity: int = 0, milk_temp: int = 0,
                   milk_percent: int = 0, *, milk_sequence=None,
                   foam_sequence: int = 0, hot_water_ml=None, bypass=None,
                   americano_temp=None, americano_sequence=None,
                   milk_output: Optional[str] = None) -> None:
        """
        请求出品：美式或显式 milk_output 使用38字节，其余保留18字节。
        milk_output: None保持旧行为；coffee/arm选择奶出口（P28），不是side。
        收到 ACK 后返回，仅确认收包，不代表出品完成。
        water_ml:    水量 ml (10..1000)
        cake_mm10:   粉饼厚度 0.1mm 单位 (30..320 -> 3.0..32.0mm)
        tamping:     压粉力 64/92/120 = 20/40/60kg
        pre_infusion/relax: 0.1s 单位 (0..100)
        milk_quantity: 奶量 0.1s 单位
        milk_temp: 旧格式枚举 0=Warm, 1=Cold, 2=WarmCold, 3=ColdWarm，非摄氏度
        side: 默认 MP0；left/right 为逻辑模块，不代表物理双嘴
        hotwater 使用 Water=2；其他原始类型仅支持 Coffee(+Milk) 流程，
        不支持 Steam/Powder；americano 使用 Coffee 流程和38字节扩展。
        americano_temp 为30..80，要求机器使用摄氏温标；默认70为示例。
        """
        if type(side) is not int or side not in (0, 1):
            raise ValueError('side 必须为整数 0/1')
        data = product_data(product_type, water_ml, bean_hopper, cake_mm10,
                            tamping, pre_infusion, relax, milk_quantity, milk_temp,
                            milk_percent, milk_sequence=milk_sequence,
                            foam_sequence=foam_sequence, hot_water_ml=hot_water_ml,
                            bypass=bypass, americano_temp=americano_temp,
                            americano_sequence=americano_sequence, milk_output=milk_output)
        self.transact(CMD_DO_PRODUCT, mp=side, data=data)


# ---------------- CLI ----------------

def main(argv=None):
    p = argparse.ArgumentParser(description="Eversys 咖啡机控制 (RS232)")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("version", help="查询 API 版本")
    sub.add_parser("status", help="查询机器状态")

    recipe = argparse.ArgumentParser(add_help=False)
    p_prod = recipe
    p_prod.add_argument("--type", default="espresso",
                        choices=list(PRODUCT_TYPES) + ["raw"],
                        help="产品类型")
    p_prod.add_argument("--type-id", type=int, default=None,
                        help="直接指定产品类型数值")
    side_help = "逻辑模块 left=MP0（默认）, right=MP1；不是物理双嘴"
    p_prod.add_argument("--side", choices=["left", "right"], default="left", help=side_help)
    p_prod.add_argument("--water", type=int, default=40, help="水量 ml")
    p_prod.add_argument("--hopper", type=int, default=BEAN_HOPPER_LEFT,
                        choices=[0, 1, 2], help="Cameo 豆仓：0=right, 1=left, 2=mix")
    p_prod.add_argument("--cake", type=int, default=80, help="粉饼厚度 0.1mm")
    p_prod.add_argument("--tamping", type=int, default=92, choices=[64, 92, 120])
    p_prod.add_argument("--milk-qty", type=int, default=0, help="奶/奶泡时间 1..3000 ×0.1s，非ml")
    p_prod.add_argument("--milk-temp", type=int, default=0, choices=[0, 1, 2, 3],
                        help="旧格式枚举：0=Warm, 1=Cold, 2=WarmCold, 3=ColdWarm；非摄氏度")
    p_prod.add_argument("--milk-percent", type=int, default=0, help="Milk Percent 奶比例0..100%%，不是泡沫百分比；示例默认0")
    p_prod.add_argument("--milk-sequence", type=int, choices=range(5), help="0咖啡后奶/1奶后咖啡/2同时/3仅奶/4延迟奶")
    p_prod.add_argument("--foam-sequence", type=int, choices=[0, 1], default=0, help="0泡沫后奶，1奶后泡沫")
    p_prod.add_argument("--milk-output", choices=['coffee', 'arm'], default=None,
                        help="奶出口P28：coffee=咖啡口，arm=独立奶臂（需含奶产品及非零奶时间）；省略保留旧格式，与side无关")
    p_prod.add_argument("--hot-water", type=int, help="美式补水ml 10..1000；示例默认120")
    p_prod.add_argument("--bypass", type=int, help="美式bypass 0..100%%；示例默认0；硬件决定使用它还是补水量")
    p_prod.add_argument("--americano-temp", type=int, help="美式混合温度30..80，要求摄氏温标；示例默认70")
    p_prod.add_argument("--americano-sequence", choices=['coffee-first', 'water-first'], help="先咖啡后水（示例默认）/先水后咖啡")

    sub.add_parser('product', parents=[recipe], help='自定义参数出品，ACK非完成')
    for name in ('espresso', 'coffee', 'americano', 'hotwater', 'milkfoam', 'milk-foam', 'latte'):
        sp = sub.add_parser(name, parents=[copy.deepcopy(recipe)], help=f'快捷 {name}，默认MP0；示例配方，设备适配未实测')
        sp.set_defaults(type=name, water=120 if name in ('coffee', 'hotwater') else 40)
        if name == 'latte':
            sp.set_defaults(milk_qty=100, milk_temp=0, milk_percent=100,
                            milk_sequence=1, foam_sequence=0)

    p_rinse = sub.add_parser("rinse", help="出口冲洗")
    p_rinse.add_argument("--side", choices=["left", "right"], default="left", help=side_help)

    p_stop = sub.add_parser("stop", help="停止")
    p_stop.add_argument("--module", type=int, default=0, help="0=全部")

    sub.add_parser("clean", help="启动清洗")

    args = p.parse_args(argv)
    tokens = list(sys.argv[1:] if argv is None else argv)
    recipe_kwargs = None
    if hasattr(args, 'type'):
        if args.type_id is not None and args.type != 'raw':
            p.error('--type-id 仅用于 --type raw')
        if args.cmd != 'product' and any(t.split('=')[0] in ('--type', '--type-id') for t in tokens):
            p.error('快捷命令不能覆盖产品类型，请使用 product')
        if args.type == 'raw' and args.type_id is None:
            p.error('--type raw 需要 --type-id')
        pt = args.type_id if args.type == 'raw' else PRODUCT_TYPES[args.type]
        milk_flags = ('--milk-qty', '--milk-temp', '--milk-percent', '--milk-sequence', '--foam-sequence')
        coffee_flags = ('--hopper', '--cake', '--tamping')
        for token in tokens:
            flag = token.split('=')[0]
            if (pt not in (12, 13, 14, 15, 16, 17) and flag in milk_flags
                    or pt in (8, 16, 17) and flag in coffee_flags
                    or pt in (16, 17) and flag == '--water'):
                p.error(f'{flag} 不适用于此产品')
        side = 0 if args.side == 'left' else 1
        recipe_kwargs = dict(water_ml=args.water, bean_hopper=args.hopper,
                             cake_mm10=args.cake, tamping=args.tamping,
                             milk_quantity=args.milk_qty, milk_temp=args.milk_temp,
                             milk_percent=args.milk_percent, milk_sequence=args.milk_sequence,
                             foam_sequence=args.foam_sequence, hot_water_ml=args.hot_water,
                             bypass=args.bypass, americano_temp=args.americano_temp,
                             milk_output=args.milk_output,
                             americano_sequence=None if args.americano_sequence is None else
                             int(args.americano_sequence == 'coffee-first'))
        try:
            product_data(pt, **recipe_kwargs)
        except ValueError as exc:
            p.error(str(exc))
    if args.cmd == "product" and args.type == "raw" and args.type_id is None:
        p.error("--type raw 需要 --type-id")

    try:
        with CoffeeMachine(args.port, args.baud) as m:
            if args.cmd == "version":
                v = m.get_api_version()
                print(f"API 版本: V{v[0]:02d}.{v[1]:02d}.{v[2]:02d}")
            elif args.cmd == "status":
                s = m.get_machine_status()
                print(
                    f"机器状态字节: 0x{s.machine_status:02X} "
                    f"(JustReset={s.just_reset}, 有请求={s.has_requests}, "
                    f"信息消息={s.has_info_messages}, ProductDump={s.has_product_dump}, "
                    f"EditMode={s.edit_mode}, 水位高于传感器={s.steam_boiler_water_level}, "
                    f"Standby={s.standby})"
                )
                print("咖啡 MP0/MP1 为逻辑模块，不代表物理双嘴")
                names = {0: "notReady", 1: "ready", 2: "OnlyCoffeeUnitReady",
                         3: "OnlyMilkUnitReady", 15: "Undef"}
                labels = ("咖啡 MP0", "咖啡 MP1", "蒸汽 left", "蒸汽 right", "热水")
                for i, label in enumerate(labels):
                    value = s.module_status(i)
                    print(f"{label}: status={names.get(value, str(value))}, action={s.module_action(i)}")
                print(f"原始数据({len(s.raw)}B): {s.raw.hex(' ')}")
            elif recipe_kwargs is not None:
                m.do_product(pt, side=side, **recipe_kwargs)
                print(f"出品指令已获 ACK（仅确认收包，非出品完成） (type={pt}, 逻辑 MP{side}, 非物理双嘴)")
            elif args.cmd == "rinse":
                side = SIDE_LEFT if args.side == "left" else SIDE_RIGHT
                m.do_rinse(side)
                print(f"冲洗指令已获 ACK（仅确认收包，非完成） (逻辑 MP{side}, 非物理双嘴)")
            elif args.cmd == "stop":
                m.stop(args.module)
                print(f"停止指令已获 ACK（仅确认收包，非完成） (module={args.module})")
            elif args.cmd == "clean":
                m.start_cleaning()
                print("清洗指令已获 ACK（仅确认收包，非完成；后续可能有人工步骤）")
    except CoffeeMachineNack as e:
        print(f"机器拒绝: {e}", file=sys.stderr)
        sys.exit(2)
    except ValueError as e:
        p.error(str(e))
    except (CoffeeMachineError, serial.SerialException) as e:
        print(f"通信失败: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
