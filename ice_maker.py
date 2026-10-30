#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
制冰机串口控制模块 (晶晶电子 V1.0 260626)

帧格式: 0xAA 0x55 + 长度 + 指令 + 参数... + 校验
- 长度: 整帧字节总数 (5~31, 最大 32 字节缓冲区)
- 校验: 从帧头开始所有字节求和对 256 取余 (低 8 位)
- 串口: 9600 8N1
"""

import time
from dataclasses import dataclass

import serial

# 用户通过拔插确认的制冰机适配器；不回退到其他串口。
DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0FNBT7-if00-port0"

# ---------- 协议常量 ----------
FRAME_HEADER = b"\xAA\x55"

CMD_POLL = 0xB0          # 查询存在/版本
CMD_STATUS = 0xB1        # 查询状态
CMD_ERROR = 0xB2         # 查询异常代码
CMD_TAKE_ICE = 0xB3      # 取冰
CMD_TAKE_ICE_WATER = 0xB4  # 取冰水
CMD_TAKE_WATER = 0xB5    # 取水
CMD_STOP = 0xB6          # 停止取冰/取冰水/取水
CMD_POWER = 0xB7         # 开关机
CMD_CLEAN = 0xB8         # 清洗
CMD_LOCK = 0xB9          # 键锁
CMD_PAUSE = 0xBA         # 暂停/继续制冰
CMD_FORCE_READY = 0xBB   # 跳过开机等待, 强制允许取冰

# 时长取 (仅配置【时长取】机型)
CMD_TIMED_ICE = 0xA0
CMD_TIMED_ICE_WATER = 0xA1
CMD_TIMED_WATER = 0xA2

# 称重取 (仅配置【称重取】机型)
CMD_TARE = 0x90          # 清零
CMD_WEIGH_ICE = 0x91
CMD_WEIGH_ICE_WATER = 0x92
CMD_WEIGH_WATER = 0x93

# 异常代码表 (表 1)
ERROR_CODES = {
    0x00: "无异常",
    0xE0: "进水异常",
    0xE1: "冷凝器温度异常",
    0xE2: "蒸发器温度异常",
    0xE3: "蒸发器电机转速异常",
    0xE4: "水箱溢出",
    0xE5: "环境温度低",
    0xE6: "水位传感器异常",
    0xE7: "蒸发器温度传感器异常",
    0xE8: "冷凝器温度传感器异常",
    0xE9: "冰量长时间未满",
}


class IceMakerError(Exception):
    """制冰机通信/运行异常"""
    pass


@dataclass
class IceMakerStatus:
    """0xB1 查询状态回复"""
    power_on: bool       # Bit7: 开机
    ice_full: bool       # Bit6: 冰满
    busy: bool           # Bit5: 忙
    fault: bool          # Bit4: 异常
    cleaning: bool       # Bit3: 清洗中
    key_locked: bool     # Bit2: 键锁
    ready: bool          # Bit1: 允许取冰(就绪)
    paused: bool         # Bit0: 暂停制冰

    @classmethod
    def from_byte(cls, b):
        return cls(
            power_on=bool(b & 0x80),
            ice_full=bool(b & 0x40),
            busy=bool(b & 0x20),
            fault=bool(b & 0x10),
            cleaning=bool(b & 0x08),
            key_locked=bool(b & 0x04),
            ready=bool(b & 0x02),
            paused=bool(b & 0x01),
        )

    def __str__(self):
        flags = []
        flags.append("开机" if self.power_on else "关机")
        if self.ice_full:
            flags.append("冰满")
        flags.append("忙" if self.busy else "闲")
        if self.fault:
            flags.append("异常")
        if self.cleaning:
            flags.append("清洗中")
        if self.key_locked:
            flags.append("键锁")
        flags.append("可取冰" if self.ready else "未就绪")
        if self.paused:
            flags.append("暂停制冰")
        return "、".join(flags)


class IceMaker:
    """制冰机控制类 (主机端)"""

    def __init__(self, port=DEFAULT_PORT, baudrate=9600, timeout=1.0):
        """
        :param port: 串口设备路径
        :param baudrate: 协议固定 9600
        :param timeout: 读超时(秒)
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self.timeout = timeout

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # ---------- 帧处理 ----------

    @staticmethod
    def _checksum(data):
        """校验 = 所有字节之和的低 8 位"""
        return sum(data) & 0xFF

    def _build_frame(self, cmd, params=b""):
        """构造帧: AA 55 + 长度 + 指令 + 参数 + 校验

        长度 = 整帧总字节数 (含帧头/长度/校验)。
        协议示例: AA 55 05 B0 B4 → 5 字节
        """
        length = 2 + 1 + 1 + len(params) + 1  # 帧头2+长度1+指令1+参数n+校验1
        if length > 0x1F:
            raise ValueError(f"帧过长: {length} > 31")
        body = FRAME_HEADER + bytes([length, cmd]) + params
        return body + bytes([self._checksum(body)])

    def _read_frame(self):
        """读取一帧, 返回 (指令, 参数)"""
        # 同步帧头 0xAA 0x55
        hdr = self.ser.read(2)
        if len(hdr) < 2 or hdr != FRAME_HEADER:
            raise IceMakerError(f"帧头错误: {hdr.hex(' ') if hdr else '超时'}")
        len_b = self.ser.read(1)
        if not len_b:
            raise IceMakerError("读取长度超时")
        length = len_b[0]
        if not 5 <= length <= 0x1F:
            raise IceMakerError(f"非法长度: {length}")
        rest = self.ser.read(length - 3)  # 指令+参数+校验 (已读帧头2+长度1)
        if len(rest) < length - 3:
            raise IceMakerError("读取帧体超时")
        frame = hdr + len_b + rest
        if self._checksum(frame[:-1]) != frame[-1]:
            raise IceMakerError(
                f"校验失败: 期望 0x{self._checksum(frame[:-1]):02X}, "
                f"实际 0x{frame[-1]:02X}")
        return frame[3], frame[4:-1]  # (指令, 参数)

    def _transact(self, cmd, params=b"", retries=3):
        """发送并等待匹配回复, 返回从机回复的参数字节"""
        frame = self._build_frame(cmd, params)
        last_err = None
        for _ in range(retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                rcmd, rparams = self._read_frame()
                if rcmd != cmd:
                    raise IceMakerError(
                        f"回复指令不匹配: 期望 0x{cmd:02X}, 实际 0x{rcmd:02X}")
                return rparams
            except IceMakerError as e:
                last_err = e
                time.sleep(0.05)
        raise IceMakerError(f"通讯失败(重试{retries}次): {last_err}")

    @staticmethod
    def _check_exec(params, what="操作"):
        """检查 0x01=执行 / 0x00=忙/未就绪 的通用回复"""
        if not params:
            raise IceMakerError(f"{what}: 回复无参数")
        if params[0] != 0x01:
            raise IceMakerError(f"{what}: 从机忙或未就绪")
        return True

    # ---------- 基础指令 ----------

    def poll(self):
        """0xB0 查询从机是否存在及软件版本, 返回 (程序标识, 软件版本)"""
        params = self._transact(CMD_POLL)
        if len(params) < 2:
            raise IceMakerError("版本回复参数不足")
        return params[0], params[1]

    def query_status(self):
        """0xB1 查询状态, 返回 IceMakerStatus"""
        params = self._transact(CMD_STATUS)
        return IceMakerStatus.from_byte(params[0])

    def query_error(self):
        """0xB2 查询异常代码, 返回 (代码, 描述)"""
        params = self._transact(CMD_ERROR)
        code = params[0]
        return code, ERROR_CODES.get(code, f"未知异常(0x{code:02X})")

    def take_ice(self):
        """0xB3 取冰"""
        return self._check_exec(self._transact(CMD_TAKE_ICE), "取冰")

    def take_ice_water(self):
        """0xB4 取冰水"""
        return self._check_exec(self._transact(CMD_TAKE_ICE_WATER), "取冰水")

    def take_water(self):
        """0xB5 取水"""
        return self._check_exec(self._transact(CMD_TAKE_WATER), "取水")

    def stop(self):
        """0xB6 停止取冰/取冰水/取水"""
        return self._check_exec(self._transact(CMD_STOP), "停止")

    def power(self, on: bool):
        """0xB7 开机/关机"""
        return self._check_exec(
            self._transact(CMD_POWER, bytes([0x01 if on else 0x00])),
            "开机" if on else "关机")

    def clean(self, start: bool):
        """0xB8 清洗/停止清洗"""
        return self._check_exec(
            self._transact(CMD_CLEAN, bytes([0x01 if start else 0x00])),
            "清洗" if start else "停止清洗")

    def lock(self, locked: bool):
        """0xB9 键锁 上锁/解锁"""
        return self._check_exec(
            self._transact(CMD_LOCK, bytes([0x01 if locked else 0x00])),
            "上锁" if locked else "解锁")

    def pause_ice_making(self, pause: bool):
        """0xBA 暂停制冰/继续制冰"""
        return self._check_exec(
            self._transact(CMD_PAUSE, bytes([0x01 if pause else 0x00])),
            "暂停制冰" if pause else "继续制冰")

    def force_ready(self):
        """0xBB 跳过开机制冰等待, 强制允许取冰"""
        return self._check_exec(self._transact(CMD_FORCE_READY), "强制就绪")

    # ---------- 时长取 (仅时长取机型) ----------

    def _timed(self, cmd, seconds, what):
        """时长取通用: 0.0~25.5 秒 (0.1 秒/单位)"""
        if not 0 <= seconds <= 25.5:
            raise ValueError("时长需在 0~25.5 秒")
        val = round(seconds * 10)
        return self._check_exec(self._transact(cmd, bytes([val])), what)

    def take_ice_timed(self, seconds):
        """0xA0 按时长取冰 (完成后从机会主动报告 0xA0 0x02)"""
        return self._timed(CMD_TIMED_ICE, seconds, "时长取冰")

    def take_ice_water_timed(self, seconds):
        """0xA1 按时长取冰水"""
        return self._timed(CMD_TIMED_ICE_WATER, seconds, "时长取冰水")

    def take_water_timed(self, seconds):
        """0xA2 按时长取水"""
        return self._timed(CMD_TIMED_WATER, seconds, "时长取水")

    # ---------- 称重取 (仅称重取机型) ----------

    def tare(self):
        """0x90 清零"""
        return self._check_exec(self._transact(CMD_TARE), "清零")

    def _weigh(self, cmd, grams, what):
        """称重取通用: 40~400g, 2 字节大端"""
        if not 40 <= grams <= 400:
            raise ValueError("重量需在 40~400g")
        return self._check_exec(
            self._transact(cmd, grams.to_bytes(2, "big")), what)

    def take_ice_weighed(self, grams):
        """0x91 称重取冰 (完成后从机主动报告 0x91 0x02+实际重量2字节)"""
        return self._weigh(CMD_WEIGH_ICE, grams, "称重取冰")

    def take_ice_water_weighed(self, grams):
        """0x92 称重取冰水"""
        return self._weigh(CMD_WEIGH_ICE_WATER, grams, "称重取冰水")

    def take_water_weighed(self, grams):
        """0x93 称重取水"""
        return self._weigh(CMD_WEIGH_WATER, grams, "称重取水")

    def wait_completion_report(self, timeout=60.0):
        """等待从机主动上报执行完成 (时长取/称重取)

        :return: (指令, 参数字节)  例如 (0x91, b'\\x02\\x00\\x64')
        """
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        try:
            return self._read_frame()
        finally:
            self.ser.timeout = old_timeout

    # ---------- 便捷方法 ----------

    def ensure_ready(self):
        """确认可取冰, 否则抛异常"""
        st = self.query_status()
        if st.fault:
            code, desc = self.query_error()
            raise IceMakerError(f"设备异常: {desc} (0x{code:02X})")
        if not st.power_on:
            raise IceMakerError("设备未开机")
        if not st.ready:
            raise IceMakerError(f"未就绪, 当前状态: {st}")
        return st


# ---------- 命令行 ----------

def main():
    import argparse

    parser = argparse.ArgumentParser(description="制冰机串口控制 (晶晶电子 V1.0)")
    parser.add_argument("-p", "--port", default=DEFAULT_PORT, help="串口设备（默认使用制冰机 USB 唯一标识）")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("poll", help="查询存在/版本")
    sub.add_parser("status", help="查询状态")
    sub.add_parser("error", help="查询异常代码")
    sub.add_parser("ice", help="取冰")
    sub.add_parser("ice-water", help="取冰水")
    sub.add_parser("water", help="取水")
    sub.add_parser("stop", help="停止取冰/取冰水/取水")

    p = sub.add_parser("power", help="开机/关机")
    p.add_argument("state", choices=["on", "off"])

    p = sub.add_parser("clean", help="清洗/停止清洗")
    p.add_argument("state", choices=["start", "stop"])

    p = sub.add_parser("lock", help="键锁 上锁/解锁")
    p.add_argument("state", choices=["lock", "unlock"])

    p = sub.add_parser("pause", help="暂停/继续制冰")
    p.add_argument("state", choices=["pause", "resume"])

    sub.add_parser("force", help="跳过开机等待, 强制允许取冰")

    p = sub.add_parser("ice-t", help="按时长取冰(秒)")
    p.add_argument("seconds", type=float)
    p = sub.add_parser("water-t", help="按时长取水(秒)")
    p.add_argument("seconds", type=float)

    p = sub.add_parser("ice-w", help="称重取冰(克)")
    p.add_argument("grams", type=int)
    p = sub.add_parser("water-w", help="称重取水(克)")
    p.add_argument("grams", type=int)
    sub.add_parser("tare", help="称重清零")

    args = parser.parse_args()

    with IceMaker(port=args.port) as dev:
        if args.action == "poll":
            pid, ver = dev.poll()
            print(f"设备在线 | 程序标识: 0x{pid:02X} | 软件版本: 0x{ver:02X}")
        elif args.action == "status":
            print(dev.query_status())
        elif args.action == "error":
            code, desc = dev.query_error()
            print(f"异常代码 0x{code:02X}: {desc}")
        elif args.action == "ice":
            dev.take_ice(); print("取冰指令已执行")
        elif args.action == "ice-water":
            dev.take_ice_water(); print("取冰水指令已执行")
        elif args.action == "water":
            dev.take_water(); print("取水指令已执行")
        elif args.action == "stop":
            dev.stop(); print("已停止")
        elif args.action == "power":
            dev.power(args.state == "on")
            print("已开机" if args.state == "on" else "已关机")
        elif args.action == "clean":
            dev.clean(args.state == "start")
            print("开始清洗" if args.state == "start" else "停止清洗")
        elif args.action == "lock":
            dev.lock(args.state == "lock")
            print("已上锁" if args.state == "lock" else "已解锁")
        elif args.action == "pause":
            dev.pause_ice_making(args.state == "pause")
            print("已暂停制冰" if args.state == "pause" else "继续制冰")
        elif args.action == "force":
            dev.force_ready(); print("已强制允许取冰")
        elif args.action == "ice-t":
            dev.take_ice_timed(args.seconds)
            print(f"按时长取冰 {args.seconds}s 已执行, 等待完成…")
            cmd, params = dev.wait_completion_report()
            print(f"完成上报: 指令 0x{cmd:02X} 参数 {params.hex(' ')}")
        elif args.action == "water-t":
            dev.take_water_timed(args.seconds)
            print(f"按时长取水 {args.seconds}s 已执行, 等待完成…")
            cmd, params = dev.wait_completion_report()
            print(f"完成上报: 指令 0x{cmd:02X} 参数 {params.hex(' ')}")
        elif args.action == "ice-w":
            dev.take_ice_weighed(args.grams)
            print(f"称重取冰 {args.grams}g 已执行, 等待完成…")
            cmd, params = dev.wait_completion_report()
            actual = int.from_bytes(params[1:3], "big") if len(params) >= 3 else "?"
            print(f"完成上报: 实际重量 {actual}g")
        elif args.action == "water-w":
            dev.take_water_weighed(args.grams)
            print(f"称重取水 {args.grams}g 已执行, 等待完成…")
            cmd, params = dev.wait_completion_report()
            actual = int.from_bytes(params[1:3], "big") if len(params) >= 3 else "?"
            print(f"完成上报: 实际重量 {actual}g")
        elif args.action == "tare":
            dev.tare(); print("已清零")


if __name__ == "__main__":
    main()
