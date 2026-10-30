#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
落杯机串口控制模块 (协议 V0.0.3, 上海宝路通咖啡机有限公司)

帧格式: 命令码 + 长度码 + 指令码 + 数据1 … 数据n + 校验码 + 结束码(0xFF)
- 指令码: 0x55 查询 / 0xAA 设置
- 长度码: 整帧字节总数
- 校验码: 除校验码和结束码外所有字节之和的低 8 位
"""

import time
from dataclasses import dataclass

import serial

# 当前落杯机实际连接的适配器；不要使用会随 USB 枚举变化的 ttyUSB 编号。
DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0FNBSZ-if00-port0"

# ---------- 协议常量 ----------
CMD_QUERY_STATUS = 0x01   # 从机状态查询
CMD_QUERY_PARAMS = 0x02   # 从机参数查询
CMD_SHUTDOWN = 0x03       # 关机
CMD_DISPENSE = 0x04       # 出饮品

INS_QUERY = 0x55          # 查询
INS_SET = 0xAA            # 设置

FRAME_END = 0xFF

# 系统工作状态 (数据2)
WORK_STATE = {0: "待机", 1: "落杯中", 2: "故障", 3: "找杯中"}


class ProtocolError(Exception):
    """通信协议错误"""
    pass


@dataclass
class DispenserStatus:
    """从机状态 (命令码 0x01 的回复)"""
    cup_missing: bool          # Bit0: 缺杯
    cup_not_taken: bool        # Bit1: 有杯子未取走
    drawer_open: bool          # Bit2: 功能抽屉已拉出
    motor_fault: bool          # Bit3: 电机故障
    seeking_cup: bool          # Bit4: 寻杯中
    work_state: int            # 数据2: 系统工作状态

    @property
    def work_state_name(self):
        return WORK_STATE.get(self.work_state, f"未知({self.work_state})")

    @property
    def ready(self):
        """待机且无异常，允许下发取杯命令"""
        return (self.work_state == 0
                and not self.cup_missing
                and not self.motor_fault
                and not self.drawer_open)

    def __str__(self):
        flags = []
        if self.cup_missing:
            flags.append("缺杯")
        if self.cup_not_taken:
            flags.append("有杯未取走")
        if self.drawer_open:
            flags.append("抽屉已拉出")
        if self.motor_fault:
            flags.append("电机故障")
        if self.seeking_cup:
            flags.append("寻杯中")
        flag_str = "、".join(flags) if flags else "无异常"
        return f"工作状态: {self.work_state_name}, 状态: {flag_str}"


class CupDispenser:
    """落杯机控制类 (主机端)"""

    def __init__(self, port=DEFAULT_PORT, baudrate=115200, timeout=1.0):
        """
        :param port: 串口设备路径
        :param baudrate: 波特率, 协议固定 115200
        :param timeout: 读超时(秒), 协议规定主机 1 秒未收到回复则重发
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
        """校验码 = 所有字节之和的低 8 位"""
        return sum(data) & 0xFF

    def _build_frame(self, cmd, ins, payload=b""):
        """构造完整帧: 命令码+长度码+指令码+数据…+校验码+0xFF"""
        length = 3 + len(payload) + 2  # 命令码+长度码+指令码+数据+校验码+结束码
        body = bytes([cmd, length, ins]) + payload
        return body + bytes([self._checksum(body), FRAME_END])

    def _read_frame(self):
        """按 长度码+结束码 规则读取一帧, 返回整帧字节"""
        # 先读 命令码+长度码
        header = self.ser.read(2)
        if len(header) < 2:
            raise ProtocolError("读取帧头超时")
        length = header[1]
        if length < 5:
            raise ProtocolError(f"非法长度码: {length}")
        rest = self.ser.read(length - 2)
        if len(rest) < length - 2:
            raise ProtocolError("读取帧体超时")
        frame = header + rest
        if frame[-1] != FRAME_END:
            raise ProtocolError(f"结束码错误: 0x{frame[-1]:02X}")
        if self._checksum(frame[:-2]) != frame[-2]:
            raise ProtocolError(
                f"校验失败: 期望 0x{self._checksum(frame[:-2]):02X}, "
                f"实际 0x{frame[-2]:02X}")
        return frame

    def _transact(self, cmd, ins, payload=b"", retries=3):
        """发送一帧并等待回复, 1 秒无回复重发, 最多 retries 次"""
        frame = self._build_frame(cmd, ins, payload)
        last_err = None
        for attempt in range(retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                resp = self._read_frame()
                if resp[0] != cmd:
                    raise ProtocolError(
                        f"回复命令码不匹配: 期望 0x{cmd:02X}, 实际 0x{resp[0]:02X}")
                return resp
            except ProtocolError as e:
                last_err = e
                if attempt < retries - 1:
                    time.sleep(0.05)
        raise ProtocolError(f"通讯错误(重发{retries}次无回复): {last_err}")

    # ---------- 协议命令 ----------

    def query_status(self):
        """0x01 从机状态查询, 返回 DispenserStatus"""
        resp = self._transact(CMD_QUERY_STATUS, INS_QUERY)
        if resp[2] != INS_QUERY or len(resp) < 7:
            raise ProtocolError(f"状态回复格式异常: {resp.hex(' ')}")
        d1, d2 = resp[3], resp[4]
        return DispenserStatus(
            cup_missing=bool(d1 & 0x01),
            cup_not_taken=bool(d1 & 0x02),
            drawer_open=bool(d1 & 0x04),
            motor_fault=bool(d1 & 0x08),
            seeking_cup=bool(d1 & 0x10),
            work_state=d2,
        )

    def query_params(self):
        """0x02 从机参数查询, 返回 dict(含版本号)"""
        resp = self._transact(CMD_QUERY_PARAMS, INS_QUERY)
        data = resp[3:-2]  # 数据1~13
        if len(data) != 13:
            raise ProtocolError(f"参数回复长度异常: {len(data)}")
        return {
            "raw": data,
            "version": f"{data[10]}.{data[11]}.{data[12]}",  # 数据11~13
        }

    def shutdown(self):
        """0x03 关机 (仅非故障状态可用), 成功返回 True"""
        resp = self._transact(CMD_SHUTDOWN, INS_SET)
        return resp[3] == 0x01

    def dispense(self, drink_id=1):
        """0x04 出饮品

        :param drink_id: 饮品编号, 1=落一个杯
        :return: 成功返回 True
        """
        resp = self._transact(CMD_DISPENSE, INS_SET, bytes([drink_id, 0x00]))
        return resp[4] == 0x01

    # ---------- 便捷方法 ----------

    def dispense_and_wait(self, drink_id=1, timeout=30.0, poll=0.3):
        """落一杯并阻塞等待完成

        :raises ProtocolError: 下发失败或等待超时
        """
        status = self.query_status()
        if status.work_state == 2:
            raise ProtocolError(f"设备故障, 无法出杯: {status}")
        if not self.dispense(drink_id):
            raise ProtocolError("出杯指令被从机拒绝(设置失败)")

        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.query_status()
            if st.work_state == 0:  # 回到待机
                return st
            if st.work_state == 2:
                raise ProtocolError(f"落杯过程中设备故障: {st}")
            time.sleep(poll)
        raise ProtocolError(f"等待落杯完成超时({timeout}s)")


# ---------- 命令行演示 ----------

def main():
    import argparse

    parser = argparse.ArgumentParser(description="落杯机串口控制 (协议 V0.0.3)")
    parser.add_argument("-p", "--port", default=DEFAULT_PORT, help="串口设备（默认使用落杯机 USB 唯一标识）")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("status", help="查询从机状态")
    sub.add_parser("params", help="查询从机参数/版本")
    sub.add_parser("drop", help="落一个杯(等待完成)")
    sub.add_parser("shutdown", help="关机")
    args = parser.parse_args()

    with CupDispenser(port=args.port) as dev:
        if args.action == "status":
            st = dev.query_status()
            print(st)
            print(f"可接受取杯命令: {'是' if st.ready else '否'}")
        elif args.action == "params":
            info = dev.query_params()
            print(f"固件版本: {info['version']}")
            print(f"原始数据: {info['raw'].hex(' ')}")
        elif args.action == "drop":
            print("下发落杯指令…")
            st = dev.dispense_and_wait(drink_id=1)
            print(f"完成: {st}")
        elif args.action == "shutdown":
            ok = dev.shutdown()
            print("关机成功" if ok else "关机失败")


if __name__ == "__main__":
    main()
