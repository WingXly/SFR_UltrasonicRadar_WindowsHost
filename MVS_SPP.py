#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import serial
import struct
import os


class FrameParser:
    """从 SPP 字节流中提取 9 字节雷达数据帧。"""

    HEADER = b"\xAA\x55"
    FRAME_LEN = 9

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []

        while True:
            idx = self.buffer.find(self.HEADER)

            # 没有找到帧头，只保留最后一个字节，防止 0xAA 被截断
            if idx < 0:
                if len(self.buffer) > 1:
                    self.buffer = self.buffer[-1:]
                break

            # 丢弃帧头之前的数据
            if idx > 0:
                del self.buffer[:idx]

            # 数据还不够一帧，继续等待
            if len(self.buffer) < self.FRAME_LEN:
                break

            frame = bytes(self.buffer[:self.FRAME_LEN])

            # 检查帧尾；不是完整帧就丢掉一个字节重新同步
            if frame[8] == 0x0D:
                frames.append(frame)
                del self.buffer[:self.FRAME_LEN]
            else:
                del self.buffer[0]

        return frames


class SPPReceiver:
    """
    JDP-31-SPP 蓝牙串口接收 / 发送器
    - COM9, 9600, 8N1
    - 后台线程持续接收
    - send_command() 用于向上位机 -> 下位机发送字符串指令
    """
    def __init__(self, port="COM9", baudrate=9600, on_bytes=None):
        self.port = port
        self.baudrate = baudrate
        self.on_bytes = on_bytes

        self.ser = None
        self._stop_event = threading.Event()
        self._thread = None
        self._write_lock = threading.Lock()


    def start_in_subProcess(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()


    def stop(self):
        self._stop_event.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


    def send_u16_command(self, cmd_type: int, param1: int, param2: int = None) -> bool:
        """
        按下位机协议发送命令。

        单参数命令:
            [AA 55 05] [cmd_type_hi cmd_type_lo] [param_hi param_lo] [checksum] [0D]
        双参数命令:
            [AA 55 07] [cmd_type_hi cmd_type_lo]
                       [param1_hi param1_lo]
                       [param2_hi param2_lo]
                       [checksum] [0D]
        """
        if self.ser is None or not self.ser.is_open:
            return False

        cmd_type &= 0xFFFF
        param1 &= 0xFFFF

        if param2 is None:
            n = 5
            payload = struct.pack(">HH", cmd_type, param1)
        else:
            n = 7
            param2 &= 0xFFFF
            payload = struct.pack(">HHH", cmd_type, param1, param2)

        frame = bytearray([0xAA, 0x55, n])
        frame.extend(payload)

        checksum = 0
        for b in frame[2:2 + n]:
            checksum ^= b

        frame.append(checksum & 0xFF)
        frame.append(0x0D)

        try:
            with self._write_lock:
                self.ser.write(frame)
            return True
        except Exception as e:
            print("[SPP] 发送失败:", e)
            return False

        
    def _open_serial(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            dsrdtr=False,
            rtscts=False,
        )
        print(f"[SPP] 已打开 {self.port} @ {self.baudrate}")


    def _run(self):
        while not self._stop_event.is_set():
            try:
                if self.ser is None or not self.ser.is_open:
                    self._open_serial()

                # 不依赖 in_waiting：有些蓝牙虚拟串口它一直是 0
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)

                if data and self.on_bytes:
                    self.on_bytes(data)

            except Exception as e:
                print("[SPP] 串口异常:", e)
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0)


"""
uint8_t sendMsg[9]:
帧头         长度    角度      距离        校验      帧尾
0xAA 0x55    N      int16     uint16      XOR      0x0D
0    1       2      3   4     5    6       7        8

角度单位: 0.01度
距离单位: mm
"""
def callback_when_notify(sender, data):
    """每收到一包数据就调用一次。data 必须是长度为 9 的 bytes。"""
    if len(data) < 9:
        return

    # 检查帧头和帧尾
    if not (data[0] == 0xAA and data[1] == 0x55 and data[8] == 0x0D):
        return

    # 检查校验和
    checksum = data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]
    if checksum != data[7]:
        return

    # 解析数据
    angle = 0.0
    distance = 0.0
    try:
        angle_row = (data[3] << 8) + data[4]
        distance_row = (data[5] << 8) + data[6]

        # 转换单位
        angle = float(angle_row) / 100.0        # 0.01 度 -> 度
        distance = float(distance_row) / 1000.0 # mm -> m
    except Exception:
        return

    os.system('cls' if os.name == 'nt' else 'clear')
    print(f"a={angle:.2f}°, d={distance:.3f}m")


def main():

    parser = FrameParser()
    def on_spp_bytes(data: bytes):
        # 从字节流里拆出完整帧，再交给已有的解析函数
        for frame in parser.feed(data):
            callback_when_notify("SPP", frame)

    receiver = SPPReceiver(
        port="COM9",
        baudrate=9600,
        on_bytes=on_spp_bytes,
    )
    receiver.start_in_subProcess()

    print("等待雷达数据...")

    try:
        while True:
            command = input("> ").strip()
            if not command:
                continue

            # 例:
            # cmd 0 0       单参数命令, 解除暂停
            # cmd 0 1       单参数命令, 暂停
            # cmd 1 1       单参数命令, 开 OLED
            # cmd 2 1 5     双参数命令
            parts = command.split()

            if parts[0].lower() == "cmd":
                try:
                    nums = [int(x, 0) for x in parts[1:]]
                except ValueError:
                    print("格式错误，例如: cmd 0 0")
                    continue

                if len(nums) == 2:
                    ok = receiver.send_u16_command(nums[0], nums[1])
                elif len(nums) == 3:
                    ok = receiver.send_u16_command(nums[0], nums[1], nums[2])
                else:
                    print("格式错误，例如: cmd 0 0 或 cmd 2 1 5")
                    continue

                if ok:
                    print(f"[PC -> 下位机] 已发送: {nums}")
                else:
                    print("[PC -> 下位机] 发送失败，串口未连接")
            else:
                print("未知输入。可用格式: cmd <cmd_type> [param1] [param2]")
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()