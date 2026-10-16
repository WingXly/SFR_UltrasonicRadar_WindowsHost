#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import serial
import struct
import queue


RESOLUTION = 0.45
DEBUG = True


'''
后处理：
0. 将所有传感器数据固定到分辨率0.45°的极坐标上
1. 同一角度重复采样 -> 卡尔曼滤波
2. 相邻两个测量角度 -> 插值（相邻两个测量角度的差值最小是0.9度，且一定是0.9°的倍数）
'''
class AnyFilter:
    def __init__(self):
        self.value  = 0
        self.numbers_of_iteration = 1

    def feed_all(self, new_values: queue.Queue, algorithm: str) -> float:
        dists = []

        while not new_values.empty():
            v = new_values.get_nowait()
            if v is None or v < 0: continue
            dists.append(v)

        if not dists:
            return 0.0
        elif len(dists) == 1:
            return dists[0]

        if algorithm.lower() == 'average':
            return sum(dists) / len(dists)
        
        elif algorithm.lower() == 'mediam':
            dists.sort()
            return dists[len(dists) // 2]

        else:
            pass
        


class PostProcessor:
    def __init__(self):
        # self.this_dist = 0
        self.this_angle = 0
        # self.previous_dist = 0
        self.previous_angle = 0

        # 初始化坐标表格，分辨率：RESOLUTION°
        self.coordinate_graph = [-1] * round(180 / RESOLUTION + 1)

        # 滤波器
        self.filter = AnyFilter()


    def write(self, r, theta):
        '''向self.coordinate_graph中写入一个点'''
        index = round(theta / RESOLUTION)
        if 0 <= index < len(self.coordinate_graph):
            self.coordinate_graph[index] = r
        

    def clear_area(self, from_angle, to_angle):
        from_index = round(from_angle / RESOLUTION)
        to_index = round(to_angle / RESOLUTION)

        left = min(from_index, to_index)
        right = max(from_index, to_index)

        # 开区间内所有数值改为-1
        for index in range(left + 1, right):
            if 0 <= index < len(self.coordinate_graph):
                self.coordinate_graph[index] = -1


    def feed_all(self, new_dists: queue.Queue, new_angle: float):
        if new_dists.empty(): return

        # 滤波
        new_dist = self.filter.feed_all(new_dists, 'average')

        # 更新self数据
        self.previous_angle = self.this_angle
        self.this_angle     = new_angle

        # 更新坐标图
        self.clear_area(self.previous_angle, self.this_angle)
        self.write(new_dist, new_angle)
        self.interpolation_auto()


    def interpolation_auto(self):
        '''将self.coordinate_graph里的-1拟合为其他数字'''
        pass



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



class SppReceiver:
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
def callback_when_notify(sender, data) -> list:
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

    if DEBUG:
        print(f"d={distance}, a={angle}")
    return [distance, angle]


def main():
    parser = FrameParser()
    post_processor = PostProcessor()
    dist_resamples = queue.Queue()
    angle_resamples = queue.Queue()
    previous_angle = -1
    
    def callback_when_receive_data(data: bytes):
        '''从字节流里拆出完整帧，再交给已有的解析函数'''
        nonlocal previous_angle

        for frame in parser.feed(data):
            decode = callback_when_notify("SPP", frame)
            if decode is None: continue
            [dist, angle] = decode

            # 如果角度变化，说明重复采样结束。开始后处理
            if previous_angle == -1:
                previous_angle = angle
            elif angle != previous_angle:
                previous_angle = angle
                post_processor.feed_all(dist_resamples, angle)

            dist_resamples.put(dist)
                
    client = SppReceiver(port="COM9",
                         baudrate=9600,
                         on_bytes=callback_when_receive_data)
    client.start_in_subProcess()
    print("上位机已启动")

    try:
        while True:
            command = input("> ").strip().split()
            if not command: continue

            if command[0].lower() == "cmd":
                try:
                    nums = [int(x, 0) for x in command[1:]]
                except ValueError:
                    print("格式错误")
                    continue

                if len(nums) == 2:
                    SEND_SUCCESS = client.send_u16_command(nums[0], nums[1])
                elif len(nums) == 3:
                    SEND_SUCCESS = client.send_u16_command(nums[0], nums[1], nums[2])
                else:
                    continue

                if SEND_SUCCESS:
                    print(f"[Windows -> STM32] 已发送: {nums}")
                else:
                    print("[Windows -> STM32] 发送失败")

            elif command[0].lower() in ['q', 'quit']:
                print("\n退出")
                break

            else:
                print("未知命令。可用格式: cmd <cmd_type> [param1] [param2]")
                
    except KeyboardInterrupt:
        print("\n退出")

    finally:
        client.stop()


if __name__ == "__main__":
    main()