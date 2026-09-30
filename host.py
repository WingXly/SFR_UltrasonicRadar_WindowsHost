"""DX-BT24 BLE 雷达上位机"""

import asyncio
import re
import queue
import threading
import time
from collections import deque

import numpy as np
from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


DEVICE_NAME = "BT24"         # 匹配广播名称，也可匹配 DX-BT24
DEVICE_ADDRESS = ""          # 多个同名模块时，填扫描输出中的蓝牙地址
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
# 9600 是 STM32 与 BT24 之间的 UART 波特率，电脑 BLE 端不设置波特率。
MAX_DISTANCE_CM = 200        # 极坐标最大半径
MAX_POINTS = 80              # 最多保留多少个点


data_queue = queue.Queue(maxsize=1000)
status_queue = queue.Queue()
stop_event = threading.Event()
pattern = re.compile(
    rb"d\s*=\s*([-+]?\d*\.?\d+)\s*,\s*a\s*=\s*([-+]?\d*\.?\d+)"
)


def set_status(message, display):
    print(message, flush=True)
    status_queue.put(display)


def parse_radar_data(buffer, data):
    """BLE 通知可能拆分或合并数据行；只解析以换行结尾的完整记录。"""
    buffer.extend(data)
    readings = []
    while b"\n" in buffer:
        end = buffer.index(b"\n")
        raw = bytes(buffer[:end]).strip()
        del buffer[:end + 1]
        match = pattern.fullmatch(raw)
        if match:
            distance, angle = map(float, match.groups())
            if 0 <= angle <= 180 and 0 <= distance <= MAX_DISTANCE_CM:
                readings.append((angle, distance))
    # 错误数据一直没有换行时，防止缓冲区无限增长。
    if len(buffer) > 4096:
        buffer.clear()
    return readings


async def ble_reader():
    while not stop_event.is_set():
        try:
            set_status("正在搜索 BT24……", "Scanning for BT24...")
            devices = await BleakScanner.discover(timeout=8, return_adv=True)
            candidates = []
            for device, advertisement in devices.values():
                name = advertisement.local_name or device.name or ""
                if DEVICE_ADDRESS:
                    matches = device.address.casefold() == DEVICE_ADDRESS.casefold()
                else:
                    matches = DEVICE_NAME.casefold() in name.casefold()
                if matches:
                    print(f"找到设备：{name} [{device.address}]", flush=True)
                    candidates.append(device)
            if not candidates:
                raise RuntimeError("未找到 BT24，请检查供电、电脑蓝牙，并断开网页/手机连接")
            if len(candidates) > 1:
                raise RuntimeError("找到多个 BT24，请将目标地址填入 DEVICE_ADDRESS")

            device = candidates[0]
            set_status(f"正在连接 {device.address}……", "Connecting to BT24...")
            async with BleakClient(device, timeout=15) as client:
                service = client.services.get_service(SERVICE_UUID)
                characteristic = service.get_characteristic(NOTIFY_UUID) if service else None
                if characteristic is None or "notify" not in characteristic.properties:
                    raise RuntimeError("未找到 FFE0/FFE1 通知通道，请核对网页助手的 UUID 设置")

                # 每次重新连接都从空缓冲开始，避免拼接上次断线前的残包。
                buffer = bytearray()
                received_bytes = 0
                last_valid_time = None

                def on_notification(sender, data):
                    nonlocal received_bytes, last_valid_time
                    if not received_bytes:
                        print(f"首包 BLE 数据：{bytes(data)!r}", flush=True)
                    received_bytes += len(data)
                    for reading in parse_radar_data(buffer, data):
                        last_valid_time = time.monotonic()
                        try:
                            data_queue.put_nowait(reading)
                        except queue.Full:
                            pass

                await client.start_notify(characteristic, on_notification)
                set_status("蓝牙已连接，正在等待雷达数据……", "Connected; waiting for data...")
                connected_time = time.monotonic()
                previous_state = "waiting"
                while client.is_connected and not stop_event.is_set():
                    now = time.monotonic()
                    if last_valid_time is not None and now - last_valid_time < 5:
                        state = "receiving"
                    elif now - connected_time >= 5:
                        state = "no_valid_data" if received_bytes else "no_data"
                    else:
                        state = "waiting"
                    if state != previous_state:
                        if state == "receiving":
                            set_status("正在接收雷达数据。", "Receiving radar data")
                        elif state == "no_data":
                            set_status("已连接但未收到数据，请确认下位机正在发送。", "Connected; no data received")
                        elif state == "no_valid_data":
                            set_status("近期无有效雷达数据，请检查发送状态、数据格式及量程。", "No recent valid radar data")
                        previous_state = state
                    await asyncio.sleep(0.2)
                if not stop_event.is_set():
                    set_status("蓝牙已断开，准备重新连接……", "Disconnected; reconnecting...")
        except Exception as exc:
            set_status(f"蓝牙连接/接收异常：{type(exc).__name__}: {exc}", "BLE error; see terminal")
        if not stop_event.is_set():
            await asyncio.sleep(3)


async def run_ble():
    # 窗口关闭时也能取消正在进行的扫描/连接，让 BLE 上下文释放连接。
    task = asyncio.create_task(ble_reader())
    try:
        while not stop_event.is_set() and not task.done():
            await asyncio.sleep(0.1)
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)


def main():
    print("请先断开BT24的蓝牙连接，无需在设置中配对。", flush=True)
    stop_event.clear()

    # ==================== 极坐标显示 ====================
    angles_deg = deque(maxlen=MAX_POINTS)
    distances = deque(maxlen=MAX_POINTS)
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("E")
    ax.set_theta_direction(1)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_ylim(0, MAX_DISTANCE_CM)
    ax.set_title("Ultrasonic Radar - BT24 BLE")
    ax.grid(True)
    line, = ax.plot([], [], "r.-", markersize=4)
    info_text = ax.text(0.02, 1.05, "", transform=ax.transAxes)
    status_text = fig.text(0.5, 0.03, "Starting BLE...", ha="center")

    def update(frame):
        while not status_queue.empty():
            try:
                status_text.set_text(status_queue.get_nowait())
            except queue.Empty:
                break
        updated = False
        while not data_queue.empty():
            try:
                angle, distance = data_queue.get_nowait()
            except queue.Empty:
                break
            angles_deg.append(np.deg2rad(angle))
            distances.append(distance)
            updated = True
        if updated and angles_deg:
            theta = np.array(angles_deg)
            r = np.array(distances)
            idx = np.argsort(theta)
            line.set_data(theta[idx], r[idx])
            info_text.set_text(
                f"angle = {np.rad2deg(theta[-1]):.1f}°, "
                f"distance = {r[-1]:.1f} cm"
            )
        return line, info_text, status_text

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    fig.canvas.mpl_connect("close_event", lambda event: stop_event.set())
    plt.tight_layout(rect=(0, 0.06, 1, 1))
    worker = threading.Thread(target=lambda: asyncio.run(run_ble()), daemon=True)
    worker.start()
    try:
        plt.show()
    finally:
        stop_event.set()
        worker.join(timeout=10)


if __name__ == "__main__":
    main()
