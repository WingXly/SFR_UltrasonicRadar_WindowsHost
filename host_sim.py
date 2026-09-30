import asyncio
import queue
import threading
import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner
from collections import deque


SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID  = "0000ffe1-0000-1000-8000-00805f9b34fb"

data_queue = queue.Queue()
stop_event = threading.Event()

async def ble_reader():
    print("正在寻找蓝牙设备...")
    devices = await BleakScanner.discover(timeout=8, return_adv=True)
    target = None
    for dev, adv in devices.values():
        name = adv.local_name or dev.name or ""
        if "BT24" in name:
            target = dev
            break
    if target is None:
        print("没找到 BT24")
        return
    else:
        print("连接成功！")

    # 从蓝牙中获取数据，处理并存放到data_queue队列当中
    async with BleakClient(target) as client:
        def on_notify(sender, data):
            try:
                text = data.decode(errors="ignore").strip()
                if not text: return
                parts = text.split(",")
                d = float(parts[0].split("=")[1])
                a = float(parts[1].split("=")[1]) * 3.14159 / 180
                data_queue.put((a, d))
            except Exception:
                pass
        await client.start_notify(NOTIFY_UUID, on_notify)
        
        while not stop_event.is_set():
            await asyncio.sleep(0.2)

def main():
    fig = plt.figure()                            # 创建一个画布（fig）
    ax = fig.add_subplot(111, projection="polar") # 在fig下创建子图（极坐标图）
    ax.set_thetamin(0)                            # 设置极坐标最小角为0度
    ax.set_thetamax(180)                          # 设置极坐标最大角为180度
    ax.set_rlim(0, 200)                           # 设置r轴的显示范围为0-200
    line_plt = ax.plot([], [], 'r-')[0]           # 初始化ax

    angles= deque(maxlen=10)
    distances = deque(maxlen=10)

    from matplotlib.animation import FuncAnimation
    def update(frame):
        while not data_queue.empty():
            a, d = data_queue.get()
            angles.append(a)
            distances.append(d)
        line_plt.set_data(angles, distances)
        return line_plt,
    ani = FuncAnimation(fig, update, interval=100, blit=False, save_count=10)

    worker = threading.Thread(
        target=lambda: asyncio.run(ble_reader()), daemon=True
    )
    worker.start()
    plt.show()
    stop_event.set()

if __name__ == "__main__":
    main()