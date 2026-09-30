import asyncio
from bleak import BleakClient, BleakScanner


DEVICE_NAME = "BT24"
NOTIFY_UUID  = "0000ffe1-0000-1000-8000-00805f9b34fb"


def callback_when_notify(sender, data):
    """每收到一包 BLE 数据就调用一次"""
    text = data.decode(errors="ignore").strip()                       # 解码data（将从蓝牙中得到的数据转换为字符串），忽略错误，并去掉开头和结尾的\r\n
    if not text: return

    # 解析 "d=123,a=45"
    try:
        parts = text.split(",")
        d = float(parts[0].split("=")[1])
        a = float(parts[1].split("=")[1])
        msg = f"a={a:.1f}°\nd={d:.1f}cm\n"
        print(msg)
    except Exception:
        pass


async def main():
    # 1. 扫描
    print("扫描中...")
    devices = await BleakScanner.discover(timeout=8, return_adv=True) # 用8s扫描并记录所有蓝牙设备（允许返回广播数据），将结果储存在devices（字典）变量中。等待此步骤完成后main再继续进行
    target = None                                                     # 声明一个目标设备
    for dev, adv in devices.values():
        name = adv.local_name or dev.name or ""
        if DEVICE_NAME in name:
            target = dev
            break

    if target is None:
        print(f"未发现{DEVICE_NAME}")
        return

    # 2. 连接 + 订阅
    async with BleakClient(target) as client:
        print(f"已连接 {target.address}")
        await client.start_notify(NOTIFY_UUID, callback_when_notify)    # 订阅特征 NOTIFY_UUID ，并为这个特征的通知添加回调函数 callback_when_notify
        print("等待数据... (Ctrl+C 退出)")
        while client.is_connected:
            await asyncio.sleep(1)


asyncio.run(main())