import asyncio
from bleak import BleakClient, BleakScanner


DEVICE_NAME = "BT24"
NOTIFY_UUID  = "0000ffe1-0000-1000-8000-00805f9b34fb"


'''
uint8_t sendMsg[9]:
帧头         长度    角度     距离        校验      帧尾
0xAA 0x55    N      int16    uint16      XOR      0x0D
0    1       2      3   4    5    6      7        8

角度单位: 0.01度
距离单位: mm
'''

"""每收到一包 BLE 数据就调用一次"""
def callback_when_notify(sender, data):
    # 校验数据
    print("收到数据")
    if not ( data[0] == 0xAA and data[1] == 0x55 and data[8] == 0x0D ): return # 检查帧头和帧尾
    checksum = data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]                 # 检查校验和
    if checksum != data[7]: return
    
    # 解析数据
    try:
        angle_row = (data[3] << 8) + data[4]
        distance_row = (data[5] << 8) + data[6]
        # 转换单位
        angle = float(angle_row) / 100.0
        distance = float(distance_row) / 1000.0
    except Exception:
        pass

    print(f"a={angle:.2f}°, d={distance:.3f}m")


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
    print(target)
    async with BleakClient(target) as client:
        print(f"已连接 {target.address}")
        await client.start_notify(NOTIFY_UUID, callback_when_notify)    # 订阅特征 NOTIFY_UUID ，并为这个特征的通知添加回调函数 callback_when_notify
        print("等待数据... (Ctrl+C 退出)")
        while client.is_connected:
            await asyncio.sleep(1)


asyncio.run(main())