import serial
import serial.tools.list_ports
import time

ports = list(serial.tools.list_ports.comports())
print("可用串口：")
for p in ports:
    print(p.device, "-", p.description, "-", p.hwid)

for p in ports:
    for baud in [9600, 115200, 38400]:
        try:
            ser = serial.Serial(p.device, baud, timeout=0.5)
        except Exception as e:
            print(f"[打开失败] {p.device} @ {baud}: {e}")
            continue

        print(f"\n[打开成功] {p.device} @ {baud}")

        start = time.time()
        got_data = False

        while time.time() - start < 3:   # 每个端口试 3 秒
            n = ser.in_waiting
            data = ser.read(n if n else 1)

            if data:
                got_data = True
                print("RAW:", repr(data))

        ser.close()

        if got_data:
            print(">>> 这个端口有数据 <<<")