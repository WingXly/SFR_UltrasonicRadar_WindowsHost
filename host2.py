import serial
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM5', 115200)
angles, distances = [], []

fig, ax = plt.subplots()
while True:
    line = ser.readline().decode().strip()
    # 假设STM32发 "A:45,D:120"
    parts = line.split(',')
    distance = float(parts[0].split('=')[1])
    angle = float(parts[1].split('=')[1])
    angles.append(np.radians(angle))
    distances.append(distance)
    # 极坐标画图
    ax.clear()
    ax.plot(angles, distances, 'b.')
    ax.set_ylim(0, 200)
    plt.pause(0.01)