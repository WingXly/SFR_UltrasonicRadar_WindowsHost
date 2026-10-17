# 基于 FreeRTOS 与 ROS2 的分布式超声波扫描系统（Windows上位机）

注意：此仓库为 Windows 上位机软件

---

## 上位机简介

使用 pyserial 库对蓝牙传入的数据进行处理。仅用于快速可行性验证，如需获得完整体验（例如 RViz2 可视化和更加人性化的指令系统），则需要使用 ROS2 版。

---

## 目录结构

```text
host_windows/                # Windows 上位机
├── MVS_SPP.py               # pyserial + 蓝牙 SPP 版本
├── MVS.py                   # BLE bleak 版本（旧）
├── LICENSE
└── ...
```

---

## 功能特性

- Windows 版使用 pyserial + Matplotlib/控制台，方便单独测试硬件和算法。

---

## 技术栈

- Python 3.10+
- pyserial
- numPy
- matplotlib

---

## Windows 上位机

发行版位于：

```text
host_windows/MVS_SPP.py
```

### 环境

```bash
pip install pyserial
```

如果需要滤波、插值和绘图，再安装：

```bash
pip install numpy scipy matplotlib
```

### 运行

确认设备管理器中蓝牙 SPP 的 COM 口，并确保串口调试助手已关闭。

修改 `MVS_SPP.py`：

```python
client = SppReceiver(
    port="COM9",
    baudrate=9600,
    on_bytes=callback_when_receive_data
)
```

运行：

```bash
cd host_windows
python MVS_SPP.py
```

程序会解析下位机二进制帧，并输出：

```text
d=1.234, a=45.00
```

### 后处理流程

```text
原始点
  -> 同一角度重复采样
  -> 均值/中值滤波
  -> 固定到 0.45° 角度表
  -> 相邻有效角度之间插值
  -> 绘制极坐标轮廓
```

> 经验证，SG90 舵机的步进角度为0.9°的整数倍，即0.45°的偶数倍

插值建议：

- 实时优先：线性插值
- 平滑轮廓：PCHIP 单调分段三次插值
- 大空洞不插值，避免把不存在的障碍物连起来

## 常见问题

### 1. 蓝牙收到乱码

- 确认蓝牙模块处于 SPP 数据模式；
- 确认 STM32 和 PC 两端波特率一致，当前为 9600；
- 确认帧头（`0XAA`、`0X55`）、帧尾（`0x0D`）与下位机一致；

### 2. Python 收不到数据

- 确认已经关闭了其他的串口调试助手；
- Windows 蓝牙 SPP 可能有多个 COM 口，使用 Outgoing 口；
- 优先用 `ser.read(1)` 验证原始字节，再接入解析器；
- 固件必须发送 `\r\n` 或使用二进制帧解析。

---

## 许可证

MIT，详见 [LICENSE](LICENSE)。