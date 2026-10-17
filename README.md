# 基于 FreeRTOS 与 ROS2 的分布式超声波扫描系统 (ROS2 版上位机)

注意，这是 ROS2 lyrical luth 版上位机。

## ROS2 版上位机简介

接收下位机二进制数据、进行滤波与插值，并绘制极坐标雷达图或在 RViz2 中显示（数据格式：LaserScan）。

---

## 目录结构

```text
ros2_ws/
├── ...
└── src/
    ├── laser_scan/                  # ROS2 上位机功能包
    │   ├── laser_scan/
    │   │   ├── stm32.py             # 串口桥接节点
    │   │   ├── cmd_controller.py    # CLI 指令解析节点
    │   │   ├── visualizer.py        # LaserScan 可视化节点
    │   │   └── radar_static_tf.py   # 静态 TF 发布节点
    │   ├── package.xml
    │   └── setup.py
    │
    ├── README.md
    └── LICENSE
```

---

## 功能特性

- 发布 `/radar/sensor_data`、`/radar/laser_scan`，可在 RViz2 中实时显示。

---

## 技术栈

| 平台 | 说明 |
|---|---|
| ROS2 lyrical luth | rclpy、sensor_msgs、std_msgs、geometry_msgs、tf2_ros、RViz2 |
| 可视化 | Matplotlib 极坐标图 / RViz2 LaserScan |
| 滤波 | 均值滤波或中值滤波 |
| 插值 | 线性插值或 PCHIP 平滑插值 |

---

## 快速开始

在开始之前，请先确认已经安装了 ROS2 lyrical luth（推荐的操作系统：ubuntu 26.04 LTS）

### ROS2 功能包位于

```text
laser_scan/
my_interfaces/
```

### 节点

| 节点 | 作用 |
|---|---|
| `stm32` | 串口/蓝牙桥接，接收传感器数据并发布 `/radar/sensor_data`；订阅 `/radar/command` |
| `cmd_controller` | 从 CLI 读取命令并发布到 `/radar/command` |
| `visualizer` | 订阅 `/radar/sensor_data`，滤波、插值后发布 `/radar/laser_scan` |
| `radar_static_tf` | 发布静态 TF，供 RViz2 显示 |

### 编译

```bash
cd ros2_ws
colcon build --packages-select my_interfaces
colcon build --packages-select laser_scan
source install/setup.bash
```

> 假设你的 ROS2 仓库是 `~/ros2_ws`

### 运行

分别打开终端：

```bash
cd ros22_ws
ros2 run laser_scan stm32
ros2 run laser_scan cmd_controller
ros2 run laser_scan visualizer
ros2 run laser_scan radar_static_tf
```

然后在 RViz2 中添加 LaserScan 显示，订阅：

```text
/radar/laser_scan
```

### 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/radar/sensor_data` | `my_interfaces/PolarCoordinates` | 原始极坐标点 |
| `/radar/laser_scan` | `sensor_msgs/LaserScan` | 可供 RViz2 显示的数据 |
| `/radar/command` | `std_msgs/Int32MultiArray` | 上位机指令 |

---

## 指令表

- CLI指令

| 文本指令 | 机器码 | 作用 |
|---|---|---|
| `start` | `0 0` | 解除暂停 |
| `stop` | `0 1` | 暂停 |
| `oled on` | `1 1` | 打开 OLED |
| `oled off` | `1 0` | 关闭 OLED |
| `speed set N` | `2 1 N` | 设置舵机速度 |
| `speed add N` | `2 2 N` | 增加舵机速度 |
| `speed remove N` | `2 3 N` | 减少舵机速度 |
| `cooling_time set N` | `3 1 N` | 设置冷却时间 |
| `cooling_time add N` | `3 2 N` | 增加冷却时间 |
| `cooling_time remove N` | `3 3 N` | 减少冷却时间 |
| `resample N` | `4 N` | 设置重复采样次数 |

- 指令文本

```text
0 0        # 解除暂停
0 1        # 暂停
1 0        # 关闭 OLED
1 1        # 打开 OLED
2 1 X      # 设置舵机速度
3 1 X      # 设置冷却时间
4 X        # 设置重复采样次数
```

> X 必须是正数

### 错误码

| 错误码 | 说明 |
|---|---|
| `0x00` | 参数无效 |
| `0x01` | 参数个数与指令不匹配 |
| `0x02` | 未找到匹配的指令 |

---

## 常见问题

### 1. 蓝牙收到乱码

- 确认蓝牙模块处于 SPP 数据模式；
- 确认 STM32 和 PC 两端波特率一致，当前为 9600；
- 确认帧头（`0XAA`、`0X55`）、帧尾（`0x0D`）与下位机一致；

---

## 许可证

MIT，详见 [LICENSE](LICENSE)。