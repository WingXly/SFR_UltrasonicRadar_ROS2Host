#!/usr/bin/env python3
""" collect data from BLU and publish them on /laser_scan """


import rclpy
from rclpy.node import Node
from my_interfaces.msg import PolarCoordinates

import select
import socket
import struct
import threading


# ---------------------------------------------------------------- 连接参数
ADDR = "1D:06:44:03:24:CD"      # 用 sdptool search --bdaddr <addr> SP 可确认
CHANNEL = 1                     # SDP 里查到的 RFCOMM 通道号（服务名 JL_SPP）
CONNECT_TIMEOUT = 10.0          # 建链阶段超时
POLL_INTERVAL = 0.1             # 读轮询间隔（秒）



class FrameProcessor:
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


    def on_spp_bytes(self, data: bytes):
            angles = []
            distances = []
            # 从字节流里拆出完整帧，再交给解析函数
            for frame in self.feed(data):
                a, d = self.analyse_frame(frame)
                angles.append(a)
                distances.append(d)
            return [angles, distances]


    def check_frame(self, frame):
        if len(frame) < 9:
            return False

        # 检查帧头和帧尾
        if not (frame[0] == 0xAA and frame[1] == 0x55 and frame[8] == 0x0D):
            return False

        # 检查校验和
        checksum = frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]
        if checksum != frame[7]:
            return False
        
        return True


    """
uint8_t sendMsg[9]:
帧头         长度    角度      距离        校验      帧尾
0xAA 0x55    N     int16    uint16      XOR      0x0D
0    1       2     3   4    5    6      7        8

角度单位: 0.01度
距离单位: mm
    """
    def analyse_frame(self, frame):
        if not self.check_frame(frame): return

        angle_row = (frame[3] << 8) + frame[4]
        distance_row = (frame[5] << 8) + frame[6]

        # 转换单位
        angle = float(angle_row) / 100.0        # 0.01 度 -> 度
        distance = float(distance_row) / 1000.0 # mm -> m

        return [angle, distance]


# ---------------------------------------------------------------- 子线程1（蓝牙接收数据）


class SPPReceiver:
    """
    JDP-31-SPP 蓝牙串口接收 / 发送器（RFCOMM socket 实现）

    - 地址 + 通道，直连；无需配对后创建的 /dev/rfcomm0
    - 后台线程持续接收，断线自动重连
    - send_u16_command() 向上位机 -> 下位机发送命令
    """

    def __init__(self, address=ADDR, channel=CHANNEL, on_bytes=None):
        self.address = address
        self.channel = int(channel)
        self.on_bytes = on_bytes

        self.sock = None
        self._stop_event = threading.Event()
        self._thread = None
        self._write_lock = threading.Lock()

    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self.sock is not None

    def _start_in_subProcess(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _stop(self):
        self._stop_event.set()
        self._drop()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    def _connect(self):
        """建立 RFCOMM 链路"""
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                             socket.BTPROTO_RFCOMM)
        sock.settimeout(CONNECT_TIMEOUT)
        try:
            sock.connect((self.address, self.channel))
        except Exception:
            sock.close()
            raise

        # 建链后改回非阻塞轮询：不用 settimeout，读超时交给 select 处理
        sock.setblocking(False)
        self.sock = sock
        print(f"[SPP] 已连接 {self.address} 通道 {self.channel}", flush=True)


    def _drop(self):
        """释放当前 socket（可安全重复调用）。"""
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    # ------------------------------------------------------------------
    @staticmethod
    def _build_u16_frame(cmd_type: int, param1: int, param2: int = None) -> bytes:
        """
        按下位机协议构造命令帧。

        单参数命令:
            [AA 55 05] [cmd_type_hi cmd_type_lo] [param_hi param_lo] [checksum] [0D]
        双参数命令:
            [AA 55 07] [cmd_type_hi cmd_type_lo]
                       [param1_hi param1_lo]
                       [param2_hi param2_lo]
                       [checksum] [0D]
        """
        cmd_type &= 0xFFFF
        param1 &= 0xFFFF

        if param2 is None:
            n = 5
            payload = struct.pack(">HH", cmd_type, param1)
        else:
            param2 &= 0xFFFF
            n = 7
            payload = struct.pack(">HHH", cmd_type, param1, param2)

        frame = bytearray([0xAA, 0x55, n])
        frame.extend(payload)

        checksum = 0
        for b in frame[2:2 + n]:
            checksum ^= b

        frame.append(checksum & 0xFF)
        frame.append(0x0D)
        return bytes(frame)

    def send_u16_command(self, cmd_type: int, param1: int, param2: int = None) -> bool:
        if self.sock is None:
            return False

        frame = self._build_u16_frame(cmd_type, param1, param2)
        try:
            with self._write_lock:
                self.sock.sendall(frame)
            return True
        except Exception as e:
            print("[SPP] 发送失败:", e)
            return False

    # ------------------------------------------------------------------
    def _run(self):
        while not self._stop_event.is_set():
            # 1) 确保链路存在
            if self.sock is None:
                try:
                    self._connect()
                except Exception as e:
                    print("[SPP] 连接失败:", e, flush=True)
                    # 用 Event.wait 代替 sleep：被 stop() 叫醒时能立刻退出
                    self._stop_event.wait(2.0)
                    continue

            # 取本地引用：stop() 可能在任何时刻把 self.sock 置空，
            # 直接用 self.sock 会在 select 与 recv 之间踩到竞态（NoneType）。
            sock = self.sock
            if sock is None:
                continue

            # 2) select 等可读，避免依赖 socket 超时异常
            try:
                ready, _, _ = select.select([sock], [], [], POLL_INTERVAL)
            except (OSError, ValueError):
                self._drop()
                continue
            if not ready:
                continue

            # 3) 读数据
            try:
                data = sock.recv(4096)
            except BlockingIOError:
                continue
            except OSError as e:
                if self._stop_event.is_set():
                    break
                print("[SPP] 读取异常，准备重连:", e, flush=True)
                self._drop()
                continue

            # recv 返回空字节 = 对端已关闭连接（串口不会出现这种情况）
            if not data:
                if self._stop_event.is_set():
                    break
                print("[SPP] 对端断开，准备重连", flush=True)
                self._drop()
                continue

            if self.on_bytes:
                self.on_bytes(data)


    def wait_for_user_cmd(self):
        command = input("> ").strip()
        if not command: return
        # 例:
        # cmd 0 0       单参数命令, 解除暂停
        # cmd 0 1       单参数命令, 暂停
        # cmd 1 1       单参数命令, 开 OLED
        # cmd 2 1 5     双参数命令
        parts = command.split()
        return parts


    def analyze_and_send_cmd(self, parts):
        if parts[0].lower() == "cmd":
            try:
                nums = [int(x, 0) for x in parts[1:]]
            except ValueError:
                print("格式错误！")
                return

            if len(nums) == 2:
                ok = self.send_u16_command(nums[0], nums[1])
            elif len(nums) == 3:
                ok = self.send_u16_command(nums[0], nums[1], nums[2])
            else:
                print("格式错误，例如: cmd 0 0 或 cmd 2 1 5")
                return

            if ok:
                print(f"[PC -> STM32] 已发送: {nums}")
            else:
                print("[PC -> STM32] 发送失败，未连接")
        else:
            print("未知命令")


    def conduct_cmd_auto(self):
        self.analyze_and_send_cmd( self.wait_for_user_cmd() )


# ---------------------------------------------------------------- 主线程（ROS2线程）（蓝牙发送数据）


class STM32(Node):
    def __init__(self):
        super().__init__("stm32")
        self.ranges_ = [float("inf")] * int(1 + 180 / SERVO_SETP_ANGLE)

        self.pub_ = self.create_publisher(PolarCoordinates, "/sensor_data", 10)
        self.tim_ = self.create_timer(0.05, self.get_and_publish_scanData)
        self.get_logger().info("STM32 node created!")

        self.bt_processor = FrameProcessor()
        self.bt_receiver = SPPReceiver(
                address = ADDR,
                channel = CHANNEL,
                on_bytes = self.bt_processor.on_spp_bytes,
            )
        self.bt_receiver._start_in_subProcess()
        self.get_logger().info("Bluetooth Processor ready!")


    def get_and_publish_scanData(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = STM32()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()