import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from my_interfaces.msg import PolarCoordinates
import socket
import threading
import queue
import select


# ============ 配置 ============
BT_ADDR = "1D:06:44:03:24:CD"
BT_PORT = 1
RECONNECT_DELAY = 2.0        # 断线后重连间隔（秒）
INIT_CMD = (0, 0)            # 下位机上电默认暂停，连上后必须发这条才会推数据
FRAME_HEAD_1 = 0xAA          # 帧头1
FRAME_HEAD_2 = 0x55          # 帧头2
FRAME_TAIL = 0x0D            # 帧尾


"""
将距离和角度通过串口发送至电脑。采用二进制帧结构发布通知：

uint8_t sendMsg[9]:
帧头         长度    角度     距离        校验      帧尾
0xAA 0x55    N      int16    uint16      XOR      0x0D
0    1       2      3   4    5    6      7        8

角度单位：度 -> 0.01度
距离单位：mm -> mm
"""


# ============ 协议解析器 ============
class RadarParser:
    def __init__(self):
        self.state = 'H1'
        self.frame = bytearray()
        self.expected = 0

    def feed(self, data: bytes):
        out = []
        for this_byte in data:
            if self.state == 'H1':
                if this_byte == FRAME_HEAD_1:
                    self.frame = bytearray([this_byte])
                    self.state = 'H2'

            elif self.state == 'H2':
                if this_byte == FRAME_HEAD_2:
                    self.frame.append(this_byte)
                    self.state = 'LEN'
                else:
                    self.state = 'H1'

            elif self.state == 'LEN':
                self.frame.append(this_byte)
                self.expected = this_byte
                self.state = 'PAYLOAD'

            elif self.state == 'PAYLOAD':
                self.frame.append(this_byte)
                if len(self.frame) == 2 + 1 + self.expected:
                    payload = self.frame[2:-1]
                    cs = 0
                    for x in payload:
                        cs ^= x
                    if cs == self.frame[-1]:
                        angle = (self.frame[3] << 8) | self.frame[4]
                        dist  = (self.frame[5] << 8) | self.frame[6]
                        out.append( (angle/100.0, dist/1000.0) )
                    self.state = 'H1'
        return out


# ============ ROS2 节点 ============
class STM32(Node):
    def __init__(self):
        super().__init__('stm32')

        self.scan_pub_ = self.create_publisher(PolarCoordinates, '/radar/sensor_data', 10)
        self.cmd_sub_  = self.create_subscription(Int32MultiArray, '/radar/command', self.cmd_callback, 10)
        self.timer_ = self.create_timer(0.01, self.receive_and_publish_data) # 每隔0.01s进行一次轮询接收，同时将获取的数据发布到/radar/sensor_data

        self.rx_queue_ = queue.Queue()
        self.tx_queue_ = queue.Queue()

        self.parser_ = RadarParser()

        self.RUNNING_ = True
        self._stop_event_ = threading.Event()   # 用 Event 代替 sleep，退出更干脆
        self.sock_ = None                       # 当前 socket（供 destroy_node 打断阻塞）
        self._sock_lock_ = threading.Lock()
        self._CONNECTED_ = False

        self.bt_thread_ = threading.Thread(target=self.bt_worker, daemon=True)
        self.bt_thread_.start()

        self.get_logger().info('STM32节点已启动！')


    # ---------- 主线程：轮询接收 ----------
    def receive_and_publish_data(self):
        while not self.rx_queue_.empty():
            data = self.rx_queue_.get_nowait()
            for angle, dist in self.parser_.feed(data):
                msg = PolarCoordinates()
                msg.angle = float(angle)
                msg.radius = float(dist)
                self.scan_pub_.publish(msg)


    # ---------- 主线程：命令回调 ----------
    def cmd_callback(self, msg: Int32MultiArray):
        if len(msg.data) == 2:
            cmd, param = int(msg.data[0]), int(msg.data[1])
            self.tx_queue_.put(self.encode_command_p1(cmd, param))
            self.get_logger().info(f'cmd {cmd} {param}')

        elif len(msg.data) == 3:
            cmd, param1, param2 = int(msg.data[0]), int(msg.data[1]), int(msg.data[2])
            self.tx_queue_.put(self.encode_command_p2(cmd, param1, param2))
            self.get_logger().info(f'cmd {cmd} {param1} {param2}')


    @staticmethod
    def encode_command_p1(cmd: int, param: int) -> bytes:
        buf = bytearray(
            [FRAME_HEAD_1, FRAME_HEAD_2, 0x05, (cmd>>8)&0xFF, cmd&0xFF, (param>>8)&0xFF, param&0xFF, 0, FRAME_TAIL]
        )
        buf[-2] = buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6]
        return bytes(buf)


    @staticmethod
    def encode_command_p2(cmd: int, param1: int, param2: int) -> bytes:
        buf = bytearray(
            [FRAME_HEAD_1, FRAME_HEAD_2, 0x07, (cmd>>8)&0xFF, cmd&0xFF, (param1>>8)&0xFF, param1&0xFF, (param2>>8)&0xFF, param2&0xFF, 0, FRAME_TAIL]
        )
        buf[-2] = buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6] ^ buf[7] ^ buf[8]
        return bytes(buf)


    # ---------- 蓝牙线程：断线自动重连 ----------
    def _open_socket(self):
        """建立一条 RFCOMM 连接，返回已就绪的 socket（失败则抛异常）。"""
        sock = socket.socket(
            socket.AF_BLUETOOTH,
            socket.SOCK_STREAM,
            socket.BTPROTO_RFCOMM,
        )
        sock.settimeout(10.0)               # 建链阶段给足时间
        try:
            sock.connect((BT_ADDR, BT_PORT))
        except Exception:
            sock.close()
            raise
        sock.settimeout(None)               # 收发阶段回到阻塞模式，等待交给 select
        return sock


    def _send_frame(self, sock, frame: bytes) -> bool:
        """发送一帧。返回 False 表示链路已断，调用方应结束 _io_loop 去重连。"""
        try:
            with self._sock_lock_:
                sock.sendall(frame)
            self.get_logger().info(f'已发送: {frame.hex(" ")}')
            return True
        except OSError as e:
            self.get_logger().error(f'发送失败: {e}')
            return False


    def _ask_command(self, sock, cmd):
        if len(cmd) == 2:
            frame = self.encode_command_p1(*cmd)
        elif len(cmd) == 3:
            frame = self.encode_command_p2(*cmd)

        self._send_frame(sock, frame)


    def _receive_data(self, sock):
        """返回收到的字节。

        返回 None 表示**链路已断**，调用方必须据此结束 _io_loop 去重连：
          - recv 抛 OSError，其中 errno 110 (ETIMEDOUT) 就是下位机
            把空闲链路拆掉时内核上报的错误；
          - recv 返回 b''，表示对端正常关闭。
        这两种情况都不能被当成「这次没数据」而忽略掉。
        """
        try:
            data = sock.recv(1024)
        except OSError as e:
            self.get_logger().error(f'蓝牙接收失败: {e}')
            return None
        if not data:
            self.get_logger().warning('对端已关闭连接')
            return None
        return data


    def bt_worker(self):
        """蓝牙线程：断线自动重连；每次连上后自动解除下位机暂停。"""

        while not self._stop_event_.is_set():

            # ---------- 1) 建链 ----------
            try:
                sock = self._open_socket()
            except Exception as e:
                # 连接失败，持续重试
                self.get_logger().warning(
                    f'蓝牙连接失败: {e}。将在{RECONNECT_DELAY}s后尝试重连',
                    throttle_duration_sec=10.0
                )
                self._stop_event_.wait(RECONNECT_DELAY)
                continue

            with self._sock_lock_:
                self.sock_ = sock

            self._CONNECTED_ = True
            self.get_logger().info('蓝牙已连接')

            # ---------- 2) 立刻解除下位机暂停 ----------
            self._ask_command(sock, INIT_CMD)

            # ---------- 3) 收发循环（返回即代表链路已断）----------
            self._io_loop(sock)

            # ---------- 4) 收尾并准备重连 ----------
            self._CONNECTED_ = False
            
            with self._sock_lock_:
                if self.sock_ == sock:
                    self.sock_ = None

            try: sock.close()
            except: pass

            if not self._stop_event_.is_set():
                self.get_logger().warning(f'断开连接。将在{RECONNECT_DELAY}s后尝试重连')
                self._stop_event_.wait(RECONNECT_DELAY)

        self.get_logger().info('蓝牙已停止工作')


    def _io_loop(self, sock):
        """收发主循环。工作时会一直卡在循环中。返回则表示链路已断"""

        while not self._stop_event_.is_set():

            # --- 等待可读（10ms 一轮，兼顾命令发送的实时性）---
            try:
                ready, _, _ = select.select([sock], [], [], 0.01)
            except (OSError, ValueError) as e:
                self.get_logger().error(f'蓝牙错误: {e}')
                return

            # --- 从蓝牙上接收数据并放进rx队列中 ---
            if ready:
                data = self._receive_data(sock)
                if data is None:
                    return                      # 链路已断 -> 交给 bt_worker 重连
                self.rx_queue_.put(data)

            # --- 将指令从tx队列中取出并全部发送 ---
            while not self.tx_queue_.empty():
                frame = self.tx_queue_.get_nowait()
                if not self._send_frame(sock, frame):
                    self.tx_queue_.put(frame)   # 没发出去的命令退回，重连后补发
                    return                      # 链路已断 -> 交给 bt_worker 重连


    def destroy_node(self):
        self.RUNNING_ = False
        self._stop_event_.set()

        # 主动关掉 socket，打断可能正在阻塞的 recv
        with self._sock_lock_:
            sock, self.sock_ = self.sock_, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

        if self.bt_thread_.is_alive():
            self.bt_thread_.join(timeout=2.0)

        super().destroy_node()


def main():
    rclpy.init()
    node = STM32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
