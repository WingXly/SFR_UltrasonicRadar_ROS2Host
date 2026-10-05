import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from my_interfaces.msg import PolarCoordinates
import socket
import threading
import queue


# ============ 配置 ============
BT_ADDR = "1D:06:44:03:24:CD"
BT_PORT = 1


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
                if this_byte == 0xAA:
                    self.frame = bytearray([this_byte])
                    self.state = 'H2'

            elif self.state == 'H2':
                if this_byte == 0x55:
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
                        out.append( (angle/100.0, dist) )
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

        self.running_ = True
        self.bt_thread_ = threading.Thread(target=self.bt_worker, daemon=True)
        self.bt_thread_.start()

        self.get_logger().info('STM32 started')


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
            [0xAA, 0x55, 0x05, (cmd>>8)&0xFF, cmd&0xFF, (param>>8)&0xFF, param&0xFF, 0, 0xD0]
        )
        buf[-2] = buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6]
        return bytes(buf)


    @staticmethod
    def encode_command_p2(cmd: int, param1: int, param2: int) -> bytes:
        buf = bytearray(
            [0xAA, 0x55, 0x07, (cmd>>8)&0xFF, cmd&0xFF, (param1>>8)&0xFF, param1&0xFF, (param2>>8)&0xFF, param2&0xFF, 0, 0xD0]
        )
        buf[-2] = buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6] ^ buf[7] ^ buf[8]
        return bytes(buf)


    # ---------- 蓝牙线程 ----------
    def bt_worker(self):
        try:
            sock = socket.socket(
                socket.AF_BLUETOOTH,
                socket.SOCK_STREAM,
                socket.BTPROTO_RFCOMM
            )
            sock.connect((BT_ADDR, BT_PORT))
            sock.settimeout(0.01)   # 10ms 超时，让收发可以交替
            self.get_logger().info('BT connected')

            while self.running_:
                # --- 接收 ---
                try:
                    data = sock.recv(1024)
                    if not data:
                        self.get_logger().warn('BT closed by peer')
                        break
                    self.rx_queue_.put(data)
                except socket.timeout:
                    pass
                except Exception as e:
                    self.get_logger().error(f'BT recv error: {e}')
                    break

                # --- 发送 ---
                if not self.tx_queue_.empty():
                    try:
                        frame = self.tx_queue_.get_nowait()
                        sock.sendall(frame)
                    except Exception as e:
                        self.get_logger().error(f'BT send error: {e}')
                        break

            sock.close()

        except Exception as e:
            self.get_logger().error(f'BT thread died: {e}')


    def destroy_node(self):
        self.running_ = False
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
