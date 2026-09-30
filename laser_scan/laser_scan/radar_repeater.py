import rclpy
from rclpy import Node
from sensor_msgs.msg import LaserScan

import asyncio
from bleak import BleakClient, BleakScanner
import threading
import math
import queue


DEVICE_NAME = "BT24"
NOTIFY_UUID  = "0000ffe1-0000-1000-8000-00805f9b34fb"
SERVO_SETP_ANGLE = 4.5

data_queue = queue.Queue()


# ---------- Bluetooth Thread ----------


def ble_callback_when_notify(sender, data):
    """每收到一包 BLE 数据就调用一次"""
    text = data.decode(errors="ignore").strip()                       # 解码data（将从蓝牙中得到的数据转换为字符串），忽略错误，并去掉开头和结尾的\r\n
    if not text: return

    # 解析 "d=123,a=45"
    try:
        parts = text.split(",")
        d = float(parts[0].split("=")[1])
        a = float(parts[1].split("=")[1])
        data_queue.put((a, d))
    except Exception:
        pass


async def ble_main():
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
        await client.start_notify(NOTIFY_UUID, ble_callback_when_notify)    # 订阅特征 NOTIFY_UUID ，并为这个特征的通知添加回调函数 callback_when_notify
        print("等待数据... (Ctrl+C 退出)")
        while client.is_connected:
            await asyncio.sleep(1)


def ble_run():
    asyncio.run(ble_main())


# ---------- ROS2 Node Init ------------


class RadarRepeater(Node):
    def __init__(self):
        super.__init__("radar_repeater")

        self.pub_ = self.create_publisher(
            LaserScan, "/laser_scan",
            10
        )

        self.tim_ = self.create_timer(
            0.05,
            self.publish_scanData
        )

        self.ranges_ = [float("inf")] * int(1 + 180 / SERVO_SETP_ANGLE)

        self.get_logger().info("radar_repeater created!")


    def publish_scanData(self):
        self.get_data_from_blu()
        msg = LaserScan()
        msg.angle_min = 0.0
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(SERVO_SETP_ANGLE)
        msg.range_min = 0.02
        msg.range_max = 2.0
        msg.ranges = self.ranges_
        self.pub_.publish(msg)


    def get_data_from_blu(self):
        while not data_queue.empty():
            a, d = data_queue.get_nowait()
            ranges_index = int(a / SERVO_SETP_ANGLE)
            if 0 <= ranges_index < 41:
                self.ranges_[ranges_index] = d # cm


def main(args=None):
    threading.Thread(target=ble_run, daemon=True).start()
    rclpy.init(args=args)
    node = RadarRepeater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()