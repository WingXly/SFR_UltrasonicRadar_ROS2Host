#!/usr/bin/env python3
""" collect data from BLU and publish them on /laser_scan """


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import asyncio
from bleak import BleakClient
import threading
import math
import queue


DEVICE_NAME = "BT24"
DEVICE_ADDRESS = "48:87:2D:D0:AB:0C"
NOTIFY_UUID  = "0000ffe1-0000-1000-8000-00805f9b34fb"
SERVO_SETP_ANGLE = 4.5
MSG_HEADER_FRAME_ID = "laser_scan_frame"

data_queue = queue.Queue()


# ---------- Bluetooth Thread ----------


def ble_callback_when_notify(sender, data):
    """每收到一包 BLE 数据就调用一次"""
    text = data.decode(errors="ignore").strip()
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
    async with BleakClient(DEVICE_ADDRESS) as client:
        print(f"已连接 {client.address}", flush=True)
        await client.start_notify(NOTIFY_UUID, ble_callback_when_notify)
        while client.is_connected:
            await asyncio.sleep(1)


def ble_run():
    asyncio.run(ble_main())


# ---------- ROS2 Node Init ------------


class RadarRepeater(Node):
    def __init__(self):
        super().__init__("radar_repeater")
        self.ranges_ = [float("inf")] * int(1 + 180 / SERVO_SETP_ANGLE)

        self.pub_ = self.create_publisher(LaserScan, "/laser_scan", 10)
        self.tim_ = self.create_timer(0.05, self.publish_scanData)
        self.get_logger().info("radar_repeater created!")


    def publish_scanData(self):
        self.get_data_from_blu()
        msg = self.configure_pubMsg()
        self.pub_.publish(msg)


    def get_data_from_blu(self):
        while not data_queue.empty():
            a, d = data_queue.get_nowait()
            ranges_index = int(a / SERVO_SETP_ANGLE)
            if 0 <= ranges_index < 41:
                self.ranges_[ranges_index] = d / 100.0 # cm to m


    def configure_pubMsg(self):
        msg = LaserScan()
        msg.header.frame_id = MSG_HEADER_FRAME_ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = 0.0
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(SERVO_SETP_ANGLE)
        msg.range_min = 0.02
        msg.range_max = 2.0
        msg.ranges = self.ranges_
        return msg


def main(args=None):
    threading.Thread(target=ble_run, daemon=True).start()
    rclpy.init(args=args)
    node = RadarRepeater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()