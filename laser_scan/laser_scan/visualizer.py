#!/usr/bin/env python3
"""把 /radar/sensor_data 的极坐标点聚合为 LaserScan，发布到 /radar/laser_scan。

数据链路：
    stm32 节点 --(PolarCoordinates)--> 本节点 --(LaserScan)--> RViz2

单位说明（已实测确认，见 stm32.py 的 RadarParser）：
    PolarCoordinates.angle  = 度
    PolarCoordinates.radius = 米（stm32.py 里已做 mm -> m）
所以这里只需要把角度转弧度，距离直接可用 —— 不要再除以 1000。

角度配置来自实测：该雷达扫 0~180°，步进 1.8°（101 个唯一角度），
约 23 点/秒，即走完一整圈约 4.4 秒。1.8° 与 101 个格子正好对齐。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from my_interfaces.msg import PolarCoordinates


# ============ 配置 ============
INPUT_TOPIC = "/radar/sensor_data"
OUTPUT_TOPIC = "/radar/laser_scan"

# 必须与 radar_staticTFto_map.py 发布的子坐标系一致（map -> laser_scan_frame）
FRAME_ID = "laser_scan_frame"

PUBLISH_PERIOD = 0.2                # 发布周期（秒）

ANGLE_MIN_DEG = 0.0                 # 实测扫描范围
ANGLE_MAX_DEG = 180.0

ANGLE_STEP_DEG = 1.8                # 实测步进

POINT_RATE_HZ = 23.0                # 实测点速率，仅用于填 time_increment / scan_time

RANGE_MIN = 0.02                    # 米
RANGE_MAX = 6.0                     # 米

# 每次发布后是否清空缓存，两种模式观感差别很大：
#   False（默认）累积：每个角度保留最近一次读数。约 4.4s（一整圈）后填满，
#                     RViz 里呈现稳定的 180° 扇形，之后随扫描持续刷新。
#   True  只发本周期：每条消息仅含最近 0.2s 收到的点（约 5 个），
#                     RViz 里是一条跟着伺服转动的短弧。
CLEAR_AFTER_PUBLISH = False

# 由上面的配置推导，不要单独改
BIN_COUNT = int(round((ANGLE_MAX_DEG - ANGLE_MIN_DEG) / ANGLE_STEP_DEG)) + 1
ANGLE_INCREMENT = math.radians(ANGLE_STEP_DEG)


class RadarVisualizer(Node):
    def __init__(self):
        super().__init__("radar_visualizer")

        # 每个角度格对应一个距离，初始为 inf（= 该方向暂无有效读数）
        self.ranges_ = [float("inf")] * BIN_COUNT

        self.sub_ = self.create_subscription(PolarCoordinates, INPUT_TOPIC, self.callback_when_subscription, 10)
        self.tim_ = self.create_timer(PUBLISH_PERIOD, self.callback_timer)
        self.pub_ = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)

        self.get_logger().info(
            f"可视化节点已启动: {INPUT_TOPIC} -> {OUTPUT_TOPIC}, "
            f"{BIN_COUNT} 个角度格 ({ANGLE_STEP_DEG}°/格), "
            f"每 {PUBLISH_PERIOD}s 发布一次"
        )


    # ---------- 订阅回调：把一个极坐标点放进对应角度格 ----------
    def callback_when_subscription(self, msg: PolarCoordinates):
        # 注意：回调签名必须带 msg 参数，否则一收到消息就会 TypeError

        idx = int(round((msg.angle - ANGLE_MIN_DEG) / ANGLE_STEP_DEG))

        if not (0 <= idx < BIN_COUNT):
            return                      # 超出扫描范围，丢弃

        radius = float(msg.radius)

        # 越界或非有限值一律记为 inf，避免 RViz 画出假的点
        if not math.isfinite(radius) or not (RANGE_MIN <= radius <= RANGE_MAX):
            self.ranges_[idx] = float("inf")
        else:
            self.ranges_[idx] = radius


    # ---------- 定时器：把缓存组装成 LaserScan 发布 ----------
    def callback_timer(self):
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID      # 没有它 RViz2 什么都画不出来

        msg.angle_min = math.radians(ANGLE_MIN_DEG)
        # 与 ranges 长度自洽: angle_max = angle_min + (len-1) * increment
        msg.angle_max = math.radians(ANGLE_MAX_DEG)
        msg.angle_increment = ANGLE_INCREMENT

        msg.time_increment = 1.0 / POINT_RATE_HZ
        msg.scan_time = (PUBLISH_PERIOD if CLEAR_AFTER_PUBLISH
                         else BIN_COUNT / POINT_RATE_HZ)

        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX

        msg.ranges = list(self.ranges_)
        msg.intensities = []

        self.pub_.publish(msg)

        if CLEAR_AFTER_PUBLISH:
            self.ranges_ = [float("inf")] * BIN_COUNT


def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
