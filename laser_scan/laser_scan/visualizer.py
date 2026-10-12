#!/usr/bin/env python3
"""把 /radar/sensor_data 的极坐标点聚合为 LaserScan，发布到 /radar/laser_scan。

数据链路：
    stm32 节点 --(PolarCoordinates)--> 本节点 --(LaserScan)--> RViz2

单位说明（已实测确认，见 stm32.py 的 RadarParser）：
    PolarCoordinates.angle  = 度
    PolarCoordinates.radius = 米（stm32.py 里已做 mm -> m）
所以这里只需要把角度转弧度，距离直接可用 —— 不要再除以 1000。

角度分辨率是**自动探测**的，不再硬编码 1.8°：
用「当前角度 − 上一个角度」推断步进，详见 _update_angle_step()。
步进一旦确定，self.ranges_ 的长度和下标都由它算出来。
"""

import math
from collections import Counter, deque

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

POINT_RATE_HZ = 23.0                # 实测点速率，仅用于填 scan_time

RANGE_MIN = 0.02                    # 米
RANGE_MAX = 6.0                     # 米

# ---- 角度步进自动探测 ----
MAX_PLAUSIBLE_STEP_DEG = 90.0       # 差值超过它视为掉头/大跳变，不参与探测
ANGLE_STEP_QUANT = 0.1              # 差值量化精度：抹掉 1.79/1.80/1.81 这类抖动
DELTA_WINDOW = 120                  # 只在最近这么多个差值里统计
MIN_STEP_CONFIRM = 3                # 一个差值至少出现这么多次才被采信

# 每次发布后是否清空缓存，两种模式观感差别很大：
#   False（默认）累积：每个角度保留最近一次读数。走完一整圈后填满，
#                     RViz 里呈现稳定的扇形，之后随扫描持续刷新。
#   True  只发本周期：每条消息仅含最近 0.2s 收到的点（约 5 个），
#                     RViz 里是一条跟着伺服转动的短弧。
CLEAR_AFTER_PUBLISH = False


class RadarVisualizer(Node):
    def __init__(self):
        super().__init__("radar_visualizer")

        # ---- 自动探测步进所需的状态 ----
        self._prev_angle_ = None        # 上一个角度（度）
        self._step_deg_ = None          # 探测到的步进（度）；None = 尚未测出
        self._delta_window_ = deque(maxlen=DELTA_WINDOW)   # 最近若干个差值
        self.ranges_ = None             # 探测到步进后才分配
        self._bin_count_ = 0
        self._warned_waiting_ = False

        self.sub_ = self.create_subscription(
            PolarCoordinates, INPUT_TOPIC, self.callback_when_subscription, 10)
        self.tim_ = self.create_timer(PUBLISH_PERIOD, self.callback_timer)
        self.pub_ = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)

        self.get_logger().info(
            f"可视化节点已启动: {INPUT_TOPIC} -> {OUTPUT_TOPIC}, "
            f"角度步进自动探测中, 每 {PUBLISH_PERIOD}s 发布一次"
        )


    # ---------- 角度步进探测 ----------
    def _update_angle_step(self, angle: float):
        """用相邻角度的差值推断步进；步进变化时按新步进重建缓存。

        判定规则（两步，缺一不可）：
          1) **先确认再采信**：一个差值要在最近 DELTA_WINDOW 个差值里出现
             MIN_STEP_CONFIRM 次以上，才算候选。这一步专门用来挡掉偶发的
             半格差值 —— 实测中 1.8° 的扫描里偶尔会蹦出一个 0.9°，
             如果直接"取最小值"，步进就会被它带偏成 0.9°，
             结果 201 个格子里只有一半能拿到数据。
          2) **在候选里取最小**：丢帧会让差值变成步进的整数倍（2×、3×…），
             只会**变大**，所以确认过的最小值就是真实步进。
             伺服掉头时差值约等于整个扫描范围，用 MAX_PLAUSIBLE_STEP_DEG 排除。
          另外差值先按 ANGLE_STEP_QUANT 量化，抹掉设备 0.01° 量化带来的抖动。
        """
        prev, self._prev_angle_ = self._prev_angle_, angle
        if prev is None:
            return

        delta = abs(angle - prev)
        if not (0.0 < delta <= MAX_PLAUSIBLE_STEP_DEG):
            return                                  # 掉头 / 大跳变，忽略

        candidate = round(delta, 1)                 # 量化掉 ±0.01 级抖动
        if candidate <= 0.0:
            return

        self._delta_window_.append(candidate)

        counts = Counter(self._delta_window_)
        confirmed = [v for v, c in counts.items() if c >= MIN_STEP_CONFIRM]
        if not confirmed:
            return                                  # 还没有任何差值被确认

        best = min(confirmed)
        if best == self._step_deg_:
            return                                  # 没有变化

        self._step_deg_ = best
        self._rebin()


    def _rebin(self):
        """按当前步进重新分配角度格。"""
        span = ANGLE_MAX_DEG - ANGLE_MIN_DEG
        self._bin_count_ = int(round(span / self._step_deg_)) + 1
        self.ranges_ = [float("inf")] * self._bin_count_
        self.get_logger().info(
            f"自动测得角度步进 = {self._step_deg_}°, "
            f"重建 {self._bin_count_} 个角度格"
        )


    # ---------- 订阅回调：把一个极坐标点放进对应角度格 ----------
    def callback_when_subscription(self, msg: PolarCoordinates):
        # 注意：回调签名必须带 msg 参数，否则一收到消息就会 TypeError

        angle = float(msg.angle)
        self._update_angle_step(angle)              # 内部会在步进变化时重建缓存

        if self._step_deg_ is None:
            return                                  # 步进还没测出（只损失 1~2 个点）

        idx = int(round((angle - ANGLE_MIN_DEG) / self._step_deg_))

        if not (0 <= idx < self._bin_count_):
            return                                  # 超出扫描范围，丢弃

        radius = float(msg.radius)

        # 越界或非有限值一律记为 inf，避免 RViz 画出假的点
        if not math.isfinite(radius) or not (RANGE_MIN <= radius <= RANGE_MAX):
            self.ranges_[idx] = float("inf")
        else:
            self.ranges_[idx] = radius


    # ---------- 定时器：把缓存组装成 LaserScan 发布 ----------
    def callback_timer(self):
        if self._step_deg_ is None or self.ranges_ is None:
            if not self._warned_waiting_:
                self._warned_waiting_ = True
                self.get_logger().warning(
                    f"还在等待足够的点来推算角度步进；"
                    f"请确认 {INPUT_TOPIC} 有数据"
                )
            return

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID      # 没有它 RViz2 什么都画不出来

        msg.angle_min = math.radians(ANGLE_MIN_DEG)
        msg.angle_increment = math.radians(self._step_deg_)
        # 由格数反推 angle_max，保证 (max-min)/inc + 1 == len(ranges)。
        # 若硬写 180°，当步进不是它的整数倍时 RViz 会按错误点数渲染。
        msg.angle_max = msg.angle_min + (self._bin_count_ - 1) * msg.angle_increment

        msg.time_increment = 1.0 / POINT_RATE_HZ
        msg.scan_time = (PUBLISH_PERIOD if CLEAR_AFTER_PUBLISH
                         else self._bin_count_ / POINT_RATE_HZ)

        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX

        msg.ranges = list(self.ranges_)
        msg.intensities = []

        self.pub_.publish(msg)

        if CLEAR_AFTER_PUBLISH:
            self.ranges_ = [float("inf")] * self._bin_count_


def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
