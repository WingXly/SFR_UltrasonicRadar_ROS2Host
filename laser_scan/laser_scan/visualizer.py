import math
import queue
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from my_interfaces.msg import PolarCoordinates

DEFAULT = float("inf")


# ============ 配置 ============
INPUT_TOPIC = "/radar/sensor_data"
OUTPUT_TOPIC = "/radar/laser_scan"

# 发布到
FRAME_ID = "laser_scan_frame"

# 发布周期（秒）
PUBLISH_PERIOD = 0.2

# 实测扫描范围
ANGLE_MIN_DEG = 0.0
ANGLE_MAX_DEG = 180.0

# 测量半径
RANGE_MIN = 0.02
RANGE_MAX = 6.0

# 角度格分辨率（度）
RESOLUTION_DEG = 0.45

# 允许插值的最大空洞
MAX_GAP = 5


class PostProcessor:
    def __init__(self):
        self.this_angle = 0
        self.previous_angle = 0

        # 初始化坐标表格，分辨率：RESOLUTION_DEG
        self.range = [DEFAULT] * round(180 / RESOLUTION_DEG + 1)


    def write(self, r, theta):
        '''向self.range中写入一个点'''
        index = round(theta / RESOLUTION_DEG)
        if 0 <= index < len(self.range):
            self.range[index] = r
        

    def clear_area(self, from_angle, to_angle):
        from_index = round(from_angle / RESOLUTION_DEG)
        to_index = round(to_angle / RESOLUTION_DEG)

        left = min(from_index, to_index)
        right = max(from_index, to_index)

        # 开区间内所有数值改为 DEFAULT
        for index in range(left + 1, right):
            if 0 <= index < len(self.range):
                self.range[index] = DEFAULT


    def feed_all(self, dists: queue.Queue, angle: float):
        if dists.empty(): return

        # 滤波
        dist = self.any_filter(dists, 'average')

        # 更新self数据
        self.previous_angle = self.this_angle
        self.this_angle     = angle

        # 更新坐标图
        self.clear_area(self.previous_angle, self.this_angle)
        self.write(dist, angle)
        self.interpolation_auto()


    @staticmethod
    def any_filter(new_values: queue.Queue, algorithm: str) -> float:
        values = []

        while not new_values.empty():
            temp = new_values.get_nowait()
            if temp is None or temp < 0: continue
            values.append(temp)

        # 如果只有一个值，就不需要滤波
        if not values:
            return 0.0
        elif len(values) == 1:
            return values[0]

        # 按要求选择滤波算法
        if algorithm.lower() == 'average':
            return sum(values) / len(values)
        elif algorithm.lower() == 'mediam':
            values.sort()
            return values[len(values) // 2]
        else:
            pass


    def interpolation_auto(self):
        '''将self.range里的float("inf")拟合为其他数字，采用线性插值'''
        # 有效值索引表格
        valid_index_list = [index for (index, value) in enumerate(self.range) if value is not DEFAULT]
        if len(valid_index_list) <= 1: return

        for i in range(len(valid_index_list) - 1):
            # 按左右有效值定义一个空洞
            left_valid_index = valid_index_list[i]
            right_valid_index = valid_index_list[i + 1]

            # 只填补中间的小空洞；大空洞保持 DEFAULT，避免乱连
            inserts = right_valid_index - left_valid_index - 1
            if inserts <= 0 or inserts > MAX_GAP:
                continue

            # 在空洞里逐个插值
            for index in range(left_valid_index + 1, right_valid_index):
                duty = (index - left_valid_index) / (right_valid_index - left_valid_index)
                self.range[index] = self.range[left_valid_index] * (1.0 - duty) + self.range[right_valid_index] * duty


class RadarVisualizer(Node):
    def __init__(self):
        super().__init__("radar_visualizer")

        self.previous_angle = None

        self.m_queue_feed_processor = queue.Queue()

        self.m_processor = PostProcessor()

        self.m_sub = self.create_subscription(PolarCoordinates, INPUT_TOPIC,
                                              self.callback_subscription, 10)
        
        self.m_tim = self.create_timer(PUBLISH_PERIOD,
                                       self.callback_timer)
        
        self.m_pub = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)

        self.get_logger().info(f"可视化节点已启动: {INPUT_TOPIC} -> {OUTPUT_TOPIC}, "
                               f"每 {PUBLISH_PERIOD}s 发布一帧")


    def publish_laser_scan(self, ranges: list):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID

        msg.angle_min = math.radians(ANGLE_MIN_DEG)
        msg.angle_max = math.radians(ANGLE_MAX_DEG)
        msg.angle_increment = math.radians(RESOLUTION_DEG)

        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX

        msg.ranges = ranges

        self.m_pub.publish(msg)


    def callback_subscription(self, msg: PolarCoordinates):
        # 如果是第一次采样
        if self.previous_angle is None:
            previous_angle = msg.angle_degree

        # 如果角度变化，说明重复采样结束。开始后处理
        elif msg.angle_degree != previous_angle:
            previous_angle = msg.angle_degree
            self.m_processor.feed_all(self.m_queue_feed_processor, msg.angle_degree)

        self.m_queue_feed_processor.put(msg.dist_m)


    def callback_timer(self):
        self.publish_laser_scan(self.m_processor.range)


def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
