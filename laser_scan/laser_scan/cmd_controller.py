#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

TOPIC = '/radar/command'
PUB_DEPTH = 10


class CmdController(Node):
    def __init__(self):
        super().__init__('cmd_controller')
        self.pub_ = self.create_publisher(Int32MultiArray, TOPIC, PUB_DEPTH)


    def send_command(self, values) -> bool:
        """发布一条命令。返回 False 表示参数个数不合法、未发布。"""

        subs = self.pub_.get_subscription_count()
        if subs == 0:
            self.get_logger().warning(f"{TOPIC} 当前没有订阅者。请确认 stm32 节点已启动")

        msg = Int32MultiArray()
        msg.data = [int(v) for v in values]
        self.pub_.publish(msg)

        self.get_logger().info(f"已发布 {list(msg.data)}")
        return True


def command_parser(p):
    if   p[0] == "start":
        return [0, 0]
    elif p[0] == "stop":
        return [0, 1]
    
    elif p[0] == "oled":
        if   p[1] == "on":
            return [1, 1]
        elif p[1] == "off":
            return [1, 0]

    elif p[0] == "speed":
        if len(p) < 3: return None
        if   p[1] == "set":
            return [2, 1, p[2]]
        elif p[1] == "add":
            return [2, 2, p[2]]
        elif p[1] == "remove":
            return [2, 3, p[2]]

    elif p[0] == "cooling_time":
        if p[1] == "set":
            return [3, 1, p[2]]
        elif p[1] == "add":
            return [3, 2, p[2]]
        elif p[2] == "remove":
            return [3, 3, p[2]]


def strline2cmd(line: str):
    """把输入解析成命令"""
    parts = line.split()
    if not parts:
        return None

    try:
        return command_parser(parts)
    except ValueError:
        return 'ERROR'


def main(args=None):
    rclpy.init(args=args)
    node = CmdController()

    node.get_logger().info(f'命令控制器已启动，发布到 {TOPIC}')
    node.get_logger().info('输入 q 以退出')

    try:
        while rclpy.ok():
            try:
                line = input('> ')
            except EOFError: # 如果主动按下Ctrl+C退出
                print("已退出")
                break

            line = line.strip()
            if not line:
                continue
            if line.lower() in ('q', 'quit', 'exit'):
                print("已退出")
                break

            values = strline2cmd(line)
            if values is None:
                continue
            if values == 'ERROR':
                node.get_logger().error(
                    f'无法解析为整数: {line!r}')
                continue

            node.send_command(values)

    except KeyboardInterrupt:
        print()

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
