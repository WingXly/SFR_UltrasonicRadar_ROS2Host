#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class CommandError(Exception): pass


TOPIC = '/radar/command'
PUB_DEPTH = 10


COMMANDS = {
    ("start",):                (0, 0),
    ("stop",):                 (0, 1),
    ("oled",):                 (1, "bool"),
    ("speed",):                (2, "value_operation", "uint16"),
    ("cooling_time",):         (3, "value_operation", "uint16"),
    ("resample",):             (4, "uint16"),
}

DISABLES = [
    "stop",
    "disable",
    "off",
    "0",
    "no",
    "not"
]

ENABLES = [
    "start",
    "enable",
    "on",
    "1",
    "yes",
    "go",
    "run"
]

# 使用sorted列表索引，优先识别长命令
COMMAND_SORTED_LIST = sorted(COMMANDS, key=len, reverse=True)


def textList2cmd(parts: list):
    """把输入解析成命令"""
    if not parts:
        return None

    for cmd_type in COMMAND_SORTED_LIST:
        if tuple(parts[:len(cmd_type)]) == cmd_type: # 定义命令类型
            argv = parts[len(cmd_type):] # 定义参数
            out = []

            for item in COMMANDS[cmd_type]:
                if item == "uint16":
                    if not argv:
                        raise CommandError(f"'{' '.join(cmd_type)}' 缺少参数")

                    try:
                        value = int(argv[0])
                    except ValueError:
                        raise CommandError(f"'{' '.join(cmd_type)}' 参数必须是整数: {argv[0]!r}")

                    if not (0 <= value <= 65535):
                        raise CommandError(f"'{' '.join(cmd_type)}' 参数类型必须为 uint16_t")

                    out.append(value)
                    argv.pop(0)

                elif item == "value_operation":
                    if not argv:
                        raise CommandError(f"'{' '.join(cmd_type)}' 缺少参数")

                    if argv[0] == "set":
                        out.append(1)

                    elif argv[0] == "add":
                        out.append(2)

                    elif argv[0] == "remove":
                        out.append(3)

                    else:
                        raise CommandError(f"'{' '.join(cmd_type)}' 未知含义的参数: {argv[0]}")

                    argv.pop(0)

                elif item == "bool":
                    if not argv:
                        raise CommandError(f"'{' '.join(cmd_type)}' 缺少参数")

                    if argv[0].lower() in DISABLES:
                        out.append(0)

                    elif argv[0].lower() in ENABLES:
                        out.append(1)

                    else:
                        raise CommandError(f"'{' '.join(cmd_type)}' 未知含义的参数: {argv[0]}")

                    argv.pop(0)

                else:
                    out.append(item)

            if argv:
                raise CommandError(f"多余参数: {argv}")
            
            return out

    raise CommandError(f"未知命令: {parts}")


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
        self.get_logger().info(f"已发布 {msg.data}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CmdController()

    node.get_logger().info(f'命令控制器已启动，发布到 {TOPIC}')
    node.get_logger().info('输入 q 以退出')

    try:
        while rclpy.ok():
            line = input('> ').strip().split()
            if not line:
                continue

            if line[0].lower() in ['/']:
                line.pop(0)
            
            if line[0].lower() in ['q', 'quit', 'exit']:
                print("已退出")
                break

            try:
                values = textList2cmd(line)
            except CommandError as e:
                node.get_logger().error(str(e))
                continue

            if values is not None:
                node.send_command(values)

    except KeyboardInterrupt:
        print("已退出")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
