#!/usr/bin/env python3
""" publish static TF: map -> laser_scan_frame """


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


CHILD_FRAME_ID  = "laser_scan_frame"
FATHER_FRAME_ID = "map"
Translation     = [0.0, 0.0, 0.0]
Rotation        = [0.0, 0.0, 0.0, 1.0]


class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__("static_tf_publisher")
        self.broadcaster_ = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FATHER_FRAME_ID
        t.child_frame_id = CHILD_FRAME_ID

        t.transform.translation.x = Translation[0]
        t.transform.translation.y = Translation[1]
        t.transform.translation.z = Translation[2]

        t.transform.rotation.x = Rotation[0]
        t.transform.rotation.y = Rotation[1]
        t.transform.rotation.z = Rotation[2]
        t.transform.rotation.w = Rotation[3]

        self.broadcaster_.sendTransform(t)
        self.get_logger().info(f"published static_TF: map -> {CHILD_FRAME_ID}")


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()