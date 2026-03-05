#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ], dtype=np.float64)


class XYYawInCharuco(Node):
    def __init__(self):
        super().__init__('tag_xyyaw_in_charuco')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tag_topic = self.declare_parameter('tag_topic', '/image_raw_apriltag_pose').value
        self.charuco_topic = self.declare_parameter('charuco_topic', '/image_raw_charuco_pose').value
        self.slop = float(self.declare_parameter('sync_slop_sec', 0.02).value)
        self.print_hz = float(self.declare_parameter('print_hz', 5.0).value)

        self.odom_publisher_ = self.create_publisher(Odometry, '/map', 10)

        self._last_print = 0.0

        self.tag_sub = Subscriber(self, PoseStamped, self.tag_topic)
        self.charuco_sub = Subscriber(self, PoseStamped, self.charuco_topic)

        self.sync = ApproximateTimeSynchronizer([self.tag_sub, self.charuco_sub],
                                                queue_size=30, slop=self.slop)
        self.sync.registerCallback(self.cb)

        self.get_logger().info(f"Listening tag={self.tag_topic} charuco={self.charuco_topic} slop={self.slop}s")

    def cb(self, tag_msg: PoseStamped, ch_msg: PoseStamped):
        now = rclpy.time.Time()


        trans = self.tf_buffer.lookup_transform(
            'calib_charuco',
            'default_cam',
            now)
        
        point_on_board = tf2_geometry_msgs.do_transform_pose_stamped(tag_msg, trans)

        angle = math.atan2(point_on_board.pose.orientation.w, point_on_board.pose.orientation.z)
        self.get_logger().info(f"x={point_on_board.pose.position.x*10:+.4f} m  y={point_on_board.pose.position.y*10:+.4f} m yaw={angle:+.2f} rad")

        self.publish_odometry(point_on_board)

    def publish_odometry(self, pose_stamped: PoseStamped):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_stamped.pose.position.x*10
        msg.pose.pose.position.y = pose_stamped.pose.position.y*10
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x =0.0
        msg.pose.pose.orientation.y =0.0
        msg.pose.pose.orientation.z = pose_stamped.pose.orientation.z
        msg.pose.pose.orientation.w = pose_stamped.pose.orientation.w
        self.odom_publisher_.publish(msg)


def main():
    rclpy.init()
    node = XYYawInCharuco()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()