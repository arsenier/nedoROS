#!/usr/bin/env python3
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer


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

        self.tag_topic = self.declare_parameter('tag_topic', '/image_raw_apriltag_pose').value
        self.charuco_topic = self.declare_parameter('charuco_topic', '/image_raw_charuco_pose').value
        self.slop = float(self.declare_parameter('sync_slop_sec', 0.02).value)
        self.print_hz = float(self.declare_parameter('print_hz', 5.0).value)

        self._last_print = 0.0

        self.tag_sub = Subscriber(self, PoseStamped, self.tag_topic)
        self.charuco_sub = Subscriber(self, PoseStamped, self.charuco_topic)

        self.sync = ApproximateTimeSynchronizer([self.tag_sub, self.charuco_sub],
                                                queue_size=30, slop=self.slop)
        self.sync.registerCallback(self.cb)

        self.get_logger().info(f"Listening tag={self.tag_topic} charuco={self.charuco_topic} slop={self.slop}s")

    def cb(self, tag_msg: PoseStamped, ch_msg: PoseStamped):
        now = time.time()
        if self.print_hz > 0 and (now - self._last_print) < (1.0 / self.print_hz):
            return
        self._last_print = now

        if tag_msg.header.frame_id != ch_msg.header.frame_id:
            self.get_logger().warn(f"frame mismatch: tag='{tag_msg.header.frame_id}' charuco='{ch_msg.header.frame_id}'")
            return

        R_ct = quat_to_rot(tag_msg.pose.orientation)     # R_cam_tag
        R_cc = quat_to_rot(ch_msg.pose.orientation)      # R_cam_charuco

        t_ct = np.array([tag_msg.pose.position.x, tag_msg.pose.position.y, tag_msg.pose.position.z], dtype=np.float64)
        t_cc = np.array([ch_msg.pose.position.x, ch_msg.pose.position.y, ch_msg.pose.position.z], dtype=np.float64)

        # tag position in charuco
        t_ch_tag = R_cc.T @ (t_ct - t_cc)

        # relative rotation charuco->tag
        R_ch_tag = R_cc.T @ R_ct
        yaw = math.atan2(R_ch_tag[1, 0], R_ch_tag[0, 0])   # around Z of charuco
        yaw_deg = yaw * 180.0 / math.pi

        self.get_logger().info(f"x={t_ch_tag[0]*10:+.4f} m  y={t_ch_tag[1]*10:+.4f} m yaw={yaw_deg:+.2f} deg")


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