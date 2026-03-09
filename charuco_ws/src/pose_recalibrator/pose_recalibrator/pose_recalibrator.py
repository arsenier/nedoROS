#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # from tf2_geometry_msgs package


class RecalibAprilTagPose(Node):
    def __init__(self):
        super().__init__('recalib_apriltag_pose')

        # Parameters
        self.target_frame = 'calib_charuco'   # TF frame you want poses in

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber and publisher
        self.sub = self.create_subscription(
            PoseStamped,
            '/image_raw_apriltag_pose',
            self.pose_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            '/image_recalib_apriltag_pose',
            10
        )

        self.get_logger().info('RecalibAprilTagPose node started')

    def pose_callback(self, msg: PoseStamped):
        # We assume msg.header.frame_id is the current pose frame (e.g. camera frame)
        if msg.header.frame_id == '':
            self.get_logger().warn('Received PoseStamped without frame_id, skipping')
            return

        try:
            # Lookup transform from target_frame to msg frame at the pose time
            # We want pose expressed in target_frame:
            # pose_out = T(target_frame <- msg.frame) * pose_in
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,           # target frame
                msg.header.frame_id,         # source frame
                rclpy.time.Time.from_msg(msg.header.stamp),  # time
                # timeout=rclpy.duration.Duration(seconds=0.1).to_msg()
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform {self.target_frame} <- {msg.header.frame_id}: {e}')
            return

        transformed_pose = PoseStamped()

        try:
            transformed_pose.pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
        except Exception as e:
            self.get_logger().warn(f'Failed to transform pose: {e}')
            return

        # Set the header correctly for the output message
        transformed_pose.header.stamp = self.get_clock().now().to_msg()
        transformed_pose.header.frame_id = self.target_frame

        transformed_pose.pose.position.x *= 10
        transformed_pose.pose.position.y *= 10
        transformed_pose.pose.position.z = 0.0

        transformed_pose.pose.orientation.x = 0.0
        transformed_pose.pose.orientation.y = 0.0

        self.pub.publish(transformed_pose)


def main(args=None):
    rclpy.init(args=args)
    node = RecalibAprilTagPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
