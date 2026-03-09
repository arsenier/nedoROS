import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped


class CharucoRectifierNode(Node):
    def __init__(self):
        super().__init__('charuco_rectifier')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('board_pose_topic', '/image_raw_charuco_pose')

        self.declare_parameter('number_of_squares_x', 5)
        self.declare_parameter('number_of_squares_y', 7)
        self.declare_parameter('square_size_m', 0.025)

        self.declare_parameter('output_width_px', 1250)
        self.declare_parameter('output_height_px', 1750)

        self.declare_parameter('board_origin_mode', 'corner')  # corner / center

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.board_pose_topic = self.get_parameter('board_pose_topic').value

        self.number_of_squares_x = self.get_parameter('number_of_squares_x').value
        self.number_of_squares_y = self.get_parameter('number_of_squares_y').value
        self.square_size_m = self.get_parameter('square_size_m').value

        self.output_width_px = self.get_parameter('output_width_px').value
        self.output_height_px = self.get_parameter('output_height_px').value

        self.board_origin_mode = self.get_parameter('board_origin_mode').value

        self.latest_pose_msg = None
        self.K = None
        self.D = None

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.board_pose_topic, self.pose_callback, 10
        )

        self.rectified_pub = self.create_publisher(
            Image, self.image_topic + '_charuco_rectified', 10
        )
        self.debug_pub = self.create_publisher(
            Image, self.image_topic + '_charuco_rectified_debug', 10
        )

        self.get_logger().info('CharucoRectifierNode started')

        self.etalon: rclpy.Optional[np.ndarray] = None

    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))

        d = np.array(msg.d, dtype=np.float64)
        if d.size >= 5:
            self.D = d[:5].reshape((1, 5))
        elif d.size > 0:
            tmp = np.zeros((1, 5), dtype=np.float64)
            tmp[0, :d.size] = d
            self.D = tmp
        else:
            self.D = np.zeros((1, 5), dtype=np.float64)

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose_msg = msg

    def image_callback(self, msg: Image):
        if self.K is None or self.D is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=2.0)
            return

        if self.latest_pose_msg is None:
            self.get_logger().warn('Waiting for board pose...', throttle_duration_sec=2.0)
            return

        try:
            cv_img = self.image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to decode image: {e}')
            return

        try:
            src_pts = self.project_board_corners(self.latest_pose_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to project board corners: {e}')
            return

        dbg = cv_img.copy()
        self.draw_quad_debug(dbg, src_pts)

        dst_pts = np.array([
            [0, 0],
            [self.output_width_px - 1, 0],
            [self.output_width_px - 1, self.output_height_px - 1],
            [0, self.output_height_px - 1]
        ], dtype=np.float32)

        H = cv2.getPerspectiveTransform(src_pts.astype(np.float32), dst_pts)
        rectified = cv2.warpPerspective(
            cv_img,
            H,
            (self.output_width_px, self.output_height_px)
        )

        if self.etalon is None:
            self.etalon = np.load("./etalon.npy") # comment to calibrate
            return
            self.etalon = rectified.copy()
            np.save("./etalon.npy", self.etalon)
            return
         
        
        diff = np.abs(rectified.astype(np.int16) - self.etalon.astype(np.int16)).astype(np.uint8)

        kernel = np.ones((20,20),np.uint8)
        open_diff = cv2.morphologyEx(diff, cv2.MORPH_OPEN, kernel)

        rect_msg = self.bgr_to_image_msg(open_diff, msg.header)
        dbg_msg = self.bgr_to_image_msg(dbg, msg.header)

        self.rectified_pub.publish(rect_msg)
        self.debug_pub.publish(dbg_msg)

    def image_msg_to_bgr(self, msg: Image) -> np.ndarray:
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise ValueError(f'Unsupported encoding: {msg.encoding}')

        channels = 3
        row_step_expected = msg.width * channels

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.step))

        arr = arr[:, :row_step_expected]
        img = arr.reshape((msg.height, msg.width, channels))

        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img.copy()

    def bgr_to_image_msg(self, img: np.ndarray, header) -> Image:
        if img.dtype != np.uint8:
            raise ValueError('Only uint8 images are supported')

        if len(img.shape) != 3 or img.shape[2] != 3:
            raise ValueError('Expected BGR image with shape HxWx3')

        msg = Image()
        msg.header = header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg

    def project_board_corners(self, pose_msg: PoseStamped) -> np.ndarray:
        board_w = self.number_of_squares_x * self.square_size_m
        board_h = self.number_of_squares_y * self.square_size_m

        if self.board_origin_mode == 'corner':
            obj_pts = np.array([
                [board_w, 0.0,     0.0],
                [0.0,     0.0,     0.0],
                [0.0,     board_h, 0.0],
                [board_w, board_h, 0.0],
            ], dtype=np.float32)
        elif self.board_origin_mode == 'center':
            hx = board_w / 2.0
            hy = board_h / 2.0
            obj_pts = np.array([
                [-hx, -hy, 0.0],
                [ hx, -hy, 0.0],
                [ hx,  hy, 0.0],
                [-hx,  hy, 0.0],
            ], dtype=np.float32)
        else:
            raise ValueError(f'Unsupported board_origin_mode: {self.board_origin_mode}')

        tvec = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z
        ], dtype=np.float64).reshape((3, 1))

        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w

        R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        rvec, _ = cv2.Rodrigues(R)

        img_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.D)
        return img_pts.reshape((-1, 2))

    @staticmethod
    def quaternion_to_rotation_matrix(qx, qy, qz, qw) -> np.ndarray:
        n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n < 1e-12:
            raise ValueError('Quaternion norm is zero')

        qx /= n
        qy /= n
        qz /= n
        qw /= n

        return np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
        ], dtype=np.float64)

    def draw_quad_debug(self, image: np.ndarray, pts: np.ndarray):
        pts_int = pts.astype(int)

        for i in range(4):
            p1 = tuple(pts_int[i])
            p2 = tuple(pts_int[(i + 1) % 4])
            cv2.line(image, p1, p2, (0, 255, 0), 2)

        for i, p in enumerate(pts_int):
            cv2.circle(image, tuple(p), 6, (0, 0, 255), -1)
            cv2.putText(
                image,
                str(i),
                (p[0] + 8, p[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 0),
                2,
                cv2.LINE_AA
            )


def main(args=None):
    rclpy.init(args=args)
    node = CharucoRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()