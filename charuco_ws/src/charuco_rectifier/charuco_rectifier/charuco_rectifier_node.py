import math
from typing import Optional
import numpy as np
import cv2

from ultralytics import YOLO
import torch


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, Int16MultiArray

from . import aux
from .router import Router

# title_window = "sad"
# cv2.namedWindow(title_window)
# alpha_slider_max = 255


# def on_trackbar(val):
#     pass


# trackbar_name = "Alpha x %d" % alpha_slider_max
# cv2.createTrackbar(trackbar_name, title_window, 0, alpha_slider_max, on_trackbar)


class CharucoRectifierNode(Node):
    def __init__(self):
        super().__init__("charuco_rectifier")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("board_pose_topic", "/image_raw_charuco_pose")
        self.declare_parameter("ally_pose_topic", "/image_recalib_apriltag_pose")
        self.declare_parameter("start_topic", "/start")

        self.declare_parameter("number_of_squares_x", 5)
        self.declare_parameter("number_of_squares_y", 7)
        self.declare_parameter("square_size_m", 0.025)

        self.declare_parameter("output_width_px", 1250)
        self.declare_parameter("output_height_px", 1750)

        self.declare_parameter("board_origin_mode", "corner")  # corner / center

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.board_pose_topic = self.get_parameter("board_pose_topic").value
        self.ally_pose_topic = self.get_parameter("ally_pose_topic").value
        self.start_topic = self.get_parameter("start_topic").value

        self.number_of_squares_x = self.get_parameter("number_of_squares_x").value
        self.number_of_squares_y = self.get_parameter("number_of_squares_y").value
        self.square_size_m = self.get_parameter("square_size_m").value

        self.output_width_px = self.get_parameter("output_width_px").value
        self.output_height_px = self.get_parameter("output_height_px").value

        self.board_origin_mode = self.get_parameter("board_origin_mode").value

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
        self.ally_pose_sub = self.create_subscription(
            PoseStamped, self.ally_pose_topic, self.ally_pose_callback, 10
        )
        self.start_sub = self.create_subscription(
            Bool, self.start_topic, self.start_callback, 10
        )

        self.rectified_pub = self.create_publisher(
            Image, self.image_topic + "_charuco_rectified", 10
        )
        self.diff_pub = self.create_publisher(
            Image, self.image_topic + "_charuco_rectified_diff", 10
        )
        self.debug_pub = self.create_publisher(
            Image, self.image_topic + "_charuco_rectified_debug", 10
        )

        self.objects_pub = self.create_publisher(
            Int16MultiArray, self.image_topic + "_charuco_objects", 10
        )

        self.get_logger().info("CharucoRectifierNode started")

        self.etalon: rclpy.Optional[np.ndarray] = None
        self.router = Router()

        self.start = False
        self.top_of_objects: Optional[list[int]] = None

        self.black_box = DetectModelYolov8("../yolo_ws/model/best.pt")

    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))

        d = np.array(msg.d, dtype=np.float64)
        if d.size >= 5:
            self.D = d[:5].reshape((1, 5))
        elif d.size > 0:
            tmp = np.zeros((1, 5), dtype=np.float64)
            tmp[0, : d.size] = d
            self.D = tmp
        else:
            self.D = np.zeros((1, 5), dtype=np.float64)

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose_msg = msg

    def ally_pose_callback(self, msg: PoseStamped):
        self.router.set_ally(msg.pose)
        # print(self.router.ally_pos)

    def start_callback(self, msg: bool):
        self.start = msg

    def image_callback(self, msg: Image):
        if self.K is None or self.D is None:
            self.get_logger().warn(
                "Waiting for camera_info...", throttle_duration_sec=2.0
            )
            return

        if self.latest_pose_msg is None:
            self.get_logger().warn(
                "Waiting for board pose...", throttle_duration_sec=2.0
            )
            return

        try:
            cv_img = self.image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return

        try:
            src_pts = self.project_board_corners(self.latest_pose_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to project board corners: {e}")
            return

        dbg = cv_img.copy()
        self.draw_quad_debug(dbg, src_pts)

        dst_pts = np.array(
            [
                [0, 0],
                [self.output_width_px - 1, 0],
                [self.output_width_px - 1, self.output_height_px - 1],
                [0, self.output_height_px - 1],
            ],
            dtype=np.float32,
        )

        H = cv2.getPerspectiveTransform(src_pts.astype(np.float32), dst_pts)
        rectified = cv2.warpPerspective(
            cv_img, H, (self.output_width_px, self.output_height_px)
        )

        """
        rect = rectified.copy()

        if self.etalon is None:
            self.etalon = np.load("./etalon.npy")  # comment to calibrate
            return
            self.etalon = rectified.copy()
            np.save("./etalon.npy", self.etalon)
            return

        diff = np.abs(rectified.astype(np.int16) - self.etalon.astype(np.int16)).astype(
            np.uint8
        )
        # diff = diff.mean(axis=2).astype(np.uint8)
        # diff = np.repeat(diff[:, :, None], 3, axis=2)

        lower_int = 50
        upper_bgr = np.array([255, 255, 255], dtype=np.uint8)
        mask = np.zeros((diff.shape[0], diff.shape[1]), dtype=np.uint8)
        for lower_boarder in [[lower_int, 0, 0], [0, lower_int, 0], [0, 0, lower_int]]:
            lower_bgr = np.array(lower_boarder, dtype=np.uint8)
            mask |= cv2.inRange(diff, lower_bgr, upper_bgr)

        if self.router.ally_pos is not None:
            radius = 300
            ally_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.circle(ally_mask, self.router.ally_pos.get_cords_int(), radius, 255, -1)
            mask[ally_mask > 0] = 0

        # исправляем тонкие линии, возникающие от неточного "вырезания" оригинального поля
        param_int = 10
        kernel_open = np.ones((param_int, param_int), np.uint8)
        mask_joined = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        # сливаем соседние контуры в один
        param_int = 50
        kernel_close = np.ones((param_int, param_int), np.uint8)
        mask_joined = cv2.morphologyEx(mask_joined, cv2.MORPH_CLOSE, kernel_close)

        # раздуваем маску
        param_int = 10
        kernel_dilate = np.ones((param_int, param_int), np.uint8)
        mask_joined = cv2.dilate(mask_joined, kernel_dilate, iterations=1)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask_joined, connectivity=8
        )
        self.router.find_objects(rectified, num_labels, labels, stats, centroids)

        # target = self.router.choose_target(rectified)
        # if target is not None:
        #     self.ducks_pub.publish(target)

        mask_bgr = cv2.cvtColor(mask_joined, cv2.COLOR_GRAY2BGR)

        rect_msg = self.bgr_to_image_msg(mask_bgr, msg.header)
        diff_msg = self.bgr_to_image_msg(diff, msg.header)
        """
        dbg_msg = self.bgr_to_image_msg(rectified, msg.header)

        # self.rectified_pub.publish(rect_msg)
        # self.diff_pub.publish(diff_msg)
        self.debug_pub.publish(dbg_msg)

        imge: Optional[cv2.typing.MatLike] = None
        if self.top_of_objects is None or not self.start:
            new_objects, imge = self.get_objects(rectified)
            if new_objects is not None:
                if self.top_of_objects is None or not 78 in new_objects:
                    for idx, fig in enumerate(new_objects):
                        if fig == 78:
                            new_objects[idx] = 7
                            
                    self.top_of_objects = new_objects

        if self.top_of_objects is not None:
            self.objects_pub.publish(Int16MultiArray(data=self.top_of_objects))

        if imge is not None:
            imge_msg = self.bgr_to_image_msg(imge, msg.header)
            self.rectified_pub.publish(imge_msg)

    def image_msg_to_bgr(self, msg: Image) -> list[int]:
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        channels = 3
        row_step_expected = msg.width * channels

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.step))

        arr = arr[:, :row_step_expected]
        img = arr.reshape((msg.height, msg.width, channels))

        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img.copy()

    def bgr_to_image_msg(self, img: np.ndarray, header) -> Image:
        if img.dtype != np.uint8:
            raise ValueError("Only uint8 images are supported")

        if len(img.shape) != 3 or img.shape[2] != 3:
            raise ValueError("Expected BGR image with shape HxWx3")

        msg = Image()
        msg.header = header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg

    def project_board_corners(self, pose_msg: PoseStamped) -> np.ndarray:
        board_w = self.number_of_squares_x * self.square_size_m
        board_h = self.number_of_squares_y * self.square_size_m

        if self.board_origin_mode == "corner":
            obj_pts = np.array(
                [
                    [board_w, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, board_h, 0.0],
                    [board_w, board_h, 0.0],
                ],
                dtype=np.float32,
            )
        elif self.board_origin_mode == "center":
            hx = board_w / 2.0
            hy = board_h / 2.0
            obj_pts = np.array(
                [
                    [-hx, -hy, 0.0],
                    [hx, -hy, 0.0],
                    [hx, hy, 0.0],
                    [-hx, hy, 0.0],
                ],
                dtype=np.float32,
            )
        else:
            raise ValueError(f"Unsupported board_origin_mode: {self.board_origin_mode}")

        tvec = np.array(
            [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ],
            dtype=np.float64,
        ).reshape((3, 1))

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
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-12:
            raise ValueError("Quaternion norm is zero")

        qx /= n
        qy /= n
        qz /= n
        qw /= n

        return np.array(
            [
                [
                    1 - 2 * qy * qy - 2 * qz * qz,
                    2 * qx * qy - 2 * qz * qw,
                    2 * qx * qz + 2 * qy * qw,
                ],
                [
                    2 * qx * qy + 2 * qz * qw,
                    1 - 2 * qx * qx - 2 * qz * qz,
                    2 * qy * qz - 2 * qx * qw,
                ],
                [
                    2 * qx * qz - 2 * qy * qw,
                    2 * qy * qz + 2 * qx * qw,
                    1 - 2 * qx * qx - 2 * qy * qy,
                ],
            ],
            dtype=np.float64,
        )

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
                cv2.LINE_AA,
            )

    half_size = 30
    margin = 20
    x_delta = 20
    y_delta = 15
    left_centers: list[tuple[int, int]] = [
        # first line
        (1000 + half_size, 500 + half_size),
        (750 - half_size, 500 + half_size),
        (500 + half_size, 500 + half_size),
        (250 - half_size, 500 + half_size),
        # second line
        (1000 - half_size, 750 + half_size),
        (750 + half_size, 750 + half_size),
        (500 - half_size, 750 + half_size),
        (250 + half_size, 750 + half_size),
    ]
    right_centers: list[tuple[int, int]] = [
        # first line
        (1000 + half_size, 1250 - half_size),
        (750 - half_size, 1250 - half_size),
        (500 + half_size, 1250 - half_size),
        (250 - half_size, 1250 - half_size),
        # second line
        (1000 - half_size, 1000 - half_size),
        (750 + half_size, 1000 - half_size),
        (500 - half_size, 1000 - half_size),
        (250 + half_size, 1000 - half_size),
    ]

    def get_objects(
        self, image: cv2.typing.MatLike
    ) -> tuple[Optional[list[int]], Optional[cv2.typing.MatLike]]:
        objects = []
        all_image = None
        size: int = (self.half_size + self.margin) * 2
        for left_center, right_center in zip(self.left_centers, self.right_centers):

            x = left_center[0] - self.half_size - self.margin
            y = left_center[1] - self.half_size - self.margin
            crop_left = image[y : y + size, x : x + size + self.x_delta]
            value_left, img_left = self.black_box.predict_object(crop_left)

            x = right_center[0] - self.half_size - self.margin
            y = right_center[1] - self.half_size - self.margin
            crop_right = image[y : y + size, x : x + size + self.x_delta]
            value_right, img_right = self.black_box.predict_object(crop_right)

            imges = np.concatenate([img_left, img_right], axis=0)
            if all_image is None:
                all_image = imges
            else:
                all_image = np.concatenate([all_image, imges], axis=1)

            # cv2.imshow("hihiha", )
            # cv2.waitKey(10000)

            if value_left != value_right:
                if 78 == value_left:
                    if value_right in [7, 8]:
                        objects.append(value_right)
                        continue

                if 78 == value_right:
                    if value_left in [7, 8]:
                        objects.append(value_left)
                        continue

                return None, all_image

            objects.append(value_right)

        return objects, all_image


class DetectModelYolov8:
    def __init__(self, path_to_model):
        self.classification_model = YOLO(path_to_model)
        self.path_to_model = path_to_model
        self.device, self.fp16 = (
            ("cuda", True) if torch.cuda.is_available() else ("cpu", False)
        )

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    def predict_object(
        self, image: cv2.typing.MatLike
    ) -> tuple[int, cv2.typing.MatLike]:
        """Predict object from image 100x120

        Input:
            image (MatLike): rgb image 100x120

        Return:
            object index (int): 1 - octopus
                                2 -  bunny
                                3 - penguine
                                4 - cilinder
                                5 - blue cube
                                6 - red cube
                                7 - aruca 21
                                8 - aruca 20

            image_result (cv2.Mat)
        """
        try:
            idx, image_aruco = self.detect_aruco(image)
            if idx == 20:
                return 8, image_aruco
            if idx == 21:
                return 7, image_aruco
        except:
            pass
        model_to_yura = {
            "Octopus": 1,
            "Bunny": 2,
            "Penguin": 3,
            "Cylinder": 4,
            "Blue Square": 5,
            "Res square": 6,
            "Aruco": 78,
        }

        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # res = detector_model(img)[0]
        prediction = self.classification_model.predict(
            source=image,
            show=False,
            save=False,
            conf=0.3,
            save_txt=False,
            save_crop=False,
            verbose=False,
            device=self.device,
            half=self.fp16,
        )[0]
        class_label = model_to_yura[prediction.names[prediction.probs.top1]]
        result_image = prediction.plot()
        cv2.putText(
            result_image,
            prediction.names[prediction.probs.top1],
            (10, 80),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (255, 0, 255),
            1,
        )
        return class_label, result_image

    def detect_aruco(self, image: cv2.typing.MatLike) -> tuple[int, cv2.typing.MatLike]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(
            gray,
        )

        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                if marker_id in [20, 21]:
                    return marker_id, cv2.aruco.drawDetectedMarkers(image, corners, ids)

        return -1, image


def main(args=None):
    rclpy.init(args=args)
    node = CharucoRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
