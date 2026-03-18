import math
import numpy as np
import cv2
from typing import Optional
from geometry_msgs.msg import Pose

from . import aux

ROBOT_RADIUS = 200


class Router:
    def __init__(self) -> None:
        self.ally_pos: Optional[aux.Point] = None
        self.ducks: list[aux.Point] = []
        self.enemy_robot: Optional[aux.Point] = None
        self.enemy_robot_size: Optional[tuple[float, float]] = None

    def reset_locals(self) -> None:
        self.ducks = []
        self.enemy_robot = None
        self.enemy_robot_size = None

    def set_ally(self, position: aux.Point) -> None:
        self.ally_pos = position

    def find_objects(
        self,
        image: cv2.typing.MatLike,
        num_labels: int,
        labels: cv2.typing.MatLike,
        stats: cv2.typing.MatLike,
        centroids: cv2.typing.MatLike,
    ) -> None:
        """returns all correct targets and"""
        MIN_AREA = 50  # слишком мелкий шум
        BIG_W = 180  # слишком большой объект по ширине
        BIG_H = 180  # слишком большой объект по высоте
        MAX_ASPECT_RATIO = 4.0  # слишком длинный / узкий объект

        for label_id in range(1, num_labels):
            x = stats[label_id, cv2.CC_STAT_LEFT]
            y = stats[label_id, cv2.CC_STAT_TOP]
            w = stats[label_id, cv2.CC_STAT_WIDTH]
            h = stats[label_id, cv2.CC_STAT_HEIGHT]
            area = stats[label_id, cv2.CC_STAT_AREA]

            cx, cy = centroids[label_id]
            cx = int(cx)
            cy = int(cy)

            if area < MIN_AREA:  # мелкий обьект
                continue

            if aspect_ratio > MAX_ASPECT_RATIO:  # полоска
                continue

            aspect_ratio = max(w, h) / max(1, min(w, h))

            if w > BIG_W and h > BIG_H:  # вражеский робот

                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(image, (cx, cy), 8, (255, 0, 255), -1)

                self.enemy_robot = aux.Point(
                    cx, cy
                )  # NOTE можно сохранять в поле класса чтобы не пропадал
                self.enemy_robot_size = (w, h)
                continue

            # нормальный обьект
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(image, (cx, cy), 6, (0, 0, 255), -1)

            self.ducks.append(aux.Point(cx, cy))

    def choose_target(self, image: cv2.typing.MatLike) -> Optional[Pose]:
        if self.ally_pos is None:
            return None
        if (
            self.enemy_robot is not None
            and aux.dist(self.ally_pos, self.enemy_robot) < ROBOT_RADIUS * 2
        ):
            # NOTE в страхе убегаем от противника, отдаем банку?
            point = aux.nearest_point_on_circle(
                self.ally_pos, self.enemy_robot, ROBOT_RADIUS * 3
            )
            return point_to_pose(point)

        target: tuple[aux.Point] = None

        points_with_length: list[tuple[aux.Point, float]] = []
        for duck in self.ducks:
            if (
                self.enemy_robot is None
                or aux.dist2line(self.ally_pos, duck, self.enemy_robot)
                < ROBOT_RADIUS * 2
            ):  # можно проехать
                points_with_length.append((duck, aux.dist(self.ally_pos, duck)))
            elif self.enemy_robot is not None:
                # враг мешает
                passthrough_points: list[aux.Point] = []
                delta_angle = math.asin(ROBOT_RADIUS / aux.dist())
                for angle in [-delta_angle, delta_angle]:
                    ally_to_enemy = self.enemy_robot - self.ally_pos
                    ally_vec = self.ally_pos + aux.rotate(ally_to_enemy, angle)
                    duck_vec = self.ally_pos + aux.rotate(ally_to_enemy, -angle)

                    passthrough_point = aux.get_line_intersection(
                        self.ally_pos, ally_vec, duck, duck_vec, "LL"
                    )
                    if passthrough_point is not None and is_point_in_field(
                        passthrough_point
                    ):
                        passthrough_points.append(passthrough_point)

                if len(passthrough_points) != 0:
                    best_point = aux.find_nearest_point(
                        self.ally_pos, passthrough_points
                    )
                    points_with_length.append(
                        (
                            best_point,
                            aux.dist(self.ally_pos, best_point)
                            + aux.dist(best_point, duck),
                        )
                    )
        if len(points_with_length) != 0:
            target = sorted(points_with_length, key=lambda x: x[1])[0]

        return point_to_pose(target)


def point_to_pose(point: aux.Point) -> Pose:
    pose = Pose()
    pose.position.x = point.x
    pose.position.y = float(point.y)
    pose.position.z = 0.0
    # работает только потому что количество пикселей совпадает с мм

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    return pose


def is_point_in_field(point: aux.Point) -> bool:
    return (
        ROBOT_RADIUS < point.x < 1250 - ROBOT_RADIUS
        and ROBOT_RADIUS < point.y < 1750 - ROBOT_RADIUS
    )
