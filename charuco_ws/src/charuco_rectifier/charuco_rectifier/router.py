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
        self.ducks: list[tuple[aux.Point, int]] = []  # position with life time
        self.enemy_robot: Optional[aux.Point] = None
        self.enemy_robot_size: Optional[tuple[float, float]] = None

    def reset_locals(self) -> None:
        self.ducks = []
        self.enemy_robot = None
        self.enemy_robot_size = None

    def set_ally(self, position: Pose) -> None:
        self.ally_pos = aux.Point(
            1250 - position.position.x * 1000, position.position.y * 1000
        )

    def find_objects(
        self,
        image: cv2.typing.MatLike,
        num_labels: int,
        labels: cv2.typing.MatLike,
        stats: cv2.typing.MatLike,
        centroids: cv2.typing.MatLike,
    ) -> None:
        """returns all correct targets and"""
        MIN_AREA = 1600  # слишком мелкий шум
        BIG_W = 180  # слишком большой объект по ширине
        BIG_H = 180  # слишком большой объект по высоте
        MAX_ASPECT_RATIO = 4.0  # слишком длинный / узкий объект

        old_ducks = self.ducks.copy()
        self.ducks = []

        if self.ally_pos is not None:
            cv2.circle(image, self.ally_pos.get_cords_int(), 20, (127, 127, 255), -1)

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

            aspect_ratio = max(w, h) / max(1, min(w, h))

            if aspect_ratio > MAX_ASPECT_RATIO:  # полоска
                continue

            if w > BIG_W and h > BIG_H:  # вражеский робот

                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(image, (cx, cy), 20, (255, 0, 255), -1)

                self.enemy_robot = aux.Point(
                    cx, cy
                )  # NOTE можно сохранять в поле класса чтобы не пропадал
                self.enemy_robot_size = (w, h)
                continue

            # нормальный обьект
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(image, (cx, cy), 6, (0, 0, 255), -1)

            new_duck = aux.Point(cx, cy)
            for old_duck, lifetime in old_ducks:
                if aux.dist(old_duck, new_duck) < 100:
                    self.ducks.append(((new_duck + old_duck) / 2, lifetime + 1))
                    break
            else:
                self.ducks.append((new_duck, 0))

            cv2.putText(
                image,
                str(self.ducks[-1][1]),
                self.ducks[-1][0].get_cords_int(),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )

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

        points_with_length: list[tuple[aux.Point, float]] = []
        for duck, lifetime in self.ducks:
            if lifetime < 5:
                continue
            if (
                self.enemy_robot is None
                or aux.dist2line(self.ally_pos, duck, self.enemy_robot)
                < ROBOT_RADIUS * 2
            ):  # можно проехать
                points_with_length.append((duck, aux.dist(self.ally_pos, duck)))
            elif self.enemy_robot is not None:
                # враг мешает
                passthrough_points: list[aux.Point] = []
                delta_angle = math.asin(
                    ROBOT_RADIUS / aux.dist(self.ally_pos, self.enemy_robot)
                )
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
                        cv2.circle(
                            image,
                            passthrough_point.get_cords_int(),
                            6,
                            (0, 255, 0),
                            -1,
                        )

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
                    cv2.line(
                        image,
                        best_point.get_cords_int(),
                        duck.get_cords_int(),
                        (0, 127, 0),
                    )

        for point, _ in points_with_length:
            cv2.line(
                image,
                self.ally_pos.get_cords_int(),
                point.get_cords_int(),
                (0, 127, 0),
            )

        if len(points_with_length) != 0:
            target, a = sorted(points_with_length, key=lambda x: x[1])[0]
            cv2.line(
                image,
                self.ally_pos.get_cords_int(),
                target.get_cords_int(),
                (127, 127, 0),
            )
            return point_to_pose(target)

        return None


def point_to_pose(point: aux.Point) -> Pose:
    pose = Pose()
    pose.position.x = float(point.x)
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
