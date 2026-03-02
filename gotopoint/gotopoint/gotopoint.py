
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from turtlesim.msg import Pose  # Тип Pose из turtlesim

class TurtlePoseFollower(Node):
    def __init__(self):
        super().__init__('gotopoint')

        # Подписчик на /turtle1/pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )

        self.current_pose = Pose()

        self.amgoing = False

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.x_goal = 0.0
        self.y_goal = 0.0

        self.k_forward = 1.0
        self.k_turn = 4.0

    def goal_pose_callback(self, msg):
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y

    def set_goal(self, x, y):
        self.x_goal = x
        self.y_goal = y

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def control_loop(self):
        dx = self.x_goal - self.current_pose.x
        dy = self.y_goal - self.current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.3:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.amgoing = True
            print("все конец")
            return 

        ####
        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.current_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        #####

        twist = Twist()
        print(angle_error)
        if (angle_error < 1 and angle_error > -1):
            twist.linear.x = self.k_forward * distance
            if twist.linear.x > 2.0:
                twist.linear.x = 2.0
        else:
            twist.linear.x = 0.0

        twist.angular.z = self.k_turn * angle_error
        if twist.angular.z > 2.0:
            twist.angular.z = 2.0
        elif twist.angular.z < -2.0:
            twist.angular.z = -2.0

        self.publisher_.publish(twist)
        


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseFollower()

    try:
        while rclpy.ok():
            # Запрашиваем у пользователя координаты цели
            # user_input = input("\nВведите x y (или q для выхода): ").strip()
            # if not user_input:
            #     continue
            # if user_input.lower() == 'q':
            #     print("Выход из программы.")
            #     break

            # # Парсим координаты
            # coords = user_input.split()
            # if len(coords) < 2:
            #     print("Ошибка: введите 2 числа (x и y) или 'q' для выхода.")
            #     continue

            # try:
            #     x_goal = float(coords[0])
            #     y_goal = float(coords[1])
            # except ValueError:
            #     print("Ошибка: некорректные координаты.")
            #     continue

            # node.set_goal(x_goal, y_goal)
            # node.get_logger().info(
            #     f'Идём к точке (x={x_goal:.2f}, y={y_goal:.2f})...'
            # )

            rclpy.spin(node)
            # while rclpy.ok() and node.amgoing == False:
            #     rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


main()
