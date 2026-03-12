
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
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

        self.pose_camera = self.create_subscription(
            PoseStamped,
            '/image_recalib_apriltag_pose',
            self.pose_callback_camera,
            10
        )

        self.poseOdom_sub = self.create_subscription(
            Odometry, '/odom', self.pose_callback_robot, 10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )

        self.current_pose = Pose()

        self.amgoing = False

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.angle_pos = 0.0

        self.k_forward = 0.58
        self.k_turn = 1.5
        self.maxline = 0.2
        self.maxz = 0.8

        self.distotcl = 0.1

        self.flag_going = False
        self.turning = True

    def goal_pose_callback(self, msg):
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y
        self.flag_going = False
        self.turning = True
        self.angle_pos = math.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2

        self.get_logger().info(f'Идём к точке (x={self.x_goal:.2f}, y={self.y_goal:.2f}), angle={self.angle_pos}')

    def set_goal(self, x, y):
        self.x_goal = x
        self.y_goal = y

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def pose_callback_camera(self, msg):
        self.current_pose.x = msg.pose.position.x
        self.current_pose.y = msg.pose.position.y
        self.current_pose.theta = math.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2
        # self.get_logger().info(f'Нам камера пишет) {self.current_pose}')

    def pose_callback_robot(self, msg):
        # self.current_pose = msg.pose.pose
        robot_pose = Pose()
        robot_pose.x = msg.pose.pose.position.x
        robot_pose.y = msg.pose.pose.position.y
        robot_pose.theta = math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2
        self.get_logger().info(f'Нам написали) {robot_pose}')

    def control_loop(self):
        # if distance < self.distotcl and self.flag_going:
        #     twist = Twist()
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0
        #     self.publisher_.publish(twist)
        #     self.amgoing = True
        #     return 
        # elif distance < self.distotcl:
        #     self.turning = False
        #     desired_angle = self.angle_pos
        #     angle_error = desired_angle - self.current_pose.theta
        #     angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        #     self.get_logger().info(f'ошибка поворота) {angle_error}')
        #     if abs(angle_error) < self.distotcl:
        #         self.flag_going = True
        # else:
        #     desired_angle = math.atan2(dy, dx)
        #     angle_error = desired_angle - self.current_pose.theta
        #     angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        # ####
        # #####

        # twist = Twist()
        # if (angle_error < 0.2 and angle_error > -0.2 and self.turning):
        #     twist.linear.x = self.k_forward * distance
        #     if twist.linear.x > self.maxline:
        #         twist.linear.x = self.maxline
        # else:
        #     twist.linear.x = 0.0

        # twist.angular.z = self.k_turn * angle_error
        # if twist.angular.z > self.maxz:
        #     twist.angular.z = self.maxz
        # elif twist.angular.z < -self.maxz:
        #     twist.angular.z = -self.maxz

        anglereg = 0.0
        v = 0.0

        dx = self.x_goal - self.current_pose.x
        dy = self.y_goal - self.current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        desired_angle = math.atan2(dy, dx)
        angle_errorToFr = desired_angle - self.current_pose.theta
        angle_errorToFr = math.atan2(math.sin(angle_errorToFr), math.cos(angle_errorToFr))

        desired_angle = self.angle_pos
        angle_error = desired_angle - self.current_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_errorToFr) > 0.2 and abs(distance) > self.distotcl:
            amglereg = angle_errorToFr
        elif abs(distance) > self.distotcl:
            v = distance
            amglereg = angle_errorToFr
        elif abs(angle_error) > 0.1:
            amglereg = angle_error
        else:
            v = 0
            amglereg = 0

        twist = Twist()

        twist.linear.x = self.k_forward * v
        if twist.linear.x > self.maxline:
            twist.linear.x = self.maxline

        twist.angular.z = self.k_turn * amglereg
        if twist.angular.z > self.maxz:
            twist.angular.z = self.maxz
        elif twist.angular.z < -self.maxz:
            twist.angular.z = -self.maxz

        # self.get_logger().info(f'eps: {angle_errorToFr}, dtheta: {angle_error}, S: {distance}')
        # self.get_logger().info(f'ошибка поворота) {amglereg}')
        # self.get_logger().info(f'расстояние вперед) {distance}')

        self.publisher_.publish(twist)
        


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseFollower()
    node.get_logger().info("ДОМОЙ")
    try:
        while rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


main()
