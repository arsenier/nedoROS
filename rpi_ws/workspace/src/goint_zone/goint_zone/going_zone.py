import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
# from turtlesim.msg import Pose 
from builtin_interfaces.msg import Time
import time

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool

from dataclasses import dataclass

@dataclass
class Pose:
    x: float
    y: float
    theta: float

class Going_zone_obect(Node):
    def __init__(self):
        super().__init__('going_zone')
        
        self.poseOdom_sub = self.create_subscription(
            Odometry, '/odom', self.pose_callback_robot, 10
        )  

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar, 10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bool_pub = self.create_publisher(Bool, '/gripper', 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.stuk)

        self.cma = 0
        self.robot_pose = Pose(x = 0.0, y = 0.0, theta = 0.0)
        self.cmr = 0
        self.mass = 1
        self.robot_theta = 0.0

        self.angle_min_out = 0
        self.angle_max_out = 0
        self.angle_increment = 0
        self.lidar_offset = -90.0 * math.pi / 180

        self.angle_par = math.pi * 2 / 3
        self.dist_par = 50.0 / 100
        self.dist_par_low = 20.0 / 100

        self.angle_otbros = math.pi/9

        self.pc = []# это надо взять точки у лидара

        self.porog_turn = math.pi / 18
        self.k_forward = 1.0
        self.k_turn = 2.5
        self.max_v = 0.1
        self.max_w = 1.0

        self.const_dist_gripper = 22.0 / 100
        self.porog_dist = 2.0 / 100

    def lidar(self, msg):
        self.angle_min_out = msg.angle_min
        self.angle_max_out = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.pc = msg.ranges

        self.cma = 0
        self.cmr = 0
        self.mass = 0

        for idx, p in enumerate(self.pc):
            angle = self.angle_min_out - idx * self.angle_increment + self.lidar_offset

            while angle > math.pi:
                angle -= 2*math.pi
            
            while angle < -math.pi:
                angle += 2*math.pi

            distance = p

            # if abs(angle) < self.angle_otbros:
            if abs(angle) < self.angle_par and distance < self.dist_par and distance > self.dist_par_low:
                # self.get_logger().info(f'{idx}, {p}, {angle}, {distance}')
                self.cma += angle
                self.cmr += distance
                self.mass += 1

        if self.mass != 0:
            self.cma /= self.mass
            self.cmr /= self.mass
        
        self.robot_theta = 0.0

        # self.get_logger().info(f'cma: {self.cma}, cmr: {self.cmr}')

    def pose_callback_robot(self, msg):
        # self.current_pose = msg.pose.pose
        # self.robot_pose.x = msg.pose.pose.position.x
        # self.robot_pose.y = msg.pose.pose.position.y
        # self.robot_pose.theta = math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2
        # self.get_logger().info(f'Нам написали) {self.robot_pose}')
        pass
        
    def stuk(self):
        twist = Twist()
        v = 0.0
        theta_turn = 0.0
        gripper = Bool()
        gripper.data = False

        dtheta = self.cma - self.robot_theta
        ddist = self.cmr - self.const_dist_gripper
        if abs(dtheta) > self.porog_turn:
            v = 0.0
            theta_turn = self.k_turn * dtheta
        else:
            v = self.k_forward * ddist
            theta_turn = self.k_turn * dtheta
        
        if abs(dtheta) < (self.porog_turn / 2) and abs(ddist) < self.porog_dist:
            gripper.data = True

        if self.cmr < 0.1:
            v = 0.0

        v = min(max(v, -self.max_v), self.max_v)
        theta_turn = min(max(theta_turn, -self.max_w), self.max_w)

        self.robot_theta += theta_turn * self.timer_period * 2

        twist.linear.x = v
        twist.angular.z = theta_turn
        self.publisher_.publish(twist)

        if gripper.data:
            self.bool_pub.publish(gripper)

        self.get_logger().info(f'cma: {self.cma}, cmr: {self.cmr}, twist: {twist}, grip: {gripper.data}')
        


def main(args=None):
    rclpy.init(args=args)
    node = Going_zone_obect()
    try:
        while rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


main()
