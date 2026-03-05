import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose 
from builtin_interfaces.msg import Time
import time

class Publisher(Node):
    def __init__(self):
        super().__init__('giveandout')
    
        self.goal_pub = self.create_publisher(
            PoseStamped, "/goal_pose", 10)
        
        self.timer_period = 0
        self.timer = self.create_timer(self.timer_period, self.publishing)
        self.timer_period2 = 10
        self.timer2 = self.create_timer(self.timer_period2, self.publishing2)

    def publishing(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 0.5
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 1.0
        pose_msg.pose.orientation.w = 0.0
        self.goal_pub.publish(pose_msg)

        self.timer.cancel()

    def publishing2(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 1.0
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(pose_msg)
        self.timer2.cancel()



def main(args=None):
    rclpy.init(args=args)
    node = Publisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


main()
