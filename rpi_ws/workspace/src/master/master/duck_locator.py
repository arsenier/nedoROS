import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class DuckLocator(Node):
    def __init__(self):
        super().__init__('duck_locator')
        self.cma = 0
        self.cmr = 0
        self.is_visible = False
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Bool, '/is_visible', 10)
        # self.timer_period = 0.1
        # self.timer = self.create_timer(self.timer_period, self.main_loop)

    def scan_callback(self, msg):
        """Lidar callback [LaserScan]"""
        lidar_offset = -83.0 * math.pi / 180
        # angle_par = math.pi * 2 / 3
        angle_par = 45.0 * math.pi / 180
        dist_par = 35.0 / 100
        dist_par_low = 18.0 / 100
        angle_otbros = math.pi/9
        pc = []

        angle_min_out = msg.angle_min
        angle_max_out = msg.angle_max
        angle_increment = msg.angle_increment
        pc = msg.ranges

        self.cma = 0
        self.cmr = math.inf
        self.is_visible = False
        mass = 0

        for idx, p in enumerate(pc):
            angle = angle_min_out - idx * angle_increment + lidar_offset

            while angle > math.pi:
                angle -= 2*math.pi
            
            while angle < -math.pi:
                angle += 2*math.pi

            distance = p

            # if abs(angle) < angle_otbros:
            if abs(angle) < angle_par and distance < dist_par and distance > dist_par_low:
                # self.get_logger().info(f'{idx}, {p}, {angle}, {distance}')
                # self.cma += angle
                # self.cmr += distance
                # mass += 1
                if distance < self.cmr:
                    self.cmr = distance
                    self.cma = angle
                    mass = 1

        if mass != 0:
            self.cma /= mass
            self.cmr /= mass
            self.is_visible = True
        
        self.get_logger().info(f'cma: {self.cma}, cmr: {self.cmr}')

        self.pub.publish(Bool(data = self.is_visible))

def main(args=None):
    rclpy.init(args=args)
    node = DuckLocator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    