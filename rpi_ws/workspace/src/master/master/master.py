from enum import Enum
import math
import rclpy
from rclpy.node import Node

from dataclasses import dataclass

@dataclass
class Pose:
    x: float
    y: float
    theta: float

@dataclass
class DuckLocator:
    cma: float
    cmr: float
    is_visible: bool

class DuckType(Enum):
    NOT_A_DUCK = -1
    NO_DUCK = 0
    GOOD_DUCK = 1
    BAD_DUCK = 2

@dataclass
class DuckSensor:
    duck_type: DuckType

class Behaviour(Enum):
    WAIT_TARGET = 0
    GO_TO_TARGET = 1
    DOCK_WITH_DUCK = 2
    GRAB_THE_DUCK = 3
    GO_TO_PBASE = 4
    GO_TO_NBASE = 5
    DROP_THE_DUCK = 6

beh2sub = {
    Behaviour.WAIT_TARGET: [
        {"type": }
    ]
}

class TSPA(Node):
    def __init__(self):
        super().__init__('tspa')

        self.current_behaviour = Behaviour.WAIT_TARGET

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_loop)

        self.duck_sensor = DuckSensor(DuckType.NOT_A_DUCK)

        self.duck_locator = DuckLocator()

        self.current_subs = []

    def scan_callback(self, msg):
        
        """Lidar callback [LaserScan]"""
        lidar_offset = -90.0 * math.pi / 180
        angle_par = math.pi * 2 / 3
        dist_par = 50.0 / 100
        dist_par_low = 20.0 / 100
        angle_otbros = math.pi/9
        pc = []

        angle_min_out = msg.angle_min
        angle_max_out = msg.angle_max
        angle_increment = msg.angle_increment
        pc = msg.ranges

        self.duck_locator.cma = 0
        self.duck_locator.cmr = 0
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
                self.duck_locator.cma += angle
                self.duck_locator.cmr += distance
                mass += 1

        if mass != 0:
            self.duck_locator.cma /= mass
            self.duck_locator.cmr /= mass
            self.duck_locator.is_visible = True
        
        robot_theta = 0.0

        # self.get_logger().info(f'cma: {self.cma}, cmr: {self.cmr}')
        pass

    def gps_callback(self, msg):
        """Gps callback [PoseStamped]"""
        pass

    def duck_callback(self, msg):
        """Duck position callback [PoseStamped]"""
        pass

    def pzone_callback(self, msg):
        """Our base position callback [PoseStamped]"""
        pass

    def nzone_callback(self, msg):
        """Enemy base position callback [PoseStamped]"""
        pass

    def duck_type_callback(self, msg):
        """Duck type callback [int]
        -1 - no value from duck scanner
        0 - no duck
        1 - good duck
        2 - bad duck
        """
        pass
    
    def set_behaviour(self, behaviour):
        self.current_behaviour = behaviour
    
    def main_loop(self):
        if self.current_behaviour == Behaviour.WAIT_TARGET:
            self.get_logger().info('Waiting for target')
        elif self.current_behaviour == Behaviour.GO_TO_TARGET:
            self.get_logger().info('Going to target')
        elif self.current_behaviour == Behaviour.DOCK_WITH_DUCK:
            self.get_logger().info('Docking with duck')
        elif self.current_behaviour == Behaviour.GRAB_THE_DUCK:
            self.get_logger().info('Grabbing the duck')
        elif self.current_behaviour == Behaviour.GO_TO_PBASE:
            self.get_logger().info('Going to pbase')
        elif self.current_behaviour == Behaviour.GO_TO_NBASE:
            self.get_logger().info('Going to nbase')
        elif self.current_behaviour == Behaviour.DROP_THE_DUCK:
            self.get_logger().info('Dropping the duck')



def main(args=None):
    rclpy.init(args=args)
    node = TSPA()
    try:
        while rclpy.ok():
            node.set_behaviour(Behaviour.WAIT_TARGET)
            while rclpy.ok():
                rclpy.spin_once(node)
                if False:
                    break
            
            node.set_behaviour(Behaviour.GO_TO_TARGET)
            while rclpy.ok():
                rclpy.spin_once(node)
                if False:
                    break

            while True:
                node.set_behaviour(Behaviour.DOCK_WITH_DUCK)
                while rclpy.ok():
                    rclpy.spin_once(node)
                    if False:
                        break
                
                node.set_behaviour(Behaviour.GRAB_THE_DUCK)
                while rclpy.ok():
                    rclpy.spin_once(node)
                    if False:
                        break
                
                if node.duck_sensor.duck_type == DuckType.GOOD_DUCK or \
                   node.duck_sensor.duck_type == DuckType.BAD_DUCK:
                    break
            
            if node.duck_sensor.duck_type == DuckType.GOOD_DUCK:
                node.set_behaviour(Behaviour.GO_TO_PBASE)
                while rclpy.ok():
                    rclpy.spin_once(node)
                    if False:
                        break
            else:
                node.set_behaviour(Behaviour.GO_TO_NBASE)
                while rclpy.ok():
                    rclpy.spin_once(node)
                    if False:
                        break

            node.set_behaviour(Behaviour.DROP_THE_DUCK)
            while rclpy.ok():
                rclpy.spin_once(node)
                if False:
                    break

    
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()