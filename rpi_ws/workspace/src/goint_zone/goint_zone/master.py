from enum import Enum

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


class TSPA(Node):
    def __init__(self):
        super().__init__('tspa')

        self.current_behaviour = Behaviour.WAIT_TARGET

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_loop)

        self.duck_sensor = DuckSensor(DuckType.NOT_A_DUCK)
    
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