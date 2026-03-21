from enum import Enum
import math
import rclpy

import time

from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool

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
    WAIT_TIME = 1000

class TSPA(Node):
    def __init__(self):
        super().__init__('tspa')

        self.gps_sub = [PoseStamped, '/image_recalib_apriltag_pose', self.gps_callback, 10]
        self.duck_sub = [PoseStamped, '/goal_pose', self.duck_callback, 10]
        self.lidar_sub = [LaserScan, '/scan', self.scan_callback, 10]
        self.duck_type_sub = [Int32, '/duck_type', self.duck_type_callback, 10]
        self.pfield_sub = [PoseStamped, '/pfield_pose', self.pzone_callback, 10]
        self.nfield_sub = [PoseStamped, '/nfield_pose', self.nzone_callback, 10]

        self.beh2sub = {
            Behaviour.WAIT_TARGET: [
                self.duck_sub
            ],
            Behaviour.GO_TO_TARGET: [
                self.duck_sub,
                self.gps_sub,
            ],
            Behaviour.DOCK_WITH_DUCK: [
                self.lidar_sub,
            ],
            Behaviour.GRAB_THE_DUCK: [
                self.duck_type_sub,
                self.lidar_sub
            ],
            Behaviour.GO_TO_PBASE: [
                self.gps_sub,
                self.pfield_sub
            ],
            Behaviour.GO_TO_NBASE: [
                self.gps_sub,
                self.nfield_sub
            ],
            Behaviour.DROP_THE_DUCK: [
            ],
            Behaviour.WAIT_TIME: [],
        }

        # https://stackoverflow.com/questions/334655/passing-a-dictionary-to-a-function-as-keyword-parameters#334666
        # self.gps_sub = self.create_subscription(**gps_sub)

        self.current_behaviour = Behaviour.WAIT_TARGET

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_loop)

        self.duck_sensor = DuckSensor(DuckType.NOT_A_DUCK)
        self.duck_locator = DuckLocator(0, 0, False)
        self.robot_pose = None
        self.duck_pose = None
        self.baze_pose = None
        self.enemy_baze_pose = None

        self.twist = Twist()
        self.gripper = Bool()
        self.timestate = None
        self.timelastbeh = None

        self.err_theta = None
        self.mas_give_cam = None
        self.duck_pos_number = None

        self.robot_theta = 0

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper', 10)
    
        self.current_subs = []

    def give_best_by_first(self):
        for i in range(8):
            if self.mas_give_cam[i] <= 4 and self.mas_give_cam[i] != 0:
                self.duck_pos_number = i
                self.mas_give_cam[i] = 0
                break

    def scan_callback(self, msg):
        """Lidar callback [LaserScan]"""
        lidar_offset = -83.0 * math.pi / 180
        angle_par = 45.0 * math.pi / 180
        dist_par = 35.0 / 100
        dist_par_low = 18.0 / 100
        angle_otbros = math.pi/9

        manip_angles = [
            0.3,
            0.4
        ]
        pc = []

        angle_min_out = msg.angle_min
        angle_max_out = msg.angle_max
        angle_increment = msg.angle_increment
        pc = msg.ranges

        self.duck_locator.cma = 0
        self.duck_locator.cmr = math.inf
        self.duck_locator.is_visible = False
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
                if manip_angles[0] < abs(angle) < manip_angles[1]:
                    continue
                # self.get_logger().info(f'{idx}, {p}, {angle}, {distance}')
                # self.duck_locator.cma += angle
                # self.duck_locator.cmr += distance
                # mass += 1
                if distance < self.duck_locator.cmr:
                    self.duck_locator.cmr = distance
                    self.duck_locator.cma = angle
                    mass = 1

        if mass != 0:
            self.duck_locator.cma /= mass
            self.duck_locator.cmr /= mass
            self.duck_locator.is_visible = True
        
        if self.duck_locator.cmr > dist_par:
            self.duck_locator.cmr = dist_par
        
        self.robot_theta = 0.0

        # self.get_logger().info(f'cma: {self.duck_locator.cma}, cmr: {self.duck_locator.cmr}')
        pass

    def gps_callback(self, msg):
        """Gps callback [PoseStamped]"""
        if self.robot_pose is None: 
            self.robot_pose = Pose(0, 0, 0)

        self.robot_pose.x = msg.pose.position.x
        self.robot_pose.y = msg.pose.position.y
        self.robot_pose.theta = math.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2
        pass

    def duck_callback(self, msg):
        """Duck position callback [PoseStamped]"""
        # self.ducks_pub = self.create_publisher(Pose, "/duck_target", 10)

        if self.duck_pose is None:
            self.duck_pose = Pose(0, 0, 0)
        
        self.duck_pose.x = msg.pose.position.x
        self.duck_pose.y = msg.pose.position.y
        self.duck_pose.theta = math.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2

        self.get_logger().info(f'Current duck pose: {self.duck_pose}')
        pass

    def pzone_callback(self, msg):
        """Our base position callback [PoseStamped]"""

        if self.baze_pose is None:
            self.baze_pose = Pose(0, 0, 0)
            
        self.baze_pose.x = msg.pose.position.x
        self.baze_pose.y = msg.pose.position.y
        self.baze_pose.theta = math.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2
        pass

    def nzone_callback(self, msg):
        """Enemy base position callback [PoseStamped]"""

        if self.enemy_baze_pose is None:
            self.enemy_baze_pose = Pose(0, 0, 0)

        self.enemy_baze_pose.x = msg.pose.position.x
        self.enemy_baze_pose.y = msg.pose.position.y
        self.enemy_baze_pose.theta = math.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2
        pass

    def duck_type_callback(self, msg):
        """Duck type callback [int]
        -1 - no value from duck scanner
        0 - no duck
        1 - good duck
        2 - bad duck
        """
        self.duck_sensor(DuckType(msg.data))
        pass
    
    def set_behaviour(self, behaviour):
        clock = self.get_clock()
        currenttime = clock.now()
        self.timelastbeh = currenttime.nanoseconds / 1e9
        self.timestate = 0
        #self.timelastbeh = self.get_clock().nanoseconds() / 1e9
        # https://stackoverflow.com/questions/38487816/unsubscribing-from-ros-topic-python
        # for sub in self.current_subs:
        #     sub.shutdown()
        self.current_subs = []

        # self.duck_sensor = None
        # self.duck_locator = None
        # self.robot_pose = None
        # self.duck_pose = None
        # self.baze_pose = None
        # self.enemy_baze_pose = None

        self.current_behaviour = behaviour

        for subd in self.beh2sub[behaviour]:
            self.current_subs.append(self.create_subscription(*subd))

    def goToPose(self, pose: Pose):

        if pose is None:
            return
        if self.robot_pose is None:
            return

        anglereg = 0.0
        v = 0.0
        distotcl = 0.1
        k_forward = 0.58
        k_turn = 0.4
        maxline = 0.1
        maxz = 0.5

        dx = pose.x - self.robot_pose.x
        dy = pose.y - self.robot_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        desired_angle = math.atan2(dy, dx)
        angle_errorToFr = desired_angle - self.robot_pose.theta
        angle_errorToFr = math.atan2(math.sin(angle_errorToFr), math.cos(angle_errorToFr))

        # desired_angle = pose.theta
        angle_error = desired_angle - self.robot_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_errorToFr) > 0.2 and abs(distance) > distotcl:
            amglereg = angle_errorToFr
        elif abs(distance) > distotcl:
            v = distance
            amglereg = angle_errorToFr
        elif abs(angle_error) > 0.1:
            amglereg = angle_error
        else:
            v = 0
            amglereg = 0

        self.err_theta = abs(amglereg)
        twist = Twist()

        twist.linear.x = k_forward * v
        if twist.linear.x > maxline:
            twist.linear.x = maxline

        twist.angular.z = k_turn * amglereg
        if twist.angular.z > maxz:
            twist.angular.z = maxz
        elif twist.angular.z < -maxz:
            twist.angular.z = -maxz

        self.twist = twist

    def object_docking(self):
        twist = Twist()
        v = 0.0
        theta_turn = 0.0
        gripper = Bool()
        gripper.data = False
        const_dist_gripper = 25.0 / 100
        porog_turn = math.pi / 18
        porog_dist = 2.0 / 100
        k_forward = 1.0
        k_turn = 0.6
        max_v = 0.1
        max_w = 0.5

        dtheta = self.duck_locator.cma - self.robot_theta
        ddist = self.duck_locator.cmr - const_dist_gripper
        if abs(self.duck_locator.cma) > porog_turn:
            v = 0.0
            theta_turn = k_turn * dtheta
        else:
            v = k_forward * ddist
            theta_turn = k_turn * dtheta
        
        if abs(dtheta) < (porog_turn / 2) and abs(ddist) < porog_dist:
            gripper.data = True

        if self.duck_locator.cmr < 0.1:
            v = 0.0

        v = min(max(v, -max_v), max_v)
        theta_turn = min(max(theta_turn, -max_w), max_w)

        # theta_turn_int = theta_turn if abs(theta_turn) > 0.3 else 0
        theta_turn_int = theta_turn

        self.robot_theta += theta_turn_int * self.timer_period

        twist.linear.x = v
        twist.angular.z = theta_turn

        self.twist = twist
        self.gripper = gripper

    def main_loop(self):
        clock = self.get_clock()
        currenttime = clock.now()
        self.timestate = currenttime.nanoseconds / 1e9 - self.timelastbeh

        if self.current_behaviour == Behaviour.WAIT_TARGET:
            self.twist = Twist()
            self.get_logger().info('Waiting for target')
        elif self.current_behaviour == Behaviour.GO_TO_TARGET:
            self.goToPose(self.duck_pose)
            self.get_logger().info(f'Going to target {self.duck_pose}')
        elif self.current_behaviour == Behaviour.DOCK_WITH_DUCK:
            self.object_docking()
            self.get_logger().info(f'Docking with duck {self.duck_locator}')
        elif self.current_behaviour == Behaviour.GRAB_THE_DUCK:
            self.twist = Twist()
            self.get_logger().info('Grabbing the duck')
        elif self.current_behaviour == Behaviour.GO_TO_PBASE:
            self.goToPose(self.baze_pose)
            self.get_logger().info('Going to pbase')
        elif self.current_behaviour == Behaviour.GO_TO_NBASE:
            self.goToPose(self.enemy_baze_pose)
            self.get_logger().info('Going to nbase')
        elif self.current_behaviour == Behaviour.DROP_THE_DUCK:
            self.twist = Twist()
            self.twist.linear.x = -0.08
            self.gripper.data = False
            self.get_logger().info('Dropping the duck')
        elif self.current_behaviour == Behaviour.WAIT_TIME:
            self.twist = Twist()
            self.get_logger().info(f'Waiting... {self.timestate}')

        self.get_logger().info(f'Current subs: {self.current_subs}')
        self.get_logger().info(f'GPS: {self.robot_pose}, Control: {self.twist}, {self.gripper}')

        self.twist_pub.publish(self.twist)
        self.gripper_pub.publish(self.gripper)

def run_behaviour(node, behaviour, until = lambda: False):
    node.set_behaviour(behaviour)
    while rclpy.ok():
        rclpy.spin_once(node)
        if until():
            break

def dist(p1, p2):
    if p1 is None or p2 is None:
        return math.inf
    return abs( \
        math.sqrt( \
            math.pow( \
                p1.x - p2.x, 2 \
            ) + math.pow( \
            p1.y - p2.y, 2 \
            ) \
        ) \
    )
    

cordination_ducks = [
    # Pose(0.336994, 0.363888, 2.27), # 1
    # Pose(0.336994, 0.363888, 0.61), # 2
    # Pose(0.835612, 0.320223, 2.35), # 3
    # Pose(0.835612, 0.320223, 0.61), # 4
    # Pose(0.359710, 0.623648, 2.15), # 5
    # Pose(0.354694, 0.621288, 1.03), # 6
    # Pose(0.870093, 0.613370, 2.11), # 7
    # Pose(0.891206, 0.633751, 0.97), # 8
    # Pose(0.366035, 0.757009, 2.00), # 9
    # Pose(0.338126, 0.750289, 1.15), # 10
    # Pose(0.852274, 0.743161, 1.11), # 11
    # Pose(0.891469, 0.749518, 2.03), # 12
    # Pose(0.858826, 1.164620, 2.47), # 13
    # Pose(0.853082, 1.129030, 0.67), # 14
    # Pose(0.621408, 1.106540, 2.21), # 15
    # Pose(0.344352, 1.145580, 2.23)  # 16
    Pose(0.25, 0.5, 2.27), # 1
    Pose(0.5, 0.5, 0.61), # 2
    Pose(0.75, 0.5, 2.35), # 3
    Pose(1, 0.5, 0.61), # 4
    Pose(0.3, 0.75, 2.15), # 5
    Pose(0.5, 0.75, 1.03), # 6
    Pose(0.75, 0.75, 2.11), # 7
    Pose(1, 0.75, 0.97), # 8
    Pose(0.366035, 0.757009, 2.00), # 9
    Pose(0.338126, 0.750289, 1.15), # 10
    Pose(0.852274, 0.743161, 1.11), # 11
    Pose(0.891469, 0.749518, 2.03), # 12
    Pose(0.858826, 1.164620, 2.47), # 13
    Pose(0.853082, 1.129030, 0.67), # 14
    Pose(0.621408, 1.106540, 2.21), # 15
    Pose(0.344352, 1.145580, 2.23)  # 16
]

cordination_baze = []

baze1 = Pose(0.869143, 0.136321, 0.00)
baze2 = Pose(1.142820, 0.403740, -1.62)

bazepos_centre = Pose(1.1, 0.15, -0.81)

for i in range(8):
    cordination_baze.append(Pose(baze1.x, 0.15 + 0.027 * (i + 1), baze1.theta))

for i in range(8):
    cordination_baze.append(Pose(1.25 - (0.027 * (i + 1)), baze2.y, baze2.theta))

def mirror_cordination(cordination: Pose, need: bool):
    if not need:
        return cordination
    else:
        return Pose(cordination.x, 1.75 - cordination.y, -cordination.theta)

robot_gotopoint_dist_threshold = 0.24
is_A_baze = False
# is_A_baze = True

def main(args=None):
    rclpy.init(args=args)
    node = TSPA()
    count_what_duck = 0
    try:
        while rclpy.ok():
            run_behaviour(node, Behaviour.WAIT_TARGET, until = lambda: node.mas_give_cam is not None)
            node.give_best_by_first()
            node.duck_pose = mirror_cordination(cordination_ducks[node.duck_pos_number], is_A_baze)
            run_behaviour(node, Behaviour.GO_TO_TARGET, until = lambda: \
                dist(node.robot_pose, node.duck_pose) < robot_gotopoint_dist_threshold and \
                    node.err_theta < 0.6)

            for i in range(3):
                run_behaviour(node, Behaviour.WAIT_TIME, until = lambda: node.timestate > 1)

                run_behaviour(node, Behaviour.DOCK_WITH_DUCK, until = lambda: node.gripper.data == True)

                run_behaviour(node, Behaviour.WAIT_TIME, until = lambda: node.timestate > 1)

                node.set_behaviour(Behaviour.GRAB_THE_DUCK)
                rclpy.spin_once(node)
                if abs(node.duck_locator.cma) > 0.1 and abs(node.duck_locator.cmr) > 0.1 or not node.duck_locator.is_visible:
                    break

            # run_behaviour(node, Behaviour.WAIT_TIME, until = lambda: node.timestate > 5)
            #     # if node.duck_sensor.duck_type == DuckType.GOOD_DUCK or \
            #     #    node.duck_sensor.duck_type == DuckType.BAD_DUCK:
            #     #     break
            
            # # if node.duck_sensor.duck_type == DuckType.GOOD_DUCK:
            #node.baze_pose = mirror_cordination(cordination_baze[0], is_A_baze)
            node.baze_pose = mirror_cordination(bazepos_centre, is_A_baze)
            run_behaviour(node, Behaviour.GO_TO_PBASE, until = lambda: \
                dist(node.robot_pose, node.baze_pose) < robot_gotopoint_dist_threshold and \
                    node.err_theta < 0.6)

            # # elif node.duck_sensor.duck_type == DuckType.BAD_DUCK:
            # #     run_behaviour(node, Behaviour.GO_TO_NBASE, until = lambda: \
            # #         dist(node.robot_pose, node.enemy_baze_pose) < 0.1 and \
            # #         abs(node.robot_pose.theta - node.enemy_baze_pose.theta) < 0.1)
            # # else:
            # #     continue

            run_behaviour(node, Behaviour.WAIT_TIME, until = lambda: node.timestate > 1)
            run_behaviour(node, Behaviour.DROP_THE_DUCK, until = lambda: node.timestate > 2)
            run_behaviour(node, Behaviour.WAIT_TIME, until = lambda: node.timestate > 1)

            count_what_duck += 1
            # break

    
    except KeyboardInterrupt:
        run_behaviour(node=node, behaviour=Behaviour.WAIT_TIME, until=lambda: node.timestate > 1)
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
