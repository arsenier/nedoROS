#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
import struct
import time
from typing import Optional
import serial
import math
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat
import tf2_ros
port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_12350481010616918465-if00"
baudrate = 115200

CONTROL_HEADER = 0x01
TX_PACKET_SIZE = 1 + 4 + 4 + 1 + 1  # 11 bytes
RX_PACKET_SIZE = 1 + 4 + 4 + 4 + 1 + 1 + 1  # 16 bytes
def xor_checksum(data: bytes) -> int:
    checksum = 0
    for b in data[1:]:
        checksum ^= b
    return checksum

ser = serial.Serial(port, baudrate=baudrate, timeout=1)
time.sleep(5)
ser.reset_input_buffer()
print("Serial initializated succesfully")


def send_speeds(left_motor_speed, right_motor_speed, gripper):
    ans: Optional[tuple[float, float, float, bool, bool]] = None

    args: list[float] = [
        CONTROL_HEADER,
        int(left_motor_speed),
        int(right_motor_speed),
        int(gripper),
    ]

    checksum = xor_checksum(struct.pack("<BffB", *args))
    args.append(checksum)
    command = struct.pack("<BffBB", *args)

    try:
        time.sleep(0.1)

        ser.write(command)
        print(f"Sent: {command}")

        header = ser.read(1)
        if len(header) == 0:
            print("Timeout: no response")
            return None
        while len(header) == 1 and header[0] != 0x01:
            print(f"{hex(header[0])} {header}")
            header = ser.read(1)
            if len(header) == 0:
                print("Timeout waiting for 0x01")
                return None
                    
        response = ser.read(RX_PACKET_SIZE-1)
        if len(response) != RX_PACKET_SIZE-1:
            print("Invalid response size:", len(response))
            return None
        
        # # print(f"Response: {response}")
        # print("Response: ", end="")
        # for b in response:
        #     print(f"{(b)} ", end="")
        # print()

        checksum = 0
        for byte in response:
            checksum ^= byte

        if checksum != 0:
            print("Invalid checksum:", checksum)
            return None
        
        # response = bytearray(response)
        # response.insert(0, 0x01)
        # print(f"Resp: {response}, resplen: {len(response)}")

        ans = struct.unpack("<fffBBB", response)

        print("Received:")
        print("  x =", ans[0])
        print("  y =", ans[1])
        print("  theta =", ans[2])
        print("  left_usik =", bool(ans[3]))
        print("  right_usik =", bool(ans[4]))

        # response = bytearray(response)
    except serial.SerialException as e:
        print("Serial error:", e)

    # finally:
    #     if ser is not None and ser.is_open:
    #         ser.close()

    return ans

class Driver(Node):

    def __init__(self):
        self.fw = 0
        self.ang = 0
        self.gripper = False
        super().__init__('Driver')
        self.subscriber1 = self.create_subscription(Twist, '/cmd_vel', self.update_data_driver, 10)
        self.subscriber2 = self.create_subscription(Bool, '/gripper', self.update_data_gripper, 10)
        self.usik_right = self.create_publisher(Bool, '/usik_right', 10)
        self.usik_left = self.create_publisher(Bool, '/usik_left', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.is_start = self.create_subscription(Bool, '/start', self.check_start, 10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.update_data_robot)
        self.data_usl = Bool()
        self.data_usr = Bool()
        self.started = False
        self.start_time = 0.0
        self.timeG = -2

    def check_start(self, msg):
        if msg.data and not self.started:
            self.started = True
            self.start_time = time.time()    


    def update_data_driver(self, msg):
        self.fw = msg.linear.x
        self.ang = msg.angular.z
    def update_data_gripper(self, msg):
        self.gripper = msg.data
    
    def update_data_robot(self):
        if not self.started:
            return 
        if time.time()-self.start_time > 90:
            send_speeds(0, 0, 0)
            return

        # fw (-1-1), ang(-pi, pi)
        R_robot = 0.08
        R_wheel = 0.04

        #kf = 50/0.25 # [popugov/(m/s)]

        vl = (self.fw - self.ang*R_robot) / R_wheel
        vr = (self.fw + self.ang*R_robot) / R_wheel
        # v = self.fw * kf
        # u = self.ang * R_robot * kf
        if time.time() - self.timeG < 2:
            ans = send_speeds(vl, vr * 1.25, self.gripper)
        else:
            ans = send_speeds(-1, -1, self.gripper)

        if ans is None:
                return

        self.data_usl.data = bool(ans[3])
        self.data_usr.data = bool(ans[4])
        self.usik_left.publish(self.data_usl)
        self.usik_right.publish(self.data_usr)
        if bool(ans[3]) or bool(ans[4]):
            self.timeG = time.time()

        #____________________ODOM_______________

        x = float(ans[0])
        y = float(ans[1])
        theta = float(ans[2])

        now = self.get_clock().now()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        quat = euler2quat(0, 0, theta)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]

        self.pub_odom.publish(odom)


        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        self.odom_broadcaster.sendTransform(t)

    


        



def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)

    # По завершении
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
