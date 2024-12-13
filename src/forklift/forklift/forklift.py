import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion
import serial
import math
import time
from .mode import Mode, Runbytes

class ForkliftSubscriber(Node):
    def __init__(self):
        super().__init__('fork_listen')
        self.ser = None
        self.open_serial()
        self.mode = Mode()
        self.control_code = 0x06

        ##############################
        # control_code definition
        # 0x06: Omnidirectional
        # 0x07: Y-direction Ackerman
        # 0x04: Translation
        # 0x03: Rotation
        # 0x01: All-direction
        # 0x00: Stop
        ##############################
        # Initial control mode is Omnidirectional
        self.ser.write(self.mode.getMode(self.control_code))
        self._runbytes = Runbytes(0, 0, 0, 0, 0, 0, 0, 0, 0)

        # Define variables for velocity, acceleration, distance
        self.vx = 0
        self.vy = 0
        self.w = 0
        self.acc = 0
        self.goal_pos = None
        self.path_received = False

        # Define flags for mode and status
        self.current_state = 0  # 0: Joystick, 1: Navigation

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_joy', self.cmd_vel_joy_callback, 10)
        self.create_subscription(Int8, 'control_mode', self.control_mode_callback, 10)
        self.create_subscription(Int8, 'state', self.state_callback, 10)
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Publishers
        self.pub_control_mode = self.create_publisher(Int8, 'control_mode', 10)

        # Timers
        self.create_timer(0.01, self.control_callback)

    def open_serial(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.01)
            self.ser.write(bytes([0x23, 0x01, 0x60, 0x06, 0x01, 0x00, 0xC8, 0x00, 0x64, 0x05, 0x39, 0xA8]))
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def cmd_vel_joy_callback(self, msg):
        if self.current_state == 0:  # Joystick control
            self.vx = int(msg.linear.x * 1000)
            self.vy = int(msg.linear.y * 1000)
            self.w = int(msg.angular.z * 180 / math.pi * 10)
            self.acc = 600

    def cmd_vel_callback(self, msg):
        if self.current_state == 1:  # Navigation mode
            self.vx = int(msg.linear.x * 1000)
            self.vy = int(msg.linear.y * 1000)
            self.w = int(msg.angular.z * 180 / math.pi * 10)

    def state_callback(self, msg):
        if msg.data is not None:
            if msg.data in [0, 1]:
                self.current_state = msg.data
                if self.current_state == 0:
                    self.get_logger().info("Status: Joystick control")
                elif self.current_state == 1:
                    self.get_logger().info("Status: Navigation control")

    def control_mode_callback(self, msg):
        if msg.data is not None:
            control_modes = {
                0: "Stop",
                1: "All-direction",
                3: "Rotation",
                4: "Translation",
                6: "Omnidirectional",
                7: "Y-direction Ackerman"
            }
            if msg.data in control_modes:
                self.control_code = msg.data
                self.ser.write(self.mode.getMode(self.control_code))
                self.get_logger().info(f"Control mode: {control_modes[msg.data]}")

    def path_callback(self, msg):
        if msg.poses:
            self.path_received = True
            self.get_logger().info("Path received from Nav2.")

    def goal_pose_callback(self, msg):
        self.goal_pos = msg
        self.get_logger().info("Goal pose received.")

    def control_callback(self):
        if self.current_state == 0:  # Joystick control
            self.SendCommand()
        elif self.current_state == 1 and self.path_received:  # Navigation mode
            self.SendCommand()

    def SendCommand(self):
        self._runbytes.vx = self.vx
        self._runbytes.vy = self.vy
        self._runbytes.w = self.w
        self._runbytes.acc = self.acc
        self._runbytes.heartbeat = (self._runbytes.heartbeat + 1) % 256

        try:
            self.ser.write(self._runbytes.to_byte(self.control_code))
        except serial.SerialException as e:
            self.get_logger().error(f"Error communicating with teensy: {e}")
            self.close_serial()


def main(args=None):
    rclpy.init(args=args)
    fork_listen = ForkliftSubscriber()
    try:
        rclpy.spin(fork_listen)
    except KeyboardInterrupt:
        pass
    finally:
        fork_listen.close_serial()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
