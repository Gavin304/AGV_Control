import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Byte, Bool, Int8
import math
import struct

class Js_subscriber(Node):
    def __init__(self):
        super().__init__('js2fork')
        self.vx = 0
        self.vy = 0
        self.w = 0
        self.acc = 0
        self._state = Int8()
        self.state_count = 0

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self._state_pub = self.create_publisher(Int8, 'state', 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.fork_control_publisher = self.create_publisher(Int8, 'fork_start', 10)
        self.control_mode_publisher = self.create_publisher(Int8, 'control_mode', 10)
        self.save_pose_sub = self.create_publisher(Bool, 'save_pose', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        ##################Init control mode###################
        self.control_mode = Int8()
        self.control_mode.data = 6
        self.control_mode_publisher.publish(self.control_mode)

    def joy_callback(self, msg):
        #### Here, you can adjust how fast you want ####
        self.vx = float(msg.axes[1]*1.0)
        self.vy = float(msg.axes[0]*0.4)
        self.w = float(msg.axes[3]*0.5)

        if msg.buttons[3] == 1:
            self.control_mode.data = 3
            self.control_mode_publisher.publish(self.control_mode)
            self.get_logger().info('Control mode: 旋轉')
        elif msg.buttons[1] == 1:
            self.control_mode.data = 6
            self.control_mode_publisher.publish(self.control_mode)
            self.get_logger().info('Control mode: 全方位')
        elif msg.buttons[2] == 1:
            self.control_mode.data = 7
            self.control_mode_publisher.publish(self.control_mode)
            self.get_logger().info('Control mode: y方向ackerman')
        elif msg.buttons[0] == 1:
            self.control_mode.data = 1
            self.control_mode_publisher.publish(self.control_mode)
            self.get_logger().info('Control mode: 全向')
        elif msg.buttons[5] == 1:
            self.control_mode.data = 4
            self.control_mode_publisher.publish(self.control_mode)
            self.get_logger().info('Control mode: 平移')
        elif msg.axes[2] == -1:
            self.control_mode.data = 0
            self.control_mode_publisher.publish(self.control_mode)
            self.get_logger().info('Control mode: 0x00 (Stop)')
        elif msg.axes[7] == 1:
            self.get_logger().info('Fork UP')
            self.fork_control_publisher.publish(Int8(data=2))
        elif msg.axes[7] == -1:
            self.fork_control_publisher.publish(Int8(data=0))
            self.get_logger().info('Fork Down')
        elif msg.axes[6] == 1:
            self.state_count += 1
            self._state.data = (self.state_count) % 2
            self._state_pub.publish(self._state)
            ######
            # 0 -> joystick
            # 1 -> navigation
            ######
            if self._state.data == 0:
                self.get_logger().info('State: Joystick mode')
            else:
                self.get_logger().info('State: Navigation mode')
        elif msg.axes[6] == -1:
            self.state_count -= 1
            self._state.data = (self.state_count) % 2
            self._state_pub.publish(self._state)
            if self._state.data == 0:
                self.get_logger().info('State: Joystick mode')
            else:
                self.get_logger().info('State: Navigation mode')
        elif msg.buttons[8] == 1:
            Bool_msg = Bool()
            Bool_msg.data = True
            self.save_pose_sub.publish(Bool_msg)
            self.get_logger().info('Save Current Position.')

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = float(self.vx)
        twist.linear.y = float(self.vy)
        twist.angular.z = float(self.w)
        self.publisher.publish(twist)

def main(args = None):
    print('This is js2fk')
    rclpy.init(args=args)
    js2fork = Js_subscriber()
    try:
        rclpy.spin(js2fork)
        # rclpy.spin(js_publisher)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
