import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Twist, PoseStamped
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
        # 0x06: 全方位
        # 0x07: y方向ackerman
        # 0x04: 平移s
        # 0x03: 旋轉 
        # 0x01: 全向
        # 0x00: stop 
        ##############################
        # Initial control mode is 全方位
        self.ser.write(self.mode.getMode(self.control_code))
        self._runbytes = Runbytes(0, 0, 0, 0, 0, 0, 0, 0, 0)
        ## Define the variables for the velocity and acceleration and distance
        self.vx = 0
        self.vy = 0
        self.w = 0
        self.acc = 0
        self.distance_x = 0
        self.distance_y = 0
        self.circular_ang = 0
        self.circular_radius = 0
        self.arc_distance = 0

        self.goal_pos = None
        self.spin_mode = True
        self.angle_err = 0
        self.pcl_pose_ = PoseStamped()
        self.fine_localization = Bool()
        self.fine_localization.data = True
        self.mode_switch = False
        self.navigation_mode = False
        self.NewStart = False


        ## Define the flag for mode and status
        self.current_state = 0 

        ## Define the subscriber for the command velocity (type Twist)
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback, #Changed This to the joy for easy testing
            10
        )
        self._cmd_vel_sub
        ## Define the subscriber for the cmd_vel_joy (type Twist)
        self._cmd_vel_joy_sub = self.create_subscription(
            Twist,
            'cmd_vel_joy',
            self.cmd_vel_joy_callback,
            10
        )
        self._cmd_vel_joy_sub

        ## Define the subscriber for control mode (type Int8)
        self._control_mode_sub = self.create_subscription(
            Int8,
            'control_mode',
            self.control_mode_callback,
            10
        )

        ## Define the subscriber for the status of the forklift (type int8)
        self._state_sub = self.create_subscription(
            Int8,
            'state',
            self.state_callback,
            10
        )
        self._state_sub

        ## This topic is to check that whether navigation task is done
        self._fine_localization_sub = self.create_subscription(
            Bool,
            '/fine_localization',
            self.fine_localization_callback,
            10
        )

        ## This is to check if AGV is in reasonable position (Useless)
        self._pcl_pose_sub = self.create_subscription(
            PoseStamped,
            '/pcl_pose',
            self.pose_callback,
            10
        )

        ## This topic is to check that whether navigation task is done
        self._fine_localization_sub = self.create_subscription(
            Bool,
            '/fine_localization',
            self.fine_localization_callback,
            10
        )
        ## This is to pub the goal pose to the navigation
        self._goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        ##Define the publisher for the control mode (type Int8)
        self.pub_control_mode = self.create_publisher(Int8, 'control_mode', 10)
        ## Define the timer for checking the connection
        self.connection_callback_timer = self.create_timer(1.0, self.connection_callback)
        ## Define the timer for checking the control
        self.control_callback_timer = self.create_timer(0.010, self.control_callback)
        ##timer for checking serial callback
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting:
            self.get_logger().info("Received data from teensy.")
            if b'\xa1' in self.ser.read(self.ser.in_waiting):
                self.mode_switch = True
    
    def open_serial(self):
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200)
                self.ser.timeout = 0.01
                self.ser.write(bytes([0x23, 0x01, 0x60, 0x06, 0x01, 0x00, 0xC8, 0x00, 0x64, 0x05, 0x39, 0xA8])) ##open ultrasonic
                # self.ser.write(bytes([0x23, 0x01, 0x60, 0x06, 0x00, 0x00, 0xC8, 0x00, 0x64, 0x05, 0x38, 0x79])) ##close ultrasonic
            except serial.SerialException as e:
                self.get_logger().error("Error opening serial port: {}".format(str(e)))
                raise

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def cmd_vel_joy_callback(self, msg):
        if self.current_state == 0:
            self.vx = int(msg.linear.x * 1000)
            self.vy = int(msg.linear.y * 1000)
            self.w = int(msg.angular.z * 180 / math.pi *10)
            # print('x = ', self.vx)
            # print('y = ', self.vy)
            # print('w = ', self.w)
            # print("vx = ", self.vx)
            # print("vy = ", self.vy)
            self.acc = int(600)
        else:
            temp = Twist()
            temp.linear.x = msg.linear.x
            temp.linear.y = msg.linear.y
            temp.angular.z = msg.angular.z

    def state_callback(self, msg):
        ## Callback function for the status
        ## This callback function is to check the current workflow of the AGV
        #######################
        # State definition
        # 0: Joystick control (on)
        # 1: Navigation control
        #######################
        if msg.data is not None:
            if msg.data in [0, 1]:
                self.current_state = msg.data
                if self.current_state == 0:
                    self.get_logger().info("Status: Joystick control")
                else:
                    self.get_logger().info("Status: Navigation control")

            if self.current_state == 1:
                self.pub_control_mode.publish(Int8(data=6))

    def cmd_vel_callback(self, msg):
        if self.current_state == 1:
                self.vx = int(msg.linear.x * 1000)
                self.vy = int(msg.linear.y * 1000)
                self.w = int(msg.angular.z * 180 / math.pi*10)
                #self._logger.info("Velocity keyboard Command vx = {}, vy = {}, w = {}, current_state = {}, navigation_mode = {}".format(self.vx, self.vy, self.w,self.current_state, self.navigation_mode))
        else:
            temp = Twist()
            temp.linear.x = msg.linear.x
            temp.linear.y = msg.linear.y
            temp.angular.z = msg.angular.z
            #self._logger.info("Velocity keyboard Command Else vx = {}, vy = {}, w = {}, current_state = {}, navigation_mode = {}".format(self.vx, self.vy, self.w,self.current_state, self.navigation_mode))

    def control_mode_callback(self, msg):
        ## Callback function for the control mode
        ## This callback function is to change the control mode of the AGV
        #######################
        # control_code definition
        # 0x06: 全方位 -> 6
        # 0x07: y方向ackerman -> 7
        # 0x04: 平移 -> 4
        # 0x03: 旋轉 -> 3
        # 0x01: 全向 -> 1
        # 0x00: stop -> 0
        #######################
        if msg.data is not None:     
            if msg.data == 0:
                self.control_code = 0x00
                self.get_logger().info("Control mode: stop")
            elif msg.data == 1:
                self.control_code = 0x01
                self.get_logger().info("Control mode: 全向")
            elif msg.data == 3:
                self.control_code = 0x03
                self.get_logger().info("Control mode: 旋轉")
            elif msg.data == 4:
                self.control_code = 0x04
                self.get_logger().info("Control mode: 平移")
            elif msg.data == 6:
                self.control_code = 0x06
                self.get_logger().info("Control mode: 全方位")
            elif msg.data == 7:
                self.control_code = 0x07
                self.get_logger().info("Control mode: y方向ackerman")
            change_code = self.mode.getMode(self.control_code)
            self.ser.write(change_code)

    def connection_callback(self):
        if not self.ser or not self.ser.is_open:
            try:
                self.open_serial()
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.ser.write(self.openinstruct.to_byte())
                self.get_logger().info("Connected to teensy.")
            except serial.SerialException as e:
                self.get_logger().error("Error communicating with teensy: {}".format(str(e)))
                self.close_serial()

    def SendCommand(self):
        self._runbytes.vx = self.vx
        self._runbytes.vy = self.vy
        self._runbytes.w = self.w
        self._runbytes.acc = self.acc
        self._runbytes.heartbeat = (self._runbytes.heartbeat + 1)%256
        self._runbytes.distance_x = self.vx
        self._runbytes.distance_y = self.vy
        self._runbytes.radius = self.circular_radius
        self._runbytes.arc_distance = self.arc_distance
        #self._logger.info("vx = {}, vy = {}, w = {}".format(self.vx, self.vy, self.w))

        if (self.control_code == 0x06 or self.control_code == 0x01):
            if (self._runbytes.vx == 0 and self._runbytes.vy != 0):
                self._runbytes.vy = 0
            if (self._runbytes.vx != 0 and self._runbytes.vy != 0):
                if ((math.atan2(abs(self._runbytes.vy), abs(self._runbytes.vx))* 180 / math.pi) > 30.0):
                    if (self._runbytes.vy > 0):
                        self._runbytes.vy = int(abs(self._runbytes.vx) * math.tan(math.pi/6))
                    else:
                        self._runbytes.vy = int(-1.0 * abs(self._runbytes.vx) * math.tan(math.pi/6))
        send_data = self._runbytes.to_byte(self.control_code)
        try:
            if self.control_code == 0x06:
                self.ser.write(send_data)
            if self.control_code == 0x04:
                self.ser.write(send_data)
        except serial.SerialException as e:
            self.get_logger().error("Error communicating with teensy: {}".format(str(e)))
            self.close_serial()

    def control_callback(self):
        if not self.ser.is_open:
            return
        if self.current_state == 0:
            self.SendCommand()
        elif self.current_state == 1:
            #### Navigation Mode ####
            if self.goal_pos is not None and self.NewStart:
                if self.spin_mode:
                    self.spin()
                    self.spin_mode = False
                elif self.navigation_mode:
                    if self.fine_localization.data == False:
                       self.SendCommand()
                    else:
                        self.navigation_mode = False
                        self.spin_mode = True
                        self.NewStart = False
                        self.spin()
                        self.goal_pos = None
                        
    def fine_localization_callback(self, msg):
        temp = msg.data
        if temp == self.fine_localization.data or temp == None:
            #do nothing
            pass
        else:
            self.fine_localization.data = temp
            self.get_logger().info("Fine localization: {}".format(self.fine_localization.data))

    def pose_callback(self, msg):
        self.pcl_pose_.pose.position.x = msg.pose.position.x
        self.pcl_pose_.pose.position.y = msg.pose.position.y
        self.pcl_pose_.pose.position.z = msg.pose.position.z
        self.pcl_pose_.pose.orientation.x = msg.pose.orientation.x
        self.pcl_pose_.pose.orientation.y = msg.pose.orientation.y
        self.pcl_pose_.pose.orientation.z = msg.pose.orientation.z
        self.pcl_pose_.pose.orientation.w = msg.pose.orientation.w
        self.caculate_angle_error(self.pcl_pose_.pose.orientation.x, self.pcl_pose_.pose.orientation.y, self.pcl_pose_.pose.orientation.z, self.pcl_pose_.pose.orientation.w)

    def spin(self):
        self.get_logger().info("Spin")
        mode = Int8()
        mode.data = 3
        self.pub_control_mode.publish(mode)
        self.pub_control_mode.publish(mode)
        self.pub_control_mode.publish(mode)
        self.get_logger().info("Control mode: Spin")
        while not self.mode_switch:
            time.sleep(0.1)
        self.get_logger().info("Mode switch detected, starting spin maneuvers.")
        while self.angle_err > 10 or self.angle_err < -10:
            self._runbytes.w = int(self.angle_err)*10 ## 0.1 degree
            send_data = self._runbytes.to_byte(self.control_code) 
            delay = abs(self._runbytes.w / 67.523)+3 ## 67.523 is the angular velocity of the forklift
            self.ser.write(send_data)
            print('delay =', delay)
            time.sleep(delay)
        self.mode_switch = False
        self.navigation_mode = True
        self.fine_localization.data = False
        
    def caculate_angle_error(self):
        if self.goal_pos is not None and self.fine_localization.data == True:
            target_x = self.goal_pos.pose.position.x
            target_y = self.goal_pos.pose.position.y
            current_x = self.pcl_pose_.pose.position.x
            current_y = self.pcl_pose_.pose.position.y
            theda1 = euler_from_quaternion([self.pcl_pose_.pose.orientation.x, self.pcl_pose_.pose.orientation.y, self.pcl_pose_.pose.orientation.z, self.pcl_pose_.pose.orientation.w])[2] * 180 / math.pi
            if theda1 < 0:
                theda1 += 360
            theda2 = math.atan2(target_y - current_y, target_x - current_x) * 180 / math.pi
            if theda2 < 0:
                theda2 += 360
            theda = theda2 - theda1 
            if theda >= 180:
                theda = theda - 360
            if theda <= -180:
                theda = theda + 360
            self.angle_err = theda
            print('target (x, y) =', target_x,',', target_y)
            print('current (x, y) =', current_x,',', current_y)
            print('theda 1 (euler_from_quaternion)=', theda1 ,'deg')
            print('theda 2 (atan2)=', theda2, 'deg')
            print('total = ', theda)
            print('---------------------------------')

    def goal_pose_callback(self, msg):
        self.goal_pos = PoseStamped()
        self.goal_pos.pose.position.x = msg.pose.position.x
        self.goal_pos.pose.position.y = msg.pose.position.y
        self.goal_pos.pose.position.z = msg.pose.position.z
        self.goal_pos.pose.orientation.x = msg.pose.orientation.x
        self.goal_pos.pose.orientation.y = msg.pose.orientation.y
        self.goal_pos.pose.orientation.z = msg.pose.orientation.z
        self.goal_pos.pose.orientation.w = msg.pose.orientation.w
        self.NewStart = True
        self.get_logger().info("Goal pose received.")
    
    def pose_callback(self, msg):
        self.pcl_pose_.pose.position.x = msg.pose.position.x
        self.pcl_pose_.pose.position.y = msg.pose.position.y
        self.pcl_pose_.pose.position.z = msg.pose.position.z
        self.pcl_pose_.pose.orientation.x = msg.pose.orientation.x
        self.pcl_pose_.pose.orientation.y = msg.pose.orientation.y
        self.pcl_pose_.pose.orientation.z = msg.pose.orientation.z
        self.pcl_pose_.pose.orientation.w = msg.pose.orientation.w
        self.caculate_angle_error(self.pcl_pose_.pose.orientation.x, self.pcl_pose_.pose.orientation.y, self.pcl_pose_.pose.orientation.z, self.pcl_pose_.pose.orientation.w)

def main(args=None):
    print('Hi from forklift.')
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
