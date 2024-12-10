from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler
import numpy as np


class DynamicTF2Broadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf2_broadcaster')
        self.broadcaster = TransformBroadcaster(self)

        # Timer for broadcasting odom -> base_link
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

        # Initialize odom -> base_link
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.prev_time = self.get_clock().now()

        # TF Buffer and Listener for querying map -> odom
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to cmd_vel for odom -> base_link (Velocity command)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.odom_callback,
            10
        )

    def broadcast_transforms(self):
        # Broadcast map -> odom
        self.broadcast_map_to_odom()

        # Broadcast odom -> base_link
        self.broadcast_odom_to_base_link()

    def broadcast_map_to_odom(self):
        try:
            # Query map -> odom transform from TF2
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            t = TransformStamped()
            t.header.stamp = transform.header.stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'

            # Set the translation
            t.transform.translation.x = transform.transform.translation.x
            t.transform.translation.y = transform.transform.translation.y
            t.transform.translation.z = 0.0

            # Set the rotation
            t.transform.rotation = transform.transform.rotation

            # Broadcast the transform
            self.broadcaster.sendTransform(t)

        except Exception as e:
            # Warn only once to avoid cluttering the console
            if not hasattr(self, 'warned_about_map') or not self.warned_about_map:
                self.get_logger().warn(f"Failed to lookup map -> odom transform: {e}")
                self.warned_about_map = True

    def broadcast_odom_to_base_link(self):
        # Create odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set the translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convert theta (yaw) to quaternion
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(t)

    def odom_callback(self, msg):
        # Calculate delta time
        current_time = self.get_clock().now()
        delta_t = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # Get the velocity commands from /cmd_vel
        self.vx = msg.linear.x  # Velocity in x
        self.vy = msg.linear.y  # Velocity in y
        self.omega = msg.angular.z  # Angular velocity

        # Update the robot's position and orientation
        self.theta += self.omega * delta_t
        self.x += (self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta)) * delta_t
        self.y += (self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta)) * delta_t


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTF2Broadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
