#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TF2Broadcaster(Node):
    def __init__(self):
        super().__init__('tf2_broadcaster')
        # Create a TransformBroadcaster object
        self.tf_broadcaster = TransformBroadcaster(self)
        # Publish at a regular rate
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        # Create a TransformStamped message for map to base_link
        t_map_base = TransformStamped()

        # Set the header
        t_map_base.header.stamp = self.get_clock().now().to_msg()
        t_map_base.header.frame_id = 'map'

        # Set the child frame
        t_map_base.child_frame_id = 'base_link'

        # Set the translation
        t_map_base.transform.translation.x = 1.0  # Adjust based on your configuration
        t_map_base.transform.translation.y = 0.0
        t_map_base.transform.translation.z = 0.0

        # Set the rotation (no rotation, quaternion [0, 0, 0, 1])
        t_map_base.transform.rotation.x = 0.0
        t_map_base.transform.rotation.y = 0.0
        t_map_base.transform.rotation.z = 0.0
        t_map_base.transform.rotation.w = 1.0

        # Send the transform
        self.tf_broadcaster.sendTransform(t_map_base)

        # Create a TransformStamped message for base_link to lidar
        t_base_lidar = TransformStamped()

        # Set the header
        t_base_lidar.header.stamp = self.get_clock().now().to_msg()
        t_base_lidar.header.frame_id = 'base_link'

        # Set the child frame
        t_base_lidar.child_frame_id = 'laser'

        # Set the translation
        t_base_lidar.transform.translation.x = 0.2  # Adjust based on your robot's configuration
        t_base_lidar.transform.translation.y = 0.0
        t_base_lidar.transform.translation.z = 0.1

        # Set the rotation (no rotation, quaternion [0, 0, 0, 1])
        t_base_lidar.transform.rotation.x = 0.0
        t_base_lidar.transform.rotation.y = 0.0
        t_base_lidar.transform.rotation.z = 0.0
        t_base_lidar.transform.rotation.w = 1.0

        # Send the transform
        self.tf_broadcaster.sendTransform(t_base_lidar)

def main(args=None):
    rclpy.init(args=args)
    node = TF2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()