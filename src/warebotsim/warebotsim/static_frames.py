import math

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticFrames(Node):
    """
    Publishes static TF frames for robot sensors and compatibility.
    
    Frames published:
    - base_link -> base_footprint: Robot ground plane reference
    - base_link -> lidar_link: Lidar sensor position
    - base_link -> jackal/lidar_link/lidar: Compatibility frame
    """
    
    def __init__(self):
        super().__init__('static_frames')

        self.declare_parameter('lidar_x', 0.20)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.75)
        self.declare_parameter('lidar_yaw', 0.0)

        x = float(self.get_parameter('lidar_x').value)
        y = float(self.get_parameter('lidar_y').value)
        z = float(self.get_parameter('lidar_z').value)
        yaw = float(self.get_parameter('lidar_yaw').value)

        self.broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        # Robot base frames
        transforms.append(self._make_tf('base_link', 'base_footprint', 0.0, 0.0, 0.0, 0.0))

        # Lidar sensor frame
        transforms.append(self._make_tf('base_link', 'lidar_link', x, y, z, yaw))

        # Compatibility frame for legacy components
        transforms.append(self._make_tf('base_link', 'jackal/lidar_link/lidar', x, y, z, yaw))

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info("Static frames published: base_footprint, lidar_link")

    def _make_tf(self, parent, child, x, y, z, yaw):
        """Create a TransformStamped message with yaw-only rotation"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        return t


def main():
    rclpy.init()
    node = StaticFrames()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()