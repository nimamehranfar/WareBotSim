import math

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticFrames(Node):
    def __init__(self):
        super().__init__('static_frames')

        # These match your model.sdf lidar pose:
        # <link name="lidar_link">
        #   <pose relative_to="base_link">0.20 0 0.25 0 0 0</pose>
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

        # Optional but common: base_footprint aligned with base_link (planar robot)
        transforms.append(self._make_tf('base_link', 'base_footprint', 0.0, 0.0, 0.0, 0.0))

        # Main: base_link -> lidar_link (this is the important one for SLAM)
        transforms.append(self._make_tf('base_link', 'lidar_link', x, y, z, yaw))

        # Compatibility: some bridges/sensors sometimes report a nested scan frame id.
        # Your order_manager previously published this exact frame.
        transforms.append(self._make_tf('base_link', 'jackal/lidar_link/lidar', x, y, z, yaw))

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info("Published static TFs: base_link->lidar_link (+ compatibility frames)")

    def _make_tf(self, parent, child, x, y, z, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # yaw-only quaternion
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
