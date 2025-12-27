import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        # Incoming odom topic from Gazebo
        self.odom_topic = '/odom'

        # Canonical frames REQUIRED by SLAM / Nav2
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            50
        )

    def _odom_callback(self, msg: Odometry):
        """
        Normalize Gazebo odometry frames:
        - Whatever Gazebo uses → publish as odom → base_link
        """

        t = TransformStamped()
        t.header.stamp = msg.header.stamp

        # FORCE canonical frame names
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
