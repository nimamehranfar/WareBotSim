from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.timer = self.create_timer(1/30, self.update)

        self.degree = pi / 180.0

        # robot state
        self.tilt = 0.
        self.tinc = self.degree
        self.swivel = 0.
        self.angle = 0.
        self.height = 0.
        self.hinc = 0.005

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'axis'
        self.joint_state = JointState()

        self.get_logger().info("{0} started".format(self.get_name()))

    def update(self):
        # update joint_state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['swivel', 'tilt', 'periscope']
        self.joint_state.position = [self.swivel, self.tilt, self.height]

        # update transform
        # (moving in a circle with radius=2)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = cos(self.angle)*2
        self.odom_trans.transform.translation.y = sin(self.angle)*2
        self.odom_trans.transform.translation.z = 0.7
        self.odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, self.angle + pi/2) # roll,pitch,yaw

        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)

        # Create new robot state
        self.tilt += self.tinc
        if self.tilt < -0.5 or self.tilt > 0.0:
            self.tinc *= -1
        self.height += self.hinc
        if self.height > 0.2 or self.height < 0.0:
            self.hinc *= -1
        self.swivel += self.degree
        self.angle += self.degree/4


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    try:
        with rclpy.init():
            node = StatePublisher()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()