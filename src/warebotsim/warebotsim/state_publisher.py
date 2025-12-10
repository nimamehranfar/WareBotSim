import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, TransformStamped
from sensor_msgs.msg import JointState
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        # Transform publisher
        self.br = TransformBroadcaster(self)

        # JointState publisher
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscribe to /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Velocities (updated through cmd_vel)
        self.v = 0.0      # linear velocity
        self.w = 0.0      # angular velocity

        # Timer for updating pose
        self.timer = self.create_timer(0.05, self.update)  # 20Hz

        # Joints
        self.wheel_joints = [
            'front_left_wheel',
            'front_right_wheel',
            'rear_left_wheel',
            'rear_right_wheel'
        ]
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        dt = 0.05

        # Update pose according to velocities
        self.x += self.v * dt * math.cos(self.yaw)
        self.y += self.v * dt * math.sin(self.yaw)
        self.yaw += self.w * dt

        # --- TF: odom -> base_link ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.br.sendTransform(t)

        # --- Joint states ---
        js = JointState()
        js.header.stamp = t.header.stamp
        js.name = self.wheel_joints
        js.position = self.wheel_pos

        self.js_pub.publish(js)

def main():
    rclpy.init()
    node = StatePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
