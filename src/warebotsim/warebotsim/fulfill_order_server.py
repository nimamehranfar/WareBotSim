import math
import subprocess
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from warebotsim_interfaces.action import FulfillOrder


class FulfillOrderServer(Node):
    """High-level task server for pick-and-deliver orders.

    Shelf and delivery target poses are read from TF frames (shelf_*, delivery_*).
    Navigation uses Nav2 (NavigateToPose). Short retreats and in-place rotations use /cmd_vel.
    """

    def __init__(self):
        super().__init__('fulfill_order_server')

        # Approach offsets (meters):
        # Shelves live at +X (east). We approach from the left/west => negative dx.
        # Deliveries live at -X (west). We approach from the right/east => positive dx.
        self.declare_parameter('shelf_approach_dx', -1.25)
        self.declare_parameter('shelf_2_approach_dx', -0.85)
        self.declare_parameter('delivery_approach_dx', 0.55)

        # Retreat distances (meters) for cmd_vel motion
        self.declare_parameter('post_pick_retreat_distance', 0.90)
        self.declare_parameter('post_drop_retreat_distance', 0.90)
        self.declare_parameter('retreat_linear_speed', 0.25)
        
        # Rotation parameters
        self.declare_parameter('rotation_angular_speed', 0.50)
        self.declare_parameter('rotation_tolerance', 0.10)

        # Package mount offset in base_link frame
        self.declare_parameter('package_mount_x', 0.0)
        self.declare_parameter('package_mount_y', 0.0)
        self.declare_parameter('package_mount_z', 0.30)

        # Spawn / place offsets
        self.declare_parameter('spawn_on_shelf_dx', -0.30)
        self.declare_parameter('spawn_on_shelf_z', 1.20)
        self.declare_parameter('place_on_delivery_z', 1.20)

        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self._as = ActionServer(
            self,
            FulfillOrder,
            '/robot/fulfill_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Robot ready. (Navigation via Nav2 NavigateToPose)')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # ------------------------------
    # TF helpers
    # ------------------------------

    def _tf_exists(self, target_frame: str, source_frame: str = 'map') -> bool:
        try:
            self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def _lookup_map_xy(self, frame_id: str) -> tuple[float, float]:
        tf = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
        return float(tf.transform.translation.x), float(tf.transform.translation.y)

    def _lookup_base_pose(self) -> tuple[float, float, float, float]:
        """Return robot (x,y,z,yaw) in map."""
        tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        z = float(tf.transform.translation.z)
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, z, float(yaw)

    def _pose_xy_yaw(self, x: float, y: float, yaw: float) -> PoseStamped:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    # ------------------------------
    # Gazebo helpers (spawn/teleport)
    # ------------------------------

    def _spawn_package(self, package_id: str, x: float, y: float, z: float) -> bool:
        """Spawn package at specific coordinates."""
        try:
            # Keep collision footprint exactly the same (0.15 cube), but improve visuals.
            # Visuals: base cardboard + tape strips.
            sdf_content = (
                "<?xml version='1.0'?>"
                "<sdf version='1.6'>"
                f"<model name='{package_id}'>"
                f"<pose>{x} {y} {z} 0 0 0</pose>"
                "<link name='link'>"
                "<inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>"
                "<iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>"
                "<collision name='collision'><geometry><box><size>0.15 0.15 0.15</size></box></geometry></collision>"
                "<visual name='box'>"
                "  <geometry><box><size>0.15 0.15 0.15</size></box></geometry>"
                "  <material><ambient>0.72 0.56 0.34 1</ambient><diffuse>0.72 0.56 0.34 1</diffuse></material>"
                "</visual>"
                "<visual name='tape_x'>"
                "  <pose>0 0 0.076 0 0 0</pose>"
                "  <geometry><box><size>0.16 0.03 0.002</size></box></geometry>"
                "  <material><ambient>0.85 0.80 0.65 1</ambient><diffuse>0.85 0.80 0.65 1</diffuse></material>"
                "</visual>"
                "<visual name='tape_y'>"
                "  <pose>0 0 0.076 0 0 0</pose>"
                "  <geometry><box><size>0.03 0.16 0.002</size></box></geometry>"
                "  <material><ambient>0.85 0.80 0.65 1</ambient><diffuse>0.85 0.80 0.65 1</diffuse></material>"
                "</visual>"
                "</link></model></sdf>"
            )

            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', f'sdf: "{sdf_content}"',
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().error(f"Spawn failed: {result.stderr}")
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"Spawn exception: {e}")
            return False

    def _set_model_pose(self, model_name: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> bool:
        try:
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', (
                    f'name: "{model_name}", '
                    f'position: {{x: {x}, y: {y}, z: {z}}} '
                    f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
                ),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().error(f"set_pose failed: {result.stderr}")
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"set_pose exception: {e}")
            return False

    def _attach_package_to_robot(self, package_id: str) -> bool:
        """Teleport package to the robot's pickup tray.

        Key fix:
        - Apply the mount offset in the robot (base_link) frame, then rotate it into map.
        - Keep the package upright (yaw-only), so it doesn't inherit roll/pitch and slide.
        """
        try:
            rx, ry, rz, yaw = self._lookup_base_pose()

            x_off = float(self.get_parameter('package_mount_x').value)
            y_off = float(self.get_parameter('package_mount_y').value)
            z_off = float(self.get_parameter('package_mount_z').value)

            c = math.cos(yaw)
            s = math.sin(yaw)

            final_x = rx + (c * x_off - s * y_off)
            final_y = ry + (s * x_off + c * y_off)
            final_z = rz + z_off

            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            return self._set_model_pose(package_id, final_x, final_y, final_z, qx, qy, qz, qw)
        except Exception as e:
            self.get_logger().error(f"Attach exception: {e}")
            return False

    def _place_package_at_delivery(self, package_id: str, x: float, y: float, z: float) -> bool:
        """Teleport package to delivery center ."""
        return self._set_model_pose(package_id, float(x), float(y), float(z), 0.0, 0.0, 0.0, 1.0)

    # ------------------------------
    # Nav2 helper
    # ------------------------------


    def _publish_feedback(self, goal_handle, stage: str, progress: float) -> None:
        fb = FulfillOrder.Feedback()
        fb.stage = stage
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    def _nav2_go_to_pose(
        self,
        goal_handle,
        pose: PoseStamped,
        stage_prefix: str,
        progress: float,
        publish_period_s: float = 0.5,
    ) -> tuple[bool, str]:
        last_pub_ns = 0

        def nav_feedback_cb(msg):
            nonlocal last_pub_ns
            try:
                dist = float(msg.feedback.distance_remaining)
            except Exception:
                return

            now_ns = self.get_clock().now().nanoseconds
            if now_ns - last_pub_ns < int(publish_period_s * 1e9):
                return
            last_pub_ns = now_ns

            self._publish_feedback(goal_handle, f"{stage_prefix} dist={dist:.2f}m", progress)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        send_future = self.nav2_client.send_goal_async(nav_goal, feedback_callback=nav_feedback_cb)
        while not send_future.done():
            time.sleep(0.05)

        nav_goal_handle = send_future.result()
        if not nav_goal_handle.accepted:
            return False, 'Nav2 rejected goal'

        result_future = nav_goal_handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return False, 'Canceled'
            time.sleep(0.2)

        res = result_future.result()
        status = res.status
        if status != 4:  # SUCCEEDED
            return False, f'Nav2 failed (status={status})'
        return True, 'ok'

    def _move_linear(
        self,
        goal_handle,
        distance: float,
        speed: float,
        stage_prefix: str,
        progress: float,
        publish_period_s: float = 0.5,
    ) -> bool:
        """Time-based straight motion via cmd_vel."""
        if abs(distance) < 0.01:
            return True

        direction = 1.0 if distance > 0 else -1.0
        speed = abs(speed) * direction
        duration = abs(distance / speed)

        twist = TwistStamped()
        twist.twist.linear.x = speed

        rate = self.create_rate(20)
        start_time = self.get_clock().now()
        last_pub_ns = 0

        while True:
            if goal_handle.is_cancel_requested:
                twist.twist.linear.x = 0.0
                twist.header.stamp = self.get_clock().now().to_msg()
                self.cmd_vel_pub.publish(twist)
                goal_handle.canceled()
                return False

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            remaining = max(0.0, abs(distance) - abs(speed) * elapsed)

            now_ns = self.get_clock().now().nanoseconds
            if now_ns - last_pub_ns >= int(publish_period_s * 1e9):
                last_pub_ns = now_ns
                self._publish_feedback(goal_handle, f"{stage_prefix} dist={remaining:.2f}m", progress)

            if elapsed >= duration:
                break

            twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.twist.linear.x = 0.0
        twist.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.3)
        return True

    def _rotate_to_yaw(
        self,
        goal_handle,
        target_yaw: float,
        stage_prefix: str,
        progress: float,
        publish_period_s: float = 0.5,
    ) -> bool:
        """Rotate in place (cmd_vel) until yaw is within tolerance."""
        angular_speed = float(self.get_parameter('rotation_angular_speed').value)
        tolerance = float(self.get_parameter('rotation_tolerance').value)

        rate = self.create_rate(20)
        twist = TwistStamped()
        last_pub_ns = 0

        while True:
            if goal_handle.is_cancel_requested:
                twist.twist.angular.z = 0.0
                twist.header.stamp = self.get_clock().now().to_msg()
                self.cmd_vel_pub.publish(twist)
                goal_handle.canceled()
                return False

            try:
                _, _, _, current_yaw = self._lookup_base_pose()
            except Exception:
                continue

            angle_diff = target_yaw - current_yaw
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            if abs(angle_diff) < tolerance:
                break

            now_ns = self.get_clock().now().nanoseconds
            if now_ns - last_pub_ns >= int(publish_period_s * 1e9):
                last_pub_ns = now_ns
                self._publish_feedback(goal_handle, f"{stage_prefix} err={abs(angle_diff):.2f}rad", progress)

            twist.twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
            twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.twist.angular.z = 0.0
        twist.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.3)
        return True

    # ------------------------------
    # Action execution
    # ------------------------------


    def execute_callback(self, goal_handle):
        req = goal_handle.request

        shelf_id = int(req.shelf_id)
        delivery_id = int(req.delivery_id)

        shelf_frame = f'shelf_{shelf_id}'
        delivery_frame = f'delivery_{delivery_id}'
        pkg_id = req.package_id if req.package_id else f'pkg_{req.order_id}'

        if not self._tf_exists(shelf_frame) or not self._tf_exists(delivery_frame):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='TF Missing')

        progress = 0.0

        # 1) Spawn on shelf
        self._publish_feedback(goal_handle, 'spawning_package', progress)

        shelf_x, shelf_y = self._lookup_map_xy(shelf_frame)
        spawn_x = shelf_x + float(self.get_parameter('spawn_on_shelf_dx').value)
        spawn_y = shelf_y
        spawn_z = float(self.get_parameter('spawn_on_shelf_z').value)

        self._spawn_package(pkg_id, spawn_x, spawn_y, spawn_z)
        time.sleep(0.5)

        progress = 0.10
        self._publish_feedback(goal_handle, 'spawn_complete', progress)

        # 2) Navigate to shelf approach pose
        shelf_approach_dx = float(self.get_parameter('shelf_approach_dx').value)
        if shelf_id == 2:
            shelf_approach_dx = float(self.get_parameter('shelf_2_approach_dx').value)

        shelf_goal = self._pose_xy_yaw(shelf_x + shelf_approach_dx, shelf_y, 0.0)
        ok, msg = self._nav2_go_to_pose(goal_handle, shelf_goal, 'goto_shelf', progress)
        if not ok:
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=msg)
        time.sleep(0.5)

        progress = 0.40
        self._publish_feedback(goal_handle, 'arrived_shelf', progress)

        # 3) Pick (attach package to robot)
        self._publish_feedback(goal_handle, 'picking', progress)
        time.sleep(0.3)

        if not self._attach_package_to_robot(pkg_id):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='Attach failed')
        time.sleep(0.5)

        progress = 0.50
        self._publish_feedback(goal_handle, 'pick_complete', progress)

        # 4) Retreat and rotate toward delivery direction (cmd_vel)
        retreat_dist = float(self.get_parameter('post_pick_retreat_distance').value)
        retreat_speed = float(self.get_parameter('retreat_linear_speed').value)

        if not self._move_linear(goal_handle, -retreat_dist, retreat_speed, 'retreat', progress):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='Retreat failed')

        if not self._rotate_to_yaw(goal_handle, math.pi, 'rotate', progress):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='Rotation failed')

        progress = 0.60
        self._publish_feedback(goal_handle, 'ready_for_delivery', progress)

        # 5) Navigate to delivery approach pose
        delivery_x, delivery_y = self._lookup_map_xy(delivery_frame)
        delivery_approach_dx = float(self.get_parameter('delivery_approach_dx').value)
        delivery_goal = self._pose_xy_yaw(delivery_x + delivery_approach_dx, delivery_y, math.pi)

        ok, msg = self._nav2_go_to_pose(goal_handle, delivery_goal, 'goto_delivery', progress)
        if not ok:
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=msg)

        progress = 0.90
        self._publish_feedback(goal_handle, 'arrived_delivery', progress)

        # 6) Place at delivery center
        self._publish_feedback(goal_handle, 'delivering', progress)
        time.sleep(0.3)

        place_z = float(self.get_parameter('place_on_delivery_z').value)
        if not self._place_package_at_delivery(pkg_id, delivery_x, delivery_y, place_z):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='Place failed')

        progress = 0.95
        self._publish_feedback(goal_handle, 'drop_complete', progress)

        # 7) Post-drop retreat and rotate to face shelves (cmd_vel)
        retreat_dist = float(self.get_parameter('post_drop_retreat_distance').value)
        if not self._move_linear(goal_handle, -retreat_dist, retreat_speed, 'retreat', progress):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='Retreat failed')

        if not self._rotate_to_yaw(goal_handle, 0.0, 'rotate', progress):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message='Final rotation failed')

        progress = 1.00
        self._publish_feedback(goal_handle, 'order_complete', progress)

        goal_handle.succeed()
        return FulfillOrder.Result(success=True, message='Success')

def main():
    rclpy.init()
    node = FulfillOrderServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
