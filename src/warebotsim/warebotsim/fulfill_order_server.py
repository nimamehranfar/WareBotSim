import math
import subprocess
import time
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from warebotsim_interfaces.action import FulfillOrder


class FulfillOrderServer(Node):
    def __init__(self):
        super().__init__('fulfill_order_server')

        self.declare_parameter('use_nav2', True)
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)

        # Reentrant group allows nested callbacks
        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 Client
        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group
        )

        # Velocity Publisher for manual maneuvers
        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self._as = ActionServer(
            self,
            FulfillOrder,
            '/robot/fulfill_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.current_package = None
        self.get_logger().info(f"Robot ready. use_nav2={self.use_nav2}")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _log(self, msg):
        self.get_logger().info(msg)

    def _tf_exists(self, target_frame: str, source_frame: str = 'map') -> bool:
        try:
            self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def _get_xy_yaw(self, target_frame: str):
        tf = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        q = tf.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return x, y, z, yaw

    def _make_goal_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _normalize_angle(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # --- ACTION HELPERS ---

    def _spawn_package(self, package_id: str, x: float, y: float, z: float) -> bool:
        """Spawn package at specific coordinates (Shelf)."""
        try:
            # Compact SDF String
            sdf_content = (
                "<?xml version='1.0'?>"
                "<sdf version='1.6'>"
                f"<model name='{package_id}'>"
                f"<pose>{x} {y} {z} 0 0 0</pose>"
                "<link name='link'>"
                "<inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>"
                "<collision name='collision'><geometry><box><size>0.15 0.15 0.15</size></box></geometry></collision>"
                "<visual name='visual'><geometry><box><size>0.15 0.15 0.15</size></box></geometry><material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material></visual>"
                "</link></model></sdf>"
            )

            self._log(f"Spawning {package_id}")
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', f'sdf: "{sdf_content}"'
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return True
            else:
                self.get_logger().error(f"Spawn failed: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Spawn exception: {e}")
            return False

    def _attach_package_to_robot(self, package_id: str) -> bool:
        """Teleport package from shelf to robot safe zone - EXACTLY ON TOP and ALIGNED."""
        try:
            rx, ry, rz, yaw = self._get_xy_yaw('base_link')

            # base_collision top is at z=0.184 in base_link, package half height is 0.075
            # Center z for sitting on top: 0.184 + 0.075 + small clearance
            dz = 0.184 + 0.075 + 0.01

            final_x = rx
            final_y = ry
            final_z = rz + dz

            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                f'name: "{package_id}", position: {{x: {final_x}, y: {final_y}, z: {final_z}}}, '
                f'orientation: {{x: 0, y: 0, z: {math.sin(yaw/2.0)}, w: {math.cos(yaw/2.0)}}}'
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return True
            else:
                self.get_logger().error(f"Attach failed: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Attach exception: {e}")
            return False

    def _place_package_at_delivery(self, package_id: str, delivery_center_x: float, delivery_center_y: float) -> bool:
        """Place package inside delivery cylinder (on the floor inside it)."""
        try:
            # Delivery cylinders in SDF:
            # - radius: 0.18m
            # - height (length): 1.4m
            # - pose z: 0.7m (center of cylinder)
            # This means cylinder spans from z=0.0 to z=1.4

            # Place package INSIDE the cylinder, resting on the floor (z=0.0)
            # Package is 0.15x0.15x0.15, so half-height is 0.075
            # To have package sitting on floor with bottom at z=0, center should be at z=0.075
            # Add small offset to ensure it's slightly above floor: z=0.10

            final_x = delivery_center_x
            final_y = delivery_center_y
            final_z = 0.10  # Package center at 10cm, so bottom at ~2.5cm (inside cylinder)

            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {final_x}, y: {final_y}, z: {final_z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return True
            else:
                self.get_logger().error(f"Delivery Placement Failed: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Delivery Exception: {e}")
            return False

    def _manual_drive(self, linear_x, angular_z, duration):
        """Manually drive robot with given velocities for duration."""
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)

        end_time = time.time() + duration
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(msg)
            time.sleep(0.1)

        # Stop
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.vel_pub.publish(msg)

    def _rotate_to_yaw(self, target_yaw, timeout_sec=6.0, tolerance=0.06):
        start_time = time.time()
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'

        while time.time() - start_time < timeout_sec:
            try:
                _, _, _, current_yaw = self._get_xy_yaw('base_link')
            except Exception:
                return False

            angle_diff = self._normalize_angle(float(target_yaw) - float(current_yaw))
            if abs(angle_diff) <= tolerance:
                break

            angular = max(min(angle_diff * 1.8, 0.7), -0.7)
            if abs(angular) < 0.22:
                angular = 0.22 if angular >= 0 else -0.22

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = float(angular)
            self.vel_pub.publish(msg)
            time.sleep(0.1)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.vel_pub.publish(msg)

        try:
            _, _, _, current_yaw = self._get_xy_yaw('base_link')
            angle_diff = self._normalize_angle(float(target_yaw) - float(current_yaw))
            return abs(angle_diff) <= tolerance
        except Exception:
            return False

    def _nav2_go_to_pose(self, goal_handle, pose: PoseStamped, stage_prefix: str,
                          p_min: float, p_max: float) -> tuple[bool, str]:

        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            return False, "Nav2 action server not available"

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        fb = FulfillOrder.Feedback()
        initial_dist = {'value': None}

        def nav_feedback_cb(msg):
            try:
                dist = float(msg.feedback.distance_remaining)
                if initial_dist['value'] is None:
                    initial_dist['value'] = max(dist, 0.001)
                ratio = 1.0 - min(dist / initial_dist['value'], 1.0)
                fb.stage = f"{stage_prefix} dist={dist:.2f}m"
                fb.progress = p_min + (p_max - p_min) * ratio
                goal_handle.publish_feedback(fb)
            except:
                pass

        # Send Goal (Synchronous)
        send_future = self.nav2_client.send_goal_async(nav_goal, feedback_callback=nav_feedback_cb)
        while not send_future.done():
            time.sleep(0.1)

        nav_goal_handle = send_future.result()

        if not nav_goal_handle.accepted:
            return False, f"Nav2 rejected goal"

        # Wait for Result (Synchronous)
        result_future = nav_goal_handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return False, "Canceled"
            time.sleep(0.5)

        res = result_future.result()
        status = res.status

        if status != 4:  # 4 = SUCCEEDED
            return False, f"Nav2 failed (status={status})"

        return True, "ok"

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        shelf_frame = f"shelf_{int(req.shelf_id)}"
        delivery_frame = f"delivery_{int(req.delivery_id)}"
        pkg_id = req.package_id if req.package_id else f"pkg_{req.order_id}"

        feedback = FulfillOrder.Feedback()

        if not self._tf_exists(shelf_frame) or not self._tf_exists(delivery_frame):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message="TF Missing")

        # ---------------------------------------------------------
        # 1. SPAWN ON SHELF
        # ---------------------------------------------------------
        feedback.stage = "spawning_package"
        feedback.progress = 0.05
        goal_handle.publish_feedback(feedback)

        try:
            shelf_x, shelf_y, _, _ = self._get_xy_yaw(shelf_frame)

            # Spawn inside shelf
            spawn_x = shelf_x + 0.35  # Inside shelf
            spawn_y = shelf_y
            spawn_z = 1.2

            self._spawn_package(pkg_id, spawn_x, spawn_y, spawn_z)
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Spawn Error: {e}")

        # ---------------------------------------------------------
        # 2. GO TO SHELF
        # ---------------------------------------------------------
        try:
            shelf_x, shelf_y, _, _ = self._get_xy_yaw(shelf_frame)
            rx, ry, _, _ = self._get_xy_yaw('base_link')

            # Approach point: stop approach_dist before shelf along the robot->shelf line
            approach_dist = 1.2
            vx = shelf_x - rx
            vy = shelf_y - ry
            d = math.hypot(vx, vy)
            if d < 1e-6:
                ux, uy = 1.0, 0.0
            else:
                ux, uy = vx / d, vy / d

            approach_x = shelf_x - ux * approach_dist
            approach_y = shelf_y - uy * approach_dist
            yaw_to_shelf = math.atan2(shelf_y - approach_y, shelf_x - approach_x)

            approach_pose = self._make_goal_pose(approach_x, approach_y, yaw_to_shelf)

            ok, msg = self._nav2_go_to_pose(goal_handle, approach_pose, "goto_shelf", 0.1, 0.45)
            if not ok:
                goal_handle.abort()
                return FulfillOrder.Result(success=False, message=msg)

            # Force final heading (one short closed-loop rotation)
            self._rotate_to_yaw(yaw_to_shelf, timeout_sec=6.0, tolerance=0.06)
        except Exception as e:
            self.get_logger().error(f"Navigation Setup Error: {e}")
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=str(e))

        # ---------------------------------------------------------
        # 3. PICK (ATTACH)
        # ---------------------------------------------------------
        feedback.stage = "picking"
        feedback.progress = 0.50
        goal_handle.publish_feedback(feedback)

        time.sleep(0.2)
        self._manual_drive(0.0, 0.0, 0.2)
        self._attach_package_to_robot(pkg_id)
        time.sleep(0.3)

        # ---------------------------------------------------------
        # 4. BACKUP
        # ---------------------------------------------------------
        self._manual_drive(-0.3, 0.0, 2.5)
        time.sleep(0.2)

        # ---------------------------------------------------------
        # 5. GO TO DELIVERY
        # ---------------------------------------------------------
        try:
            delivery_x, delivery_y, _, _ = self._get_xy_yaw(delivery_frame)
            rx, ry, _, _ = self._get_xy_yaw('base_link')

            approach_dist = 1.0
            vx = delivery_x - rx
            vy = delivery_y - ry
            d = math.hypot(vx, vy)
            if d < 1e-6:
                ux, uy = 1.0, 0.0
            else:
                ux, uy = vx / d, vy / d

            approach_x = delivery_x - ux * approach_dist
            approach_y = delivery_y - uy * approach_dist
            yaw_to_delivery = math.atan2(delivery_y - approach_y, delivery_x - approach_x)

            delivery_approach = self._make_goal_pose(approach_x, approach_y, yaw_to_delivery)

            ok, msg = self._nav2_go_to_pose(goal_handle, delivery_approach, "goto_delivery", 0.55, 0.90)
            if not ok:
                goal_handle.abort()
                return FulfillOrder.Result(success=False, message=msg)

            self._rotate_to_yaw(yaw_to_delivery, timeout_sec=6.0, tolerance=0.06)
        except Exception as e:
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=f"Delivery Nav Error: {e}")

        # ---------------------------------------------------------
        # 6. PLACE (DELIVER)
        # ---------------------------------------------------------
        feedback.stage = "delivering"
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)

        try:
            delivery_x, delivery_y, _, _ = self._get_xy_yaw(delivery_frame)
        except Exception:
            delivery_x, delivery_y = 0.0, 0.0

        self._place_package_at_delivery(pkg_id, delivery_x, delivery_y)
        goal_handle.succeed()
        return FulfillOrder.Result(success=True, message="Success")


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
