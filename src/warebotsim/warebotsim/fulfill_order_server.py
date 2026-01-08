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
        print(f"[FulfillOrderServer] {msg}", flush=True)

    def _tf_exists(self, target_frame: str, source_frame: str = 'map') -> bool:
        try:
            self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def _pose_from_tf(self, target_frame: str) -> PoseStamped:
        tf = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _get_robot_pose(self):
        """Get robot's current pose in map frame."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y, tf.transform.rotation
        except Exception as e:
            self._log(f"Failed to get robot pose: {e}")
            return None, None, None

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

            self._log(f"Spawning {package_id} at ({x:.2f}, {y:.2f}, {z:.2f})")
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', f'sdf: "{sdf_content}"'
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self._log("Spawn success!")
                return True
            else:
                self.get_logger().error(f"Spawn failed: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Spawn exception: {e}")
            return False

    def _attach_package_to_robot(self, package_id: str) -> bool:
        """Teleport package from shelf to robot safe zone - CENTERED and ALIGNED."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Robot Position
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            rz = tf.transform.translation.z

            # Get Yaw from Robot Quaternion
            q_robot = tf.transform.rotation
            (roll, pitch, yaw) = euler_from_quaternion([q_robot.x, q_robot.y, q_robot.z, q_robot.w])
            
            self._log(f"Robot at ({rx:.2f}, {ry:.2f}), yaw={math.degrees(yaw):.1f}°")

            # CENTERED POSITION CALCULATION:
            # Lidar is at x=+0.20 from base_link
            # Package is 0.15x0.15x0.15
            # Place package at x=0.0 (centered on robot, behind lidar)
            # This gives 0.20m clearance to lidar (lidar at +0.20, package front at +0.075)
            offset_x = 0.0 * math.cos(yaw) - 0.0 * math.sin(yaw)
            offset_y = 0.0 * math.sin(yaw) + 0.0 * math.cos(yaw)

            final_x = rx + offset_x
            final_y = ry + offset_y
            final_z = rz + 0.30  # Slightly above robot

            self._log(f"Teleporting {package_id} to ({final_x:.2f}, {final_y:.2f}, {final_z:.2f}), yaw={math.degrees(yaw):.1f}°")
            
            # Use INLINE pose format with yaw directly
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {final_x}, y: {final_y}, z: {final_z}}}, orientation: {{x: 0, y: 0, z: {math.sin(yaw/2.0)}, w: {math.cos(yaw/2.0)}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self._log("Attach success!")
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
            
            self._log(f"Placing {package_id} inside delivery cylinder ({final_x:.2f}, {final_y:.2f}, {final_z:.2f})...")
            
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {final_x}, y: {final_y}, z: {final_z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self._log("Delivery Placement Success!")
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
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        
        end_time = time.time() + duration
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.vel_pub.publish(msg)

    def _backup_and_turn(self, target_yaw):
        """Back up from shelf and rotate to face target direction."""
        self._log("Backing up from shelf...")
        self._manual_drive(-0.3, 0.0, 3.0)  # Back up 0.9m
        time.sleep(0.5)
        
        # Get current yaw
        rx, ry, q = self._get_robot_pose()
        if q is None:
            self._log("Can't get robot pose, skipping rotation")
            return
        
        current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        
        # Calculate angle difference
        angle_diff = target_yaw - current_yaw
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        self._log(f"Rotating {math.degrees(angle_diff):.1f}° to face target...")
        
        # Rotate to face target
        rotation_time = abs(angle_diff) / 0.5  # 0.5 rad/s rotation speed
        rotation_dir = 1.0 if angle_diff > 0 else -1.0
        self._manual_drive(0.0, 0.5 * rotation_dir, rotation_time)
        time.sleep(0.5)

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

        self._log(f"Sending Nav2 goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})...")
        
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

        if status != 4: # 4 = SUCCEEDED
            return False, f"Nav2 failed (status={status})"

        return True, "ok"

    def execute_callback(self, goal_handle):
        self._log("Received Action Goal")
        req = goal_handle.request
        shelf_frame = f"shelf_{int(req.shelf_id)}"
        delivery_frame = f"delivery_{int(req.delivery_id)}"
        pkg_id = req.package_id if req.package_id else f"pkg_{req.order_id}"
        
        feedback = FulfillOrder.Feedback()

        if not self._tf_exists(shelf_frame) or not self._tf_exists(delivery_frame):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message="TF Missing")

        # Get shelf and delivery real positions from warehouse_world.sdf
        # Shelves: (2.2, 1.5), (1.8, -0.2), (2.5, -2.0)
        # Deliveries: (-2.2, 2.0), (-1.8, 0.0), (-2.5, -2.2)
        shelf_positions = {
            1: (2.2, 1.5),
            2: (1.8, -0.2),
            3: (2.5, -2.0),
        }
        delivery_positions = {
            1: (-2.2, 2.0),
            2: (-1.8, 0.0),
            3: (-2.5, -2.2),
        }
        
        shelf_id = int(req.shelf_id)
        delivery_id = int(req.delivery_id)
        
        shelf_center_x, shelf_center_y = shelf_positions[shelf_id]
        delivery_center_x, delivery_center_y = delivery_positions[delivery_id]

        # ---------------------------------------------------------
        # 1. SPAWN ON SHELF
        # ---------------------------------------------------------
        feedback.stage = "spawning_package"
        feedback.progress = 0.05
        goal_handle.publish_feedback(feedback)

        try:
            # Spawn inside shelf
            spawn_x = shelf_center_x + 0.35  # Inside shelf
            spawn_y = shelf_center_y
            spawn_z = 1.2
            
            self._spawn_package(pkg_id, spawn_x, spawn_y, spawn_z)
            time.sleep(1.0)
        except Exception as e:
            self._log(f"Spawn Error: {e}")

        # ---------------------------------------------------------
        # 2. GO TO SHELF
        # ---------------------------------------------------------
        try:
            # Approach at 1.2m before shelf center (was 1.0m, increasing for safety)
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = 'map'
            approach_pose.header.stamp = self.get_clock().now().to_msg()
            approach_pose.pose.position.x = shelf_center_x - 1.2
            approach_pose.pose.position.y = shelf_center_y
            approach_pose.pose.position.z = 0.0
            
            # Orientation: Face EAST (towards shelf, 0 degrees)
            q = quaternion_from_euler(0, 0, 0.0)
            approach_pose.pose.orientation.x = q[0]
            approach_pose.pose.orientation.y = q[1]
            approach_pose.pose.orientation.z = q[2]
            approach_pose.pose.orientation.w = q[3]

            ok, msg = self._nav2_go_to_pose(goal_handle, approach_pose, "goto_shelf", 0.1, 0.45)
            if not ok:
                goal_handle.abort()
                return FulfillOrder.Result(success=False, message=msg)
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
        
        self._log(f"Picking {pkg_id}...")
        time.sleep(1.0)
        self._attach_package_to_robot(pkg_id)
        time.sleep(1.0)

        # ---------------------------------------------------------
        # 4. BACKUP AND ROTATE TO FACE DELIVERY
        # ---------------------------------------------------------
        # Calculate bearing to delivery
        target_yaw = math.atan2(delivery_center_y - shelf_center_y, 
                                delivery_center_x - shelf_center_x)
        self._backup_and_turn(target_yaw)

        # ---------------------------------------------------------
        # 5. GO TO DELIVERY
        # ---------------------------------------------------------
        try:
            # Approach at 1.0m after delivery center
            delivery_approach = PoseStamped()
            delivery_approach.header.frame_id = 'map'
            delivery_approach.header.stamp = self.get_clock().now().to_msg()
            delivery_approach.pose.position.x = delivery_center_x + 1.0
            delivery_approach.pose.position.y = delivery_center_y
            delivery_approach.pose.position.z = 0.0
            
            # Orientation: Face WEST (towards delivery, 180 degrees = pi)
            q = quaternion_from_euler(0, 0, math.pi)
            delivery_approach.pose.orientation.x = q[0]
            delivery_approach.pose.orientation.y = q[1]
            delivery_approach.pose.orientation.z = q[2]
            delivery_approach.pose.orientation.w = q[3]

            ok, msg = self._nav2_go_to_pose(goal_handle, delivery_approach, "goto_delivery", 0.55, 0.90)
            if not ok:
                goal_handle.abort()
                return FulfillOrder.Result(success=False, message=msg)
        except Exception as e:
             goal_handle.abort()
             return FulfillOrder.Result(success=False, message=f"Delivery Nav Error: {e}")

        # ---------------------------------------------------------
        # 6. PLACE (DELIVER)
        # ---------------------------------------------------------
        feedback.stage = "delivering"
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)
        
        # Place package inside delivery cylinder (at center, on floor)
        self._place_package_at_delivery(pkg_id, delivery_center_x, delivery_center_y)
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