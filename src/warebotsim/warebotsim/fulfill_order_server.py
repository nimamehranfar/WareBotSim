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
        """Teleport package from shelf to robot safe zone."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Robot Position
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            rz = tf.transform.translation.z

            # Get Yaw from Robot Quaternion
            q_robot = tf.transform.rotation
            (roll, pitch, yaw) = euler_from_quaternion([q_robot.x, q_robot.y, q_robot.z, q_robot.w])
            
            # Create a clean quaternion (Pitch=0, Roll=0) to ensure box is FLAT
            q_clean = quaternion_from_euler(0, 0, yaw)

            # POSITION: Center of robot (rx, ry)
            # No offset needed if we just want it on top.
            final_x = rx 
            final_y = ry
            final_z = rz + 0.30 

            self._log(f"Teleporting {package_id} to robot ({final_x:.2f}, {final_y:.2f})...")
            
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {final_x}, y: {final_y}, z: {final_z}}} orientation: {{x: {q_clean[0]}, y: {q_clean[1]}, z: {q_clean[2]}, w: {q_clean[3]}}}'
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

    def _place_package_at_delivery(self, package_id: str, x: float, y: float, z: float) -> bool:
        """Teleport package from robot to on top of the delivery point."""
        try:
            # DELIVERY HEIGHT FIX:
            # Delivery model in SDF is a cylinder length 1.4m at pose z=0.7.
            # This means it occupies z=0.0 to z=1.4.
            # We must place package ON TOP, so Z must be > 1.4.
            # Using 1.5m ensures it sits on the pillar.
            final_z = 1.5
            
            self._log(f"Placing {package_id} on delivery pillar ({x:.2f}, {y:.2f}, {final_z:.2f})...")
            
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {x}, y: {y}, z: {final_z}}} orientation: {{x: 0, y: 0, z: 0, w: 1}}'
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

    def _backup_robot(self, duration=2.5):
        """Manually drive backward to clear shelf costmap inflation."""
        self._log("Backing up to clear shelf...")
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = -0.3 
        
        end_time = time.time() + duration
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.twist.linear.x = 0.0
        self.vel_pub.publish(msg)
        self._log("Backup complete.")

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

        # ---------------------------------------------------------
        # 1. SPAWN ON SHELF
        # ---------------------------------------------------------
        feedback.stage = "spawning_package"
        feedback.progress = 0.05
        goal_handle.publish_feedback(feedback)

        try:
            tf_shelf = self.tf_buffer.lookup_transform('map', shelf_frame, rclpy.time.Time())
            # Shelf spawning offset (inside shelf)
            shelf_center_x = tf_shelf.transform.translation.x
            shelf_center_y = tf_shelf.transform.translation.y
            
            spawn_x = shelf_center_x + 0.5
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
            # APPROACH POSE: 0.55m in front of shelf center (x - 0.55)
            # This is VERY close to ensure successful picking.
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = 'map'
            approach_pose.header.stamp = self.get_clock().now().to_msg()
            approach_pose.pose.position.x = shelf_center_x - 0.55
            approach_pose.pose.position.y = shelf_center_y
            approach_pose.pose.position.z = 0.0
            
            # Orientation: Face the shelf (Face +X)
            # This forces the robot to look AT the shelf
            q = quaternion_from_euler(0, 0, 0) 
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
        # 4. BACKUP (CLEAR SHELF)
        # ---------------------------------------------------------
        # Facing +X, so negative linear.x moves us away (-X)
        self._backup_robot(1.5)

        # ---------------------------------------------------------
        # 5. GO TO DELIVERY
        # ---------------------------------------------------------
        delivery_x = 0.0
        delivery_y = 0.0
        delivery_z = 0.0
        
        try:
            tf_delivery = self.tf_buffer.lookup_transform('map', delivery_frame, rclpy.time.Time())
            delivery_x = tf_delivery.transform.translation.x
            delivery_y = tf_delivery.transform.translation.y
            delivery_z = tf_delivery.transform.translation.z
            
            # APPROACH POSE: 0.8m in front of delivery point
            # Increased spacing (from 0.6 to 0.8) so robot doesn't stand inside the delivery zone
            delivery_approach = PoseStamped()
            delivery_approach.header.frame_id = 'map'
            delivery_approach.header.stamp = self.get_clock().now().to_msg()
            delivery_approach.pose.position.x = delivery_x - 0.8
            delivery_approach.pose.position.y = delivery_y
            delivery_approach.pose.position.z = 0.0
            
            # Orientation: Face +X (Towards Delivery)
            q = quaternion_from_euler(0, 0, 0) 
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
        
        # Place package AT the actual delivery point coordinates.
        # Fixed Height: 1.5m to land ON TOP of the 1.4m pillar.
        self._place_package_at_delivery(pkg_id, delivery_x, delivery_y, delivery_z)
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