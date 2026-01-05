import asyncio
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

from warebotsim_interfaces.action import FulfillOrder


class FulfillOrderServer(Node):
    def __init__(self):
        super().__init__('fulfill_order_server')

        self.declare_parameter('use_nav2', True)
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)

        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 client
        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group
        )

        # Manual velocity publisher for backing up (TwistStamped to match your Bridge config)
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

    def _backup_robot(self, duration=2.0):
        """Manually reverse the robot to clear obstacles."""
        self._log("Backing up to clear shelf...")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = -0.3 # Move backward
        
        end_time = time.time() + duration
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.twist.linear.x = 0.0
        self.vel_pub.publish(msg)
        self._log("Backup complete.")

    def _spawn_package(self, package_id: str, x: float, y: float, z: float) -> bool:
        """Spawn package at specific coordinates."""
        try:
            # Compact SDF
            sdf_xml = (
                "<?xml version='1.0'?>"
                "<sdf version='1.6'>"
                f"<model name='{package_id}'>"
                f"<pose>{x} {y} {z} 0 0 0</pose>"
                "<link name='link'>"
                "<inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>"
                "<visual name='visual'><geometry><box><size>0.2 0.2 0.2</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>"
                "<collision name='collision'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>"
                "</link>"
                "</model>"
                "</sdf>"
            )

            self._log(f"Spawning {package_id} at ({x:.2f}, {y:.2f}, {z:.2f})...")
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'sdf: "{sdf_xml}"'
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
        """Teleport package from shelf to robot."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            # HIGH Z: 0.6m to ensure it is ABOVE the LIDAR scan plane
            rz = tf.transform.translation.z + 0.6

            self._log(f"Teleporting {package_id} to robot ({rx:.2f}, {ry:.2f})...")
            
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {rx}, y: {ry}, z: {rz}}}'
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

    def remove_package(self, package_name: str):
        try:
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_name}" type: MODEL'
            ]
            self._log(f"Removing package {package_name}...")
            subprocess.run(cmd, check=True, capture_output=True, timeout=5)
            return True
        except Exception as e:
            self.get_logger().error(f"Remove exception: {e}")
            return False

    def _nav2_go_to_frame(self, goal_handle, target_frame: str, stage_prefix: str,
                          p_min: float, p_max: float) -> tuple[bool, str]:
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            return False, "Nav2 action server not available"

        try:
            pose = self._pose_from_tf(target_frame)
        except Exception as e:
            return False, f"TF Lookup failed: {e}"

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

        self._log(f"Sending Nav2 goal to {target_frame}...")
        
        send_future = self.nav2_client.send_goal_async(nav_goal, feedback_callback=nav_feedback_cb)
        while not send_future.done():
            time.sleep(0.1)
        
        nav_goal_handle = send_future.result()

        if not nav_goal_handle.accepted:
            return False, f"Nav2 rejected goal to {target_frame}"

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
        goal = goal_handle.request
        shelf_frame = f"shelf_{int(goal.shelf_id)}"
        delivery_frame = f"delivery_{int(goal.delivery_id)}"
        package_name = f"box_{goal.order_id}"

        result = FulfillOrder.Result()

        if not self._tf_exists(shelf_frame) or not self._tf_exists(delivery_frame):
            result.success = False
            result.message = "TF frame missing"
            goal_handle.succeed()
            return result

        # 1. Spawn on Shelf (Before moving)
        try:
            tf = self.tf_buffer.lookup_transform('map', shelf_frame, rclpy.time.Time())
            # Use EXACT shelf coordinates (removed offset per user request)
            shelf_x = tf.transform.translation.x
            shelf_y = tf.transform.translation.y
            shelf_z = 1.5 
            
            self._spawn_package(package_name, shelf_x, shelf_y, shelf_z)
            time.sleep(1.0)
        except Exception as e:
            self._log(f"Spawn Error: {e}")

        # 2. Go to Shelf
        self._log(f"Step 1: Navigating to {shelf_frame}")
        ok, msg = self._nav2_go_to_frame(goal_handle, shelf_frame, "goto_shelf", 0.0, 0.45)
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 3. Pick - Attach
        self._log(f"Step 2: Picking {package_name}...")
        self._attach_package_to_robot(package_name)
        time.sleep(1.0)

        # 4. BACK UP (Critical Step to Unstuck)
        self._backup_robot(duration=2.0)

        # 5. Go to Delivery
        self._log(f"Step 3: Navigating to {delivery_frame}")
        ok, msg = self._nav2_go_to_frame(goal_handle, delivery_frame, "goto_delivery", 0.55, 0.90)
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 6. Place
        self._log(f"Step 4: Delivering {package_name}")
        self.remove_package(package_name)

        result.success = True
        result.message = "Order Complete"
        goal_handle.succeed()
        self._log("Mission Complete.")
        return result


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