import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from warebotsim_interfaces.action import FulfillOrder

import subprocess
import time


class FulfillOrderServer(Node):
    def __init__(self):
        super().__init__('fulfill_order_server')

        # Default True now: we are in Nav2 phase
        self.declare_parameter('use_nav2', True)
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)

        self.cb_group = ReentrantCallbackGroup()

        # TF: we will look up target frames in the MAP frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client: BT Navigator exposes /navigate_to_pose
        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group
        )

        self._as = ActionServer(
            self,
            FulfillOrder,
            '/robot/fulfill_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # Track current package being carried
        self.current_package = None

        self.get_logger().info(f"Robot ready. use_nav2={self.use_nav2}")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _tf_exists(self, target_frame: str, source_frame: str = 'map') -> bool:
        try:
            _ = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
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

    def _spawn_package(self, package_id: str, x: float, y: float, z: float) -> bool:
        """Spawn a package (small box) at the given location using Gazebo."""
        try:
            # Create a simple box model SDF
            sdf_content = f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{package_id}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.15 0.15 0.15</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.15 0.15 0.15</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.3 0.1 1</ambient>
          <diffuse>0.8 0.3 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

            # Write SDF to temp file
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
                f.write(sdf_content)
                sdf_file = f.name

            # Spawn using gz service
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', f'sdf_filename:"{sdf_file}", name:"{package_id}"'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            # Clean up temp file
            import os
            os.unlink(sdf_file)
            
            if result.returncode == 0:
                self.get_logger().info(f"Spawned package {package_id} at ({x}, {y}, {z})")
                return True
            else:
                self.get_logger().error(f"Failed to spawn package: {result.stderr}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Exception spawning package: {e}")
            return False

    def _attach_package_to_robot(self, package_id: str) -> bool:
        """Attach package to robot by moving it to robot's position."""
        try:
            # Get robot's current position from TF
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
            robot_z = tf.transform.translation.z
            
            # Place package on top of robot
            package_z = robot_z + 0.3  # 30cm above robot base
            
            # Use Gazebo service to set pose
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {robot_x}, y: {robot_y}, z: {package_z}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0:
                self.get_logger().info(f"Attached package {package_id} to robot")
                self.current_package = package_id
                return True
            else:
                self.get_logger().warn(f"Package attach result: {result.stderr}")
                self.current_package = package_id
                return True  # Continue anyway
                
        except Exception as e:
            self.get_logger().error(f"Exception attaching package: {e}")
            return False

    def _place_package_at_delivery(self, package_id: str, delivery_frame: str) -> bool:
        """Place package at delivery location."""
        try:
            # Get delivery point position from TF
            tf = self.tf_buffer.lookup_transform('map', delivery_frame, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = 0.1  # Ground level
            
            # Place package at delivery point
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{package_id}", position: {{x: {x}, y: {y}, z: {z}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0:
                self.get_logger().info(f"Placed package {package_id} at {delivery_frame}")
                self.current_package = None
                return True
            else:
                self.get_logger().warn(f"Package place result: {result.stderr}")
                self.current_package = None
                return True  # Continue anyway
                
        except Exception as e:
            self.get_logger().error(f"Exception placing package: {e}")
            return False

    async def _nav2_go_to_frame(self, goal_handle, target_frame: str, stage_prefix: str,
                               p_min: float, p_max: float) -> tuple[bool, str]:
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            return False, "Nav2 action server not available: /navigate_to_pose"

        pose = self._pose_from_tf(target_frame)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        fb = FulfillOrder.Feedback()
        initial_dist = {'value': None}

        def nav_feedback_cb(msg):
            # NavigateToPose feedback includes distance_remaining in Humble+ (and later). 
            dist = float(msg.feedback.distance_remaining)
            if initial_dist['value'] is None:
                initial_dist['value'] = max(dist, 0.001)

            ratio = 1.0 - min(dist / initial_dist['value'], 1.0)
            fb.stage = f"{stage_prefix} dist={dist:.2f}m"
            fb.progress = p_min + (p_max - p_min) * ratio
            goal_handle.publish_feedback(fb)

        send_future = self.nav2_client.send_goal_async(nav_goal, feedback_callback=nav_feedback_cb)
        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            return False, f"Nav2 rejected goal to {target_frame}"

        result_future = nav_goal_handle.get_result_async()

        # Wait while allowing cancel
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                _ = nav_goal_handle.cancel_goal_async()
                fb.stage = "canceled"
                fb.progress = p_min
                goal_handle.publish_feedback(fb)
                goal_handle.canceled()
                return False, "Canceled by client"
            await asyncio.sleep(0.1)

        res = await result_future
        status = res.status

        # status codes are actionlib style; 4 is SUCCEEDED in ROS 2 actions
        if status != 4:
            return False, f"Nav2 failed (status={status}) while going to {target_frame}"

        fb.stage = f"{stage_prefix} arrived"
        fb.progress = p_max
        goal_handle.publish_feedback(fb)
        return True, "ok"

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        shelf_frame = f"shelf_{int(goal.shelf_id)}"
        delivery_frame = f"delivery_{int(goal.delivery_id)}"
        package_id = goal.package_id

        feedback = FulfillOrder.Feedback()
        result = FulfillOrder.Result()

        # 1) Validate targets exist in TF
        feedback.stage = "validating_targets"
        feedback.progress = 0.05
        goal_handle.publish_feedback(feedback)

        if not self._tf_exists(shelf_frame, 'map'):
            result.success = False
            result.message = f"Missing TF frame in map: {shelf_frame}"
            goal_handle.succeed()
            return result

        if not self._tf_exists(delivery_frame, 'map'):
            result.success = False
            result.message = f"Missing TF frame in map: {delivery_frame}"
            goal_handle.succeed()
            return result

        if not self.use_nav2:
            result.success = False
            result.message = "use_nav2=false (Nav2 disabled)"
            goal_handle.succeed()
            return result

        # 1.5) Spawn package at shelf location
        feedback.stage = "spawning_package"
        feedback.progress = 0.08
        goal_handle.publish_feedback(feedback)
        
        try:
            # Get shelf position to spawn package
            tf = self.tf_buffer.lookup_transform('map', shelf_frame, rclpy.time.Time())
            shelf_x = tf.transform.translation.x
            shelf_y = tf.transform.translation.y
            # Spawn package at shelf height
            package_z = 0.8  # Shelf height from your SDF
            
            spawn_ok = self._spawn_package(package_id, shelf_x, shelf_y, package_z)
            if not spawn_ok:
                self.get_logger().warn(f"Failed to spawn package, but continuing...")
            
            # Small delay to let Gazebo process the spawn
            await asyncio.sleep(0.5)
            
        except Exception as e:
            self.get_logger().error(f"Error spawning package: {e}")

        # 2) Navigate to shelf
        ok, msg = await self._nav2_go_to_frame(
            goal_handle, shelf_frame, f"navigating_to_{shelf_frame}", 0.10, 0.45
        )
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 3) Pick - attach package to robot
        feedback.stage = "picking"
        feedback.progress = 0.50
        goal_handle.publish_feedback(feedback)
        
        # Wait a moment at shelf
        await asyncio.sleep(1.0)
        
        # Attach package to robot
        attach_ok = self._attach_package_to_robot(package_id)
        if not attach_ok:
            result.success = False
            result.message = f"Failed to attach package {package_id}"
            goal_handle.succeed()
            return result
        
        # Wait to show package attached
        await asyncio.sleep(0.5)

        # 4) Navigate to delivery
        ok, msg = await self._nav2_go_to_frame(
            goal_handle, delivery_frame, f"navigating_to_{delivery_frame}", 0.55, 0.90
        )
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 5) Place package at delivery point
        feedback.stage = "placing"
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)
        
        # Wait a moment at delivery
        await asyncio.sleep(1.0)
        
        # Place package
        place_ok = self._place_package_at_delivery(package_id, delivery_frame)
        if not place_ok:
            result.success = False
            result.message = f"Failed to place package {package_id}"
            goal_handle.succeed()
            return result

        result.success = True
        result.message = f"Delivered {goal.package_id} from {shelf_frame} to {delivery_frame}"
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = FulfillOrderServer()

    # Needed because this node is both an ActionServer and ActionClient
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()