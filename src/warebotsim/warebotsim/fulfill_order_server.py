import asyncio
import math
import subprocess
import time
import sys

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


class FulfillOrderServer(Node):
    def __init__(self):
        super().__init__('fulfill_order_server')

        self.declare_parameter('use_nav2', True)
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)

        # Use Reentrant group to allow nested callbacks (ActionClient inside ActionServer)
        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.get_logger().info(f"Robot ready. use_nav2={self.use_nav2}")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _log(self, msg):
        self.get_logger().info(msg)
        # Force flush stdout just in case
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

    def spawn_package_on_robot(self, package_name: str):
        """
        Spawns a box model on top of the robot.
        """
        try:
            # 1. Get Robot Position
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            rz = 0.4 # Height

            # 2. Compact SDF
            sdf_xml = (
                "<?xml version='1.0'?>"
                "<sdf version='1.6'>"
                f"<model name='{package_name}'>"
                f"<pose>{rx} {ry} {rz} 0 0 0</pose>"
                "<link name='link'>"
                "<inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>"
                "<visual name='visual'><geometry><box><size>0.2 0.2 0.2</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>"
                "<collision name='collision'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>"
                "</link>"
                "</model>"
                "</sdf>"
            )

            # 3. Execute Command
            self._log(f"Spawning {package_name} at ({rx:.2f}, {ry:.2f})...")
            
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'sdf: "{sdf_xml}"'
            ]

            # Added timeout to prevent hanging
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self._log("Spawn command executed successfully.")
                return True
            else:
                self.get_logger().error(f"Spawn command FAILED: {result.stderr}")
                return False

        except subprocess.TimeoutExpired:
            self.get_logger().error("Spawn command TIMED OUT.")
            return False
        except Exception as e:
            self.get_logger().error(f"Spawn exception: {e}")
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

    async def _nav2_go_to_frame(self, goal_handle, target_frame: str, stage_prefix: str,
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
            # Try to get distance, handle if missing
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
        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            return False, f"Nav2 rejected goal to {target_frame}"

        result_future = nav_goal_handle.get_result_async()

        # Wait loop
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return False, "Canceled"
            # Increase sleep slightly to yield CPU
            await asyncio.sleep(0.5)

        res = await result_future
        status = res.status
        self._log(f"Nav2 finished with status: {status}")

        if status != 4: # 4 = SUCCEEDED
            return False, f"Nav2 failed (status={status})"

        return True, "ok"

    async def execute_callback(self, goal_handle):
        self._log("Received Action Goal")
        goal = goal_handle.request
        shelf_frame = f"shelf_{int(goal.shelf_id)}"
        delivery_frame = f"delivery_{int(goal.delivery_id)}"
        package_name = f"box_{goal.order_id}"

        result = FulfillOrder.Result()

        # 1. Validate
        if not self._tf_exists(shelf_frame) or not self._tf_exists(delivery_frame):
            result.success = False
            result.message = "TF frame missing"
            goal_handle.succeed()
            return result

        # 2. Go to Shelf
        self._log(f"Step 1: Navigating to {shelf_frame}")
        ok, msg = await self._nav2_go_to_frame(goal_handle, shelf_frame, "goto_shelf", 0.0, 0.45)
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 3. Pick
        self._log(f"Step 2: Spawning {package_name}")
        spawn_success = self.spawn_package_on_robot(package_name)
        if not spawn_success:
            self._log("Spawn failed! Check Gazebo logs.")
        
        # Wait a bit for physics
        time.sleep(1.0)

        # 4. Go to Delivery
        self._log(f"Step 3: Navigating to {delivery_frame}")
        ok, msg = await self._nav2_go_to_frame(goal_handle, delivery_frame, "goto_delivery", 0.55, 0.90)
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 5. Place
        self._log(f"Step 4: Delivering {package_name}")
        self.remove_package(package_name)

        result.success = True
        result.message = "Order Complete"
        goal_handle.succeed()
        self._log("Action Complete.")
        return result


def main():
    rclpy.init()
    node = FulfillOrderServer()
    # Use MultiThreadedExecutor to ensure ActionServer and ActionClient callbacks don't block each other
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