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

        # 2) Navigate to shelf
        ok, msg = await self._nav2_go_to_frame(
            goal_handle, shelf_frame, f"navigating_to_{shelf_frame}", 0.10, 0.45
        )
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 3) Pick (logic later; keep immediate for now)
        feedback.stage = "picking"
        feedback.progress = 0.50
        goal_handle.publish_feedback(feedback)

        # 4) Navigate to delivery
        ok, msg = await self._nav2_go_to_frame(
            goal_handle, delivery_frame, f"navigating_to_{delivery_frame}", 0.55, 0.90
        )
        if not ok:
            result.success = False
            result.message = msg
            goal_handle.succeed()
            return result

        # 5) Place (logic later; keep immediate for now)
        feedback.stage = "placing"
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)

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
