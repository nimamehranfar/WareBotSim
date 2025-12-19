import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from warebotsim_interfaces.action import FulfillOrder

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FulfillOrderServer(Node):
    def __init__(self):
        super().__init__('fulfill_order_server')

        # Nav2 later: keep the switch now (default False)
        self.declare_parameter('use_nav2', False)
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)

        # TF lookup to validate semantic locations exist (world -> shelf_N / delivery_N)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._as = ActionServer(
            self,
            FulfillOrder,
            '/robot/fulfill_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(f"Robot ready. use_nav2={self.use_nav2}")

    def goal_callback(self, goal_request):
        # Accept all well-formed goals; validate inside execute for better messaging
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _tf_exists(self, target_frame: str, source_frame: str = 'world') -> bool:
        try:
            _ = self.tf_buffer.lookup_transform(
                source_frame, target_frame, rclpy.time.Time()
            )
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        shelf_frame = f"shelf_{int(goal.shelf_id)}"
        delivery_frame = f"delivery_{int(goal.delivery_id)}"

        feedback = FulfillOrder.Feedback()
        result = FulfillOrder.Result()

        # 1) Validate semantic TF targets exist
        feedback.stage = "validating_targets"
        feedback.progress = 0.05
        goal_handle.publish_feedback(feedback)

        if not self._tf_exists(shelf_frame):
            result.success = False
            result.message = f"Missing TF frame: {shelf_frame}"
            goal_handle.succeed()
            return result

        if not self._tf_exists(delivery_frame):
            result.success = False
            result.message = f"Missing TF frame: {delivery_frame}"
            goal_handle.succeed()
            return result

        # 2) Navigate to shelf (sim now, Nav2 later)
        feedback.stage = f"navigating_to_{shelf_frame}"
        feedback.progress = 0.20
        goal_handle.publish_feedback(feedback)

        self._simulate_or_nav2(shelf_frame)

        # 3) Pick
        feedback.stage = "picking"
        feedback.progress = 0.50
        goal_handle.publish_feedback(feedback)
        time.sleep(1.0)

        # 4) Navigate to delivery
        feedback.stage = f"navigating_to_{delivery_frame}"
        feedback.progress = 0.70
        goal_handle.publish_feedback(feedback)

        self._simulate_or_nav2(delivery_frame)

        # 5) Place
        feedback.stage = "placing"
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)
        time.sleep(1.0)

        result.success = True
        result.message = (
            f"Delivered {goal.package_id} from {shelf_frame} to {delivery_frame}"
        )
        goal_handle.succeed()
        return result

    def _simulate_or_nav2(self, target_frame: str):
        # For now: deterministic “fake navigation” so RViz/TF is the focus.
        # Later: replace this body with a Nav2 NavigateToPose action client call.
        if not self.use_nav2:
            time.sleep(1.5)
            return

        # Placeholder for Nav2 integration step:
        # - look up world->target_frame transform
        # - build PoseStamped goal in 'world'
        # - send goal to /navigate_to_pose (nav2_msgs/action/NavigateToPose)
        # Nav2 uses NavigateToPose action definition in nav2_msgs. :contentReference[oaicite:8]{index=8}
        time.sleep(1.5)


def main():
    rclpy.init()
    node = FulfillOrderServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
