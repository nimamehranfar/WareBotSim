import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from warebotsim_interfaces.srv import CreateOrder
from warebotsim_interfaces.action import FulfillOrder


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')

        # Parameters: numbered shelves/deliveries (start with [1] and scale later)
        self.declare_parameter('shelf_ids', [1])
        self.declare_parameter('delivery_ids', [1])

        self.shelf_ids = set(self.get_parameter('shelf_ids').value)
        self.delivery_ids = set(self.get_parameter('delivery_ids').value)

        self._next_order_id = 1

        # Service: the ONLY “user command”
        self._srv = self.create_service(
            CreateOrder,
            '/warehouse/create_order',
            self._handle_create_order
        )

        # Action client: dispatch work to the robot
        self._ac = ActionClient(self, FulfillOrder, '/robot/fulfill_order')

        self.get_logger().info(
            f"Ready. Shelves={sorted(self.shelf_ids)} Deliveries={sorted(self.delivery_ids)}"
        )

    def _handle_create_order(self, request, response):
        shelf_id = int(request.shelf_id)
        delivery_id = int(request.delivery_id)

        if shelf_id not in self.shelf_ids:
            response.accepted = False
            response.order_id = 0
            response.message = f"Unknown shelf_id={shelf_id}"
            return response

        if delivery_id not in self.delivery_ids:
            response.accepted = False
            response.order_id = 0
            response.message = f"Unknown delivery_id={delivery_id}"
            return response

        order_id = self._next_order_id
        self._next_order_id += 1

        package_id = request.package_id.strip() or f"pkg_{order_id:03d}"

        # Dispatch asynchronously (service stays short)
        goal = FulfillOrder.Goal()
        goal.order_id = order_id
        goal.shelf_id = shelf_id
        goal.delivery_id = delivery_id
        goal.package_id = package_id

        if not self._ac.wait_for_server(timeout_sec=0.5):
            response.accepted = False
            response.order_id = order_id
            response.message = "Robot action server not available: /robot/fulfill_order"
            return response

        send_future = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

        response.accepted = True
        response.order_id = order_id
        response.message = f"Order {order_id} accepted: {package_id} shelf_{shelf_id} -> delivery_{delivery_id}"
        return response

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Robot rejected the goal.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[Robot] stage={fb.stage} progress={fb.progress:.2f}")

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"[Robot result] success={result.success} msg='{result.message}'")


def main():
    rclpy.init()
    node = OrderManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
