import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from warebotsim_interfaces.srv import CreateOrder
from warebotsim_interfaces.action import FulfillOrder


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')

        # Updated to include ID 3
        self.declare_parameter('shelf_ids', [1, 2, 3])
        self.declare_parameter('delivery_ids', [1, 2, 3])

        self.shelf_ids = set(self.get_parameter('shelf_ids').value)
        self.delivery_ids = set(self.get_parameter('delivery_ids').value)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_frames()

        self._next_order_id = 1
        self._last_feedback_time = 0.0

        self._srv = self.create_service(
            CreateOrder,
            '/warehouse/create_order',
            self._handle_create_order
        )

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
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Log every 2 seconds to reduce spam
        if current_time - self._last_feedback_time >= 2.0:
            self.get_logger().info(f"[Robot] {fb.stage} | progress={fb.progress:.2f}")
            self._last_feedback_time = current_time

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"[Robot result] success={result.success} msg='{result.message}'")

    def _publish_static_frames(self):
        def send(parent, child, x, y, z, yaw):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child

            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)

            self.static_tf_broadcaster.sendTransform(t)

        # Base frames
        send('base_link', 'base_footprint', 0.0, 0.0, 0.0, 0.0)
        
        # UPDATED: LIDAR is now high up at 0.75m
        send('base_link', 'lidar_link', 0.20, 0.0, 0.75, 0.0)
        send('base_link', 'jackal/lidar_link/lidar', 0.20, 0.0, 0.75, 0.0)

        # UPDATED COORDINATES from warehouse_world.sdf
        shelves = {
            1: (2.2, 1.5),
            2: (1.8, -0.2),
            3: (2.5, -2.0),
        }
        deliveries = {
            1: (-2.2, 2.0),
            2: (-1.8, 0.0),
            3: (-2.5, -2.2),
        }

        # Publish semantic frames in MAP frame (CENTERS, no approach offsets).
        # All approach offsets are handled in fulfill_order_server to avoid double-offset confusion.
        for sid, (x, y) in shelves.items():
            send('map', f'shelf_{sid}', x, y, 0.0, 0.0)

        for did, (x, y) in deliveries.items():
            send('map', f'delivery_{did}', x, y, 0.0, math.pi)


def main():
    rclpy.init()
    node = OrderManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()