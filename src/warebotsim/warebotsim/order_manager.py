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

        # Parameters: numbered shelves/deliveries (start with [1] and scale later)
        self.declare_parameter('shelf_ids', [1])
        self.declare_parameter('delivery_ids', [1])

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_frames()

        self.shelf_ids = set(self.get_parameter('shelf_ids').value)
        self.delivery_ids = set(self.get_parameter('delivery_ids').value)

        self._next_order_id = 1

        # Service: user command entry-point
        self._srv = self.create_service(
            CreateOrder,
            '/warehouse/create_order',
            self._handle_create_order
        )

        # Action client: dispatch to robot
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
        self.get_logger().info(f"[Robot] stage={fb.stage} progress={fb.progress:.2f}")

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"[Robot result] success={result.success} msg='{result.message}'")

    def publish_static_frames(self):
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

        # LiDAR mount (matches model.sdf pose for lidar_link)
        send('base_link', 'lidar_link', 0.20, 0.0, 0.25, 0.0)

        # IMPORTANT: ros_gz_bridge produces LaserScan frame_id like "jackal/lidar_link/lidar"
        # SLAM Toolbox must be able to transform scan frame -> base_link.
        send('base_link', 'jackal/lidar_link/lidar', 0.20, 0.0, 0.25, 0.0)

        # World alignment (keep your convention)
        send('map', 'world', 0.0, 0.0, 0.0, 0.0)

        # Shelves (match warehouse_world.sdf)
        send('world', 'shelf_1',  2.0,  0.0, 1.0, 0.0)
        send('world', 'shelf_2',  2.0, -2.0, 1.0, 0.0)

        # Delivery points
        send('world', 'delivery_1', -4.0,  0.0, 0.025, 0.0)
        send('world', 'delivery_2', -4.0, -2.0, 0.025, 0.0)


def main():
    rclpy.init()
    node = OrderManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
