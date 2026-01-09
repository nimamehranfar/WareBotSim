import math
import subprocess
import time

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


def _wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FulfillOrderServer(Node):
    def __init__(self):
        super().__init__('fulfill_order_server')

        # TF publishing offsets from order_manager.py (do NOT change unless you change both sides)
        self.declare_parameter('shelf_approach_dx', -0.8)
        self.declare_parameter('delivery_approach_dx', +0.8)

        # SAFETY: extra standoff from the approach frame to prevent physical collision
        # (this is exactly the "shelf_fx - 0.55" you tested)
        self.declare_parameter('shelf_goal_extra_backoff', 0.55)
        self.declare_parameter('delivery_goal_extra_backoff', 0.0)

        # Stop Nav2 early to avoid micro-adjusting near the goal
        self.declare_parameter('nav2_early_stop_dist_shelf', 0.35)
        self.declare_parameter('nav2_early_stop_dist_delivery', 0.35)

        # Post-arrival rotate tuning
        self.declare_parameter('final_yaw_tolerance_deg', 1.0)
        self.declare_parameter('final_yaw_timeout_sec', 8.0)
        self.declare_parameter('final_yaw_kp', 2.2)
        self.declare_parameter('final_yaw_max_w', 1.0)
        self.declare_parameter('final_yaw_min_w', 0.25)

        # Delivery placement: cylinder top is 1.4 (pose z=0.7, length 1.4 -> top at 1.4)
        # Put package slightly "pressed" onto the top so it settles centered and doesn’t bounce off the rim.
        self.declare_parameter('delivery_place_z', 1.42)

        # Feedback/log throttling
        self._last_fb_time = 0.0
        self._fb_period_sec = 0.5

        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group
        )

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

        self.get_logger().info("FulfillOrderServer ready")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _log(self, msg: str):
        self.get_logger().info(msg)
        print(f"[FulfillOrderServer] {msg}", flush=True)

    def _tf_exists(self, target_frame: str, source_frame: str = 'map') -> bool:
        try:
            self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def _lookup_map_tf(self, target_frame: str):
        return self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())

    def _yaw_from_quat(self, q) -> float:
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return float(yaw)

    def _get_robot_xy_yaw(self) -> tuple[float, float, float]:
        tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = self._yaw_from_quat(tf.transform.rotation)
        return rx, ry, yaw

    def _publish_cmd(self, lin_x: float, ang_z: float):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(lin_x)
        msg.twist.angular.z = float(ang_z)
        self.vel_pub.publish(msg)

    def _stop_robot(self):
        for _ in range(4):
            self._publish_cmd(0.0, 0.0)
            time.sleep(0.05)

    def _rotate_in_place_to_yaw(self, target_yaw: float) -> bool:
        tol = math.radians(float(self.get_parameter('final_yaw_tolerance_deg').value))
        timeout = float(self.get_parameter('final_yaw_timeout_sec').value)
        kp = float(self.get_parameter('final_yaw_kp').value)
        w_max = float(self.get_parameter('final_yaw_max_w').value)
        w_min = float(self.get_parameter('final_yaw_min_w').value)

        start = time.time()
        last_log = 0.0

        self._stop_robot()

        while (time.time() - start) < timeout:
            try:
                _, _, yaw = self._get_robot_xy_yaw()
            except Exception:
                self._publish_cmd(0.0, 0.0)
                time.sleep(0.05)
                continue

            err = _wrap_to_pi(target_yaw - yaw)
            if abs(err) <= tol:
                self._stop_robot()
                self._log(f"Final yaw OK: current={yaw:.3f} target={target_yaw:.3f} err={err:.3f}")
                return True

            w = kp * err
            w = max(-w_max, min(w_max, w))

            if 0.0 < abs(w) < w_min:
                w = w_min if w > 0.0 else -w_min

            self._publish_cmd(0.0, w)

            now = time.time()
            if (now - last_log) > 0.2:
                last_log = now
                self._log(f"[yaw] current={yaw:.3f} target={target_yaw:.3f} err={err:.3f} cmd_w={w:.3f}")

            time.sleep(0.05)

        self._stop_robot()
        self._log("Final yaw alignment TIMEOUT")
        return False

    def _spawn_package(self, package_id: str, x: float, y: float, z: float) -> bool:
        try:
            sdf_content = (
                "<?xml version='1.0'?>"
                "<sdf version='1.6'>"
                f"<model name='{package_id}'>"
                f"<pose>{x} {y} {z} 0 0 0</pose>"
                "<link name='link'>"
                "<inertial><mass>0.5</mass><inertia>"
                "<ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz>"
                "</inertia></inertial>"
                "<collision name='collision'><geometry><box><size>0.15 0.15 0.15</size></box></geometry></collision>"
                "<visual name='visual'><geometry><box><size>0.15 0.15 0.15</size></box></geometry>"
                "<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material>"
                "</visual>"
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
            return result.returncode == 0
        except Exception as e:
            self.get_logger().error(f"Spawn exception: {e}")
            return False

    def _set_model_pose(self, name: str, x: float, y: float, z: float, yaw: float) -> bool:
        try:
            q = quaternion_from_euler(0.0, 0.0, yaw)
            cmd = [
                'gz', 'service', '-s', '/world/warehouse_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                (
                    f'name: "{name}", '
                    f'position: {{x: {x}, y: {y}, z: {z}}} '
                    f'orientation: {{x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}}}'
                )
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except Exception as e:
            self.get_logger().error(f"set_pose exception: {e}")
            return False

    def _attach_package_to_robot(self, package_id: str) -> bool:
        try:
            rx, ry, yaw = self._get_robot_xy_yaw()

            offset_back = -0.12
            x = rx + offset_back * math.cos(yaw)
            y = ry + offset_back * math.sin(yaw)
            z = 0.30

            self._log(f"Attach {package_id} -> robot at ({x:.2f}, {y:.2f}) yaw={yaw:.3f}")
            return self._set_model_pose(package_id, x, y, z, yaw)
        except Exception as e:
            self.get_logger().error(f"Attach exception: {e}")
            return False

    def _backup_robot(self, duration: float = 1.2):
        self._log("Backing up...")
        end_t = time.time() + float(duration)
        while time.time() < end_t:
            self._publish_cmd(-0.25, 0.0)
            time.sleep(0.1)
        self._stop_robot()

    def _place_package_at_delivery_center(self, package_id: str, delivery_center_x: float, delivery_center_y: float) -> bool:
        try:
            _, _, yaw = self._get_robot_xy_yaw()
            z = float(self.get_parameter('delivery_place_z').value)
            self._log(
                f"Place {package_id} -> delivery center ({delivery_center_x:.2f}, {delivery_center_y:.2f}, {z:.3f}) yaw={yaw:.3f}"
            )
            return self._set_model_pose(package_id, delivery_center_x, delivery_center_y, z, yaw)
        except Exception as e:
            self.get_logger().error(f"Delivery place exception: {e}")
            return False

    def _nav2_go_to_pose_with_early_stop(
        self,
        goal_handle,
        pose: PoseStamped,
        stage_prefix: str,
        p_min: float,
        p_max: float,
        early_stop_dist: float
    ) -> tuple[bool, str]:
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            return False, "Nav2 action server not available"

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        fb = FulfillOrder.Feedback()
        initial_dist = {'value': None}

        def nav_feedback_cb(msg):
            now = time.time()
            if (now - self._last_fb_time) < self._fb_period_sec:
                return
            self._last_fb_time = now
            try:
                dist = float(msg.feedback.distance_remaining)
                if initial_dist['value'] is None:
                    initial_dist['value'] = max(dist, 0.001)
                ratio = 1.0 - min(dist / initial_dist['value'], 1.0)
                fb.stage = stage_prefix
                fb.progress = p_min + (p_max - p_min) * ratio
                goal_handle.publish_feedback(fb)
            except Exception:
                pass

        gx = float(pose.pose.position.x)
        gy = float(pose.pose.position.y)

        self._log(f"Nav2 goal: ({gx:.2f}, {gy:.2f}) early_stop_dist={early_stop_dist:.2f}")

        send_future = self.nav2_client.send_goal_async(nav_goal, feedback_callback=nav_feedback_cb)
        while not send_future.done():
            time.sleep(0.05)

        nav_goal_handle = send_future.result()
        if not nav_goal_handle.accepted:
            return False, "Nav2 rejected goal"

        result_future = nav_goal_handle.get_result_async()

        # Monitor distance ourselves; cancel early to avoid micro-adjusting
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return False, "Canceled"

            try:
                rx, ry, _ = self._get_robot_xy_yaw()
                dist = math.hypot(gx - rx, gy - ry)
                if dist <= early_stop_dist:
                    self._log(f"Early stop: dist={dist:.3f} <= {early_stop_dist:.3f}. Canceling Nav2 goal.")
                    nav_goal_handle.cancel_goal_async()
                    self._stop_robot()
                    return True, "ok(early_stop)"
            except Exception:
                pass

            time.sleep(0.15)

        res = result_future.result()
        status = res.status
        if status != 4:
            return False, f"Nav2 failed (status={status})"
        return True, "ok"

    def execute_callback(self, goal_handle):
        self._log("Received Action Goal")
        req = goal_handle.request

        shelf_frame = f"shelf_{int(req.shelf_id)}"
        delivery_frame = f"delivery_{int(req.delivery_id)}"
        pkg_id = req.package_id if req.package_id else f"pkg_{req.order_id}"

        if not self._tf_exists(shelf_frame) or not self._tf_exists(delivery_frame):
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message="TF Missing")

        shelf_dx = float(self.get_parameter('shelf_approach_dx').value)
        deliv_dx = float(self.get_parameter('delivery_approach_dx').value)

        shelf_backoff = float(self.get_parameter('shelf_goal_extra_backoff').value)
        delivery_backoff = float(self.get_parameter('delivery_goal_extra_backoff').value)

        early_shelf = float(self.get_parameter('nav2_early_stop_dist_shelf').value)
        early_delivery = float(self.get_parameter('nav2_early_stop_dist_delivery').value)

        feedback = FulfillOrder.Feedback()

        # ---------------------------------------------------------
        # 1) SPAWN ON SHELF (keep your working logic)
        # ---------------------------------------------------------
        feedback.stage = "spawning_package"
        feedback.progress = 0.05
        goal_handle.publish_feedback(feedback)

        try:
            tf_shelf = self._lookup_map_tf(shelf_frame)
            shelf_fx = tf_shelf.transform.translation.x
            shelf_fy = tf_shelf.transform.translation.y

            # Working spawn logic (shelf TF is approach frame left of shelf)
            spawn_x = shelf_fx + 0.5
            spawn_y = shelf_fy
            spawn_z = 1.2

            self._spawn_package(pkg_id, spawn_x, spawn_y, spawn_z)
            time.sleep(0.8)
        except Exception as e:
            self._log(f"Spawn Error: {e}")

        # ---------------------------------------------------------
        # 2) GO TO SHELF SAFELY (do NOT collide)
        # ---------------------------------------------------------
        try:
            tf_shelf = self._lookup_map_tf(shelf_frame)
            shelf_fx = tf_shelf.transform.translation.x
            shelf_fy = tf_shelf.transform.translation.y

            # Safety: move further away from shelf than the TF point
            shelf_goal = PoseStamped()
            shelf_goal.header.frame_id = 'map'
            shelf_goal.header.stamp = self.get_clock().now().to_msg()
            shelf_goal.pose.position.x = float(shelf_fx - shelf_backoff)
            shelf_goal.pose.position.y = float(shelf_fy)
            shelf_goal.pose.position.z = 0.0

            # Keep shelf approach yaw (0)
            shelf_goal.pose.orientation = tf_shelf.transform.rotation

            ok, msg = self._nav2_go_to_pose_with_early_stop(goal_handle, shelf_goal, "goto_shelf", 0.10, 0.45, early_shelf)
            if not ok:
                goal_handle.abort()
                return FulfillOrder.Result(success=False, message=msg)

            # Force facing shelf: shelves are axis-aligned yaw=0 in your world
            target_yaw = 0.0
            self._log(f"Align to shelf yaw=0.0 (fixed).")
            self._rotate_in_place_to_yaw(target_yaw)

        except Exception as e:
            self.get_logger().error(f"Shelf Nav Error: {e}")
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=str(e))

        # ---------------------------------------------------------
        # 3) PICK (ATTACH) - live pose+yaw
        # ---------------------------------------------------------
        feedback.stage = "picking"
        feedback.progress = 0.50
        goal_handle.publish_feedback(feedback)

        time.sleep(0.4)
        self._attach_package_to_robot(pkg_id)
        time.sleep(0.3)

        self._backup_robot(1.2)

        # ---------------------------------------------------------
        # 4) GO TO DELIVERY SAFELY
        # ---------------------------------------------------------
        try:
            tf_delivery = self._lookup_map_tf(delivery_frame)
            deliv_fx = tf_delivery.transform.translation.x
            deliv_fy = tf_delivery.transform.translation.y

            delivery_goal = PoseStamped()
            delivery_goal.header.frame_id = 'map'
            delivery_goal.header.stamp = self.get_clock().now().to_msg()
            delivery_goal.pose.position.x = float(deliv_fx - delivery_backoff)
            delivery_goal.pose.position.y = float(deliv_fy)
            delivery_goal.pose.position.z = 0.0
            delivery_goal.pose.orientation = tf_delivery.transform.rotation

            ok, msg = self._nav2_go_to_pose_with_early_stop(goal_handle, delivery_goal, "goto_delivery", 0.55, 0.90, early_delivery)
            if not ok:
                goal_handle.abort()
                return FulfillOrder.Result(success=False, message=msg)

            # Force facing delivery inward: delivery approach yaw is pi in order_manager
            target_yaw = math.pi
            self._log("Align to delivery yaw=pi (fixed).")
            self._rotate_in_place_to_yaw(target_yaw)

        except Exception as e:
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=f"Delivery Nav Error: {e}")

        # ---------------------------------------------------------
        # 5) PLACE (deliver)
        # ---------------------------------------------------------
        feedback.stage = "delivering"
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)

        try:
            tf_delivery = self._lookup_map_tf(delivery_frame)
            deliv_fx = tf_delivery.transform.translation.x
            deliv_fy = tf_delivery.transform.translation.y

            # True model center from TF approach frame and dx:
            # TF is center + deliv_dx, so center = TF - deliv_dx
            delivery_center_x = float(deliv_fx - deliv_dx)
            delivery_center_y = float(deliv_fy)

            self._place_package_at_delivery_center(pkg_id, delivery_center_x, delivery_center_y)
        except Exception as e:
            goal_handle.abort()
            return FulfillOrder.Result(success=False, message=f"Place Error: {e}")

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
