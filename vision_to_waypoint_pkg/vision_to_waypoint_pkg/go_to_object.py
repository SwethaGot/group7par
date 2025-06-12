import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from par_interfaces.srv import CurrentPose
from par_interfaces.action import WaypointMove

class GoToObjectNode(Node):
    def __init__(self):
        super().__init__('go_to_object_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_client = self.create_client(CurrentPose, '/par_moveit/get_current_pose')
        self.move_client = ActionClient(self, WaypointMove, '/par_moveit/waypoint_move')

        while not self.pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("⏳ Waiting for CurrentPose service...")

        self.timer = self.create_timer(1.0, self.move_once)
        self.has_moved = False

    def move_once(self):
        if self.has_moved:
            return

        object_frame = "object_21"
        source_frame = "camera_color_optical_frame"
        target_frame = "base_link"

        try:
            # TF lookup: object in camera frame
            obj_trans = self.tf_buffer.lookup_transform(
                source_frame, object_frame, rclpy.time.Time(), timeout=Duration(seconds=1)
            )

            pose_cam = PoseStamped()
            pose_cam.header.frame_id = source_frame
            pose_cam.pose.position.x = obj_trans.transform.translation.x
            pose_cam.pose.position.y = obj_trans.transform.translation.y
            pose_cam.pose.position.z = obj_trans.transform.translation.z - 0.20  # 20cm above
            pose_cam.pose.orientation.w = 1.0

            self.get_logger().info(
                f"📷 Object pose in camera frame: x={pose_cam.pose.position.x:.3f}, "
                f"y={pose_cam.pose.position.y:.3f}, z={pose_cam.pose.position.z:.3f}"
            )

            # Transform to base frame
            pose_base = self.tf_buffer.transform(
                pose_cam, target_frame, timeout=Duration(seconds=1)
            )

            x, y, z = (
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z
            )

            self.get_logger().info(
                f"🦾 Transformed pose in base_link: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

            # Get current gripper orientation
            req = CurrentPose.Request()
            future = self.pose_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if not future.result():
                self.get_logger().error("❌ Failed to get gripper orientation.")
                return

            orientation = future.result().pose.orientation
            self.get_logger().info("✅ Got orientation. Sending move goal...")

            # Send WaypointMove goal
            goal = WaypointMove.Goal()
            goal.target_pose = Pose()
            goal.target_pose.position.x = x
            goal.target_pose.position.y = y
            goal.target_pose.position.z = z
            goal.target_pose.orientation = orientation
            goal.rotation = 0.0

            self.get_logger().info(
                f"🚀 Sending hover goal: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

            self.move_client.wait_for_server()
            move_future = self.move_client.send_goal_async(goal)
            move_future.add_done_callback(self.move_response_callback)

            self.has_moved = True

        except Exception as e:
            self.get_logger().error(f"⚠️ TF or planning failed: {e}")

    def move_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Move goal rejected.")
            return

        self.get_logger().info("✅ Goal accepted. Executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"🎯 Move done: x={result.final_pose.position.x:.3f}, "
            f"y={result.final_pose.position.y:.3f}, z={result.final_pose.position.z:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = GoToObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
