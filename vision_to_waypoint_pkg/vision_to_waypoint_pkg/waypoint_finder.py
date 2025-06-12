import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from par_interfaces.srv import CurrentPose


class WaypointFinder(Node):
    def __init__(self):
        super().__init__('waypoint_finder')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_client = self.create_client(CurrentPose, '/par_moveit/get_current_pose')
        while not self.pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("⏳ Waiting for CurrentPose service...")

        self.timer = self.create_timer(1.0, self.find_once)
        self.has_logged = False

    def find_once(self):
        if self.has_logged:
            return

        object_frame = "object_21"
        try:
            # Lookup object pose from TF
            trans = self.tf_buffer.lookup_transform(
                "base_link", object_frame, rclpy.time.Time(), timeout=Duration(seconds=1)
            )

            obj_x = trans.transform.translation.x
            obj_y = trans.transform.translation.y
            obj_z = trans.transform.translation.z
            hover_z = obj_z + 0.20

            self.get_logger().info(
                f"📍 Object position: x={obj_x:.3f}, y={obj_y:.3f}, z={obj_z:.3f}"
            )
            self.get_logger().info(
                f"⬆️  Hover target (20cm above): x={obj_x:.3f}, y={obj_y:.3f}, z={hover_z:.3f}"
            )

            # Attempt to fetch gripper orientation
            req = CurrentPose.Request()
            future = self.pose_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is None:
                self.get_logger().warn("⚠️ Could not get gripper orientation. Skipping orientation log.")
            else:
                orientation = future.result().pose.orientation
                self.get_logger().info(
                    f"🎯 Current orientation: x={orientation.x:.3f}, y={orientation.y:.3f}, "
                    f"z={orientation.z:.3f}, w={orientation.w:.3f}"
                )

            self.has_logged = True

        except Exception as e:
            self.get_logger().error(f"❌ Failed to get TF or pose: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
