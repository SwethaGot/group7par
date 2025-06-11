import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from par_interfaces.srv import CurrentPose
from geometry_msgs.msg import Pose


class WaypointFinder(Node):
    def __init__(self):
        super().__init__('waypoint_finder')

        # TF listener for object position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action and service clients
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._pose_client = self.create_client(CurrentPose, '/par_moveit/get_current_pose')

        # Wait for the service
        while not self._pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Waiting for CurrentPose service...')

        # Main loop trigger
        self.timer = self.create_timer(1.0, self.plan_once)
        self.has_moved = False

    def plan_once(self):
        if self.has_moved:
            return
        self.has_moved = True

        object_frame = "object_21"

        try:
            # TF Lookup for object position
            trans = self.tf_buffer.lookup_transform(
                "base_link", object_frame, rclpy.time.Time(), timeout=Duration(seconds=1)
            )

            obj_x = trans.transform.translation.x
            obj_y = trans.transform.translation.y
            obj_z = trans.transform.translation.z
            self.get_logger().info(f"📍 Object position: x={obj_x:.3f}, y={obj_y:.3f}, z={obj_z:.3f}")

            # Call current pose service
            req = CurrentPose.Request()
            future = self._pose_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error('❌ Failed to get gripper pose from service.')
                return
            else:
                self.get_logger().info('✅ Received gripper pose from service')

            current_pose = future.result().pose
            self.get_logger().info(
                f"🤖 Gripper at: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}"
            )

            # Target pose: 20 cm above the object
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position.x = obj_x
            target_pose.pose.position.y = obj_y
            target_pose.pose.position.z = obj_z + 0.20
            target_pose.pose.orientation = current_pose.orientation  # maintain orientation

            self.get_logger().info(
                f"🎯 Target pose: x={target_pose.pose.position.x:.3f}, "
                f"y={target_pose.pose.position.y:.3f}, "
                f"z={target_pose.pose.position.z:.3f}"
            )
            self.get_logger().info("🔄 Target pose ready, sending to planner...")
            self.send_pose_to_move_group(target_pose)

        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed during TF or service: {e}")

    def send_pose_to_move_group(self, target_pose: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ MoveGroup action server not available.')
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_manipulator"
        goal.request.allowed_planning_time = 5.0

        # Position constraint (tight sphere)
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0

        # Orientation constraint (same orientation, loose tolerance)
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('🚀 Sending goal to MoveGroup...')
        future = self._action_client.send_goal_async(goal)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('❌ Goal rejected by MoveGroup')
                return

            self.get_logger().info('✅ Goal accepted. Executing...')
            result_future = goal_handle.get_result_async()

            def result_callback(res_fut):
                result = res_fut.result().result
                if result.error_code.val == result.error_code.SUCCESS:
                    self.get_logger().info('🎉 Movement succeeded!')
                else:
                    self.get_logger().error(f'💥 Movement failed. Error code: {result.error_code.val}')

            result_future.add_done_callback(result_callback)

        future.add_done_callback(goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
