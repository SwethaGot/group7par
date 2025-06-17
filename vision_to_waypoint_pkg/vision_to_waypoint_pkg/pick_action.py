# pick_action.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from par_interfaces.action import WaypointMove
from onrobot_rg2_msgs.action import GripperSetWidth

class PickActionNode(Node):
    def __init__(self):
        super().__init__('pick_action_node')

        self.move_client = ActionClient(self, WaypointMove, '/par_moveit/waypoint_move')
        self.gripper_client = ActionClient(self, GripperSetWidth, '/rg2/set_width')

    def pick_at(self, pose):
        self.get_logger().info(f"🚀 Moving to pickup pose: {pose}")

        move_goal = WaypointMove.Goal()
        move_goal.target_pose.position.x = pose[0]
        move_goal.target_pose.position.y = pose[1]
        move_goal.target_pose.position.z = pose[2]
        move_goal.target_pose.rotation = pose[3]

        self.move_client.wait_for_server()
        move_future = self.move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, move_future)
        goal_handle = move_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Move goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(
            f"✅ Reached: x={result.final_pose.position.x:.4f}, y={result.final_pose.position.y:.4f}, z={result.final_pose.position.z:.4f}"
        )

        # Now trigger gripper
        self.get_logger().info("🗜️ Closing gripper to 33mm with 60N force")
        grip_goal = GripperSetWidth.Goal()
        grip_goal.target_width = 33.0
        grip_goal.target_force = 60.0

        self.gripper_client.wait_for_server()
        grip_future = self.gripper_client.send_goal_async(grip_goal)
        rclpy.spin_until_future_complete(self, grip_future)

        grip_handle = grip_future.result()
        if not grip_handle.accepted:
            self.get_logger().error("❌ Gripper goal rejected.")
            return False

        grip_result_future = grip_handle.get_result_async()
        rclpy.spin_until_future_complete(self, grip_result_future)
        grip_result = grip_result_future.result().result
        self.get_logger().info(f"✅ Gripper closed at {grip_result.final_width:.2f} mm")

        return True

def main(args=None):
    rclpy.init(args=args)
    node = PickActionNode()
    node.get_logger().info("📦 pick_action_node ready. Use .pick_at(pose) from another script.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
