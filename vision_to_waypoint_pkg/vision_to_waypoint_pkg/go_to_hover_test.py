import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from par_interfaces.action import WaypointMove
from par_interfaces.msg import WaypointPose

class HoverTestNode(Node):
    def __init__(self):
        super().__init__('hover_test_node')

        self.move_client = ActionClient(self, WaypointMove, '/par_moveit/waypoint_move')
        self.go_to_hover_pose()

    def go_to_hover_pose(self):
        self.get_logger().info('🚀 Moving to hover pose...')

        goal = WaypointMove.Goal()
        wp = WaypointPose()
        wp.position.x = -0.286
        wp.position.y = 0.283
        wp.position.z = 0.203
        wp.rotation = 0.0  # ✅ Only this — no orientation field!

        goal.target_pose = wp

        self.move_client.wait_for_server()
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Hover move goal rejected')
            return

        self.get_logger().info('✅ Hover move goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f'🎯 Reached hover: x={result.final_pose.position.x:.3f}, '
            f'y={result.final_pose.position.y:.3f}, z={result.final_pose.position.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = HoverTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
