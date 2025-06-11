import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from par_interfaces.action import WaypointMove

class GoHomeNode(Node):
    def __init__(self):
        super().__init__('go_home_node')

        self.move_client = ActionClient(self, WaypointMove, '/par_moveit/waypoint_move')

        # Call once on startup
        self.timer = self.create_timer(1.0, self.send_home_once)
        self.goal_sent = False

    def send_home_once(self):
        if self.goal_sent:
            return
        self.goal_sent = True

        self.get_logger().info('🏠 Moving to updated home position...')

        goal = WaypointMove.Goal()
        goal.target_pose.position.x = -0.10081600147395449
        goal.target_pose.position.y = 0.325684615585017
        goal.target_pose.position.z = 0.19506721306238672
        goal.target_pose.rotation = 0.0  # assuming your interface handles this for orientation

        if hasattr(goal, 'velocity_scaling'):
            goal.velocity_scaling = 0.2  # Optional speed scaling
            self.get_logger().info('🐢 Velocity scaling set to 20%')

        self.move_client.wait_for_server()
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server')
            return

        self.get_logger().info('✅ Home goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'✅ Home reached at: x={result.final_pose.position.x:.3f}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoHomeNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()