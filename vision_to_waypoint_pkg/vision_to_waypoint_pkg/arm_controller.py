import rclpy
from rclpy.node import Node
from vision_to_waypoint_pkg.pick_action import PickActionNode
import json
from pathlib import Path

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self.picker = PickActionNode()
        self.get_logger().info("🧭 ArmController ready. Loading waypoints...")

        # Load waypoint file
        path = Path(__file__).parent / 'data' / 'waypoints.json'
        with open(path, 'r') as f:
            self.waypoints = json.load(f)

        # Example usage (pick from waypoint 'b')
        self.pick_from_waypoint('b')

    def pick_from_waypoint(self, waypoint_name):
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f"❌ Waypoint '{waypoint_name}' not found!")
            return

        pose = self.waypoints[waypoint_name]
        success = self.picker.pick_at(pose)
        if success:
            self.get_logger().info(f"✅ Successfully picked from {waypoint_name}")
        else:
            self.get_logger().error(f"❌ Failed to pick from {waypoint_name}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
