import rclpy
from rclpy.node import Node
from nav_msgs.msg import (OccupancyGrid, Path)
from geometry_msgs.msg import PoseStamped
from vis_astar.visibility_astar import VisAStar
from vis_astar.utils import clear_map, to_grid

class PlannerNode(Node):

    def __init__(self):
        super().__init__("planner_node")
        self.planner = VisAStar(lambda_score=0.3)
        self.map_received = False
        self.map_sub = (self.create_subscription(OccupancyGrid, "/map", self.callback, 10))
        self.path_pub = (self.create_publisher(Path, "/vis_path", 10))
        self.get_logger().info("Planner started")

    def callback(self, msg):
        if self.map_received:
            return
        
        self.map_received = True
        self.get_logger().info("Map received")

        grid = to_grid(msg)
        clearance = clear_map(grid)

        start = (5, 5)
        goal = (len(grid[0]) - 5, len(grid) - 5)
        self.get_logger().info(f"Planning from {start} to {goal}")

        path = self.planner.solve_vis(grid, clearance, start, goal)

        self.publish_path(path, msg)

    def publish_path(self, path, map_msg):
        if not path:
            self.get_logger().warn("No path found")
            return
        
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = (self.get_clock().now().to_msg())

        resolution = (map_msg.info.resolution)
        origin_x = (map_msg.info.origin.position.x)
        origin_y = (map_msg.info.origin.position.y)

        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = (x * resolution + origin_x)
            pose.pose.position.y = (y * resolution + origin_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info("Path published")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()