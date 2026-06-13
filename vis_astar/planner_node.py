import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from vis_astar.clearance_astar import ClearanceAStar
from vis_astar.utils import clear_map, to_grid

class PlannerNode(Node):
    """
    Subscribes:
      /map (nav_msgs/OccupancyGrid)                  
      /initialpose (geometry_msgs/PoseWithCovarianceStamped)  
      /goal_pose (geometry_msgs/PoseStamped)               
    Publishes:
      /vis_path (nav_msgs/Path)
    """

    def __init__(self):
        super().__init__("planner_node")
        self.planner = ClearanceAStar(lambda_score=0.5)

        # map state
        self.grid = None
        self.clearance = None
        self.map_info = None

        self.start_world = None
        self.goal_world = None

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, map_qos)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initialpose_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, "/vis_path", 10)
        self.get_logger().info("PlannerNode started — set /initialpose and /goal_pose in RViz")

    def map_cb(self, msg: OccupancyGrid):
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height} "
            f"@ {msg.info.resolution:.3f} m/px"
        )

        self.grid = to_grid(msg)
        self.clearance = clear_map(self.grid)
        self.map_info = msg.info
        self._try_plan()

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        """Receive start pose from RViz 2D Pose Estimate"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.start_world = (x, y)
        self.get_logger().info(f"Start set: world ({x:.2f}, {y:.2f})")
        self._try_plan()

    def goal_cb(self, msg: PoseStamped):
        """Receive goal pose from RViz Nav2 Goal or 2D Goal Pose"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal_world = (x, y)
        self.get_logger().info(f"Goal set: world ({x:.2f}, {y:.2f})")
        self._try_plan()

    def _try_plan(self):
        if self.grid is None:
            self.get_logger().warn("No map yet — waiting for /map")
            return
        if self.start_world is None:
            self.get_logger().warn("No start — use RViz '2D Pose Estimate'")
            return
        if self.goal_world is None:
            self.get_logger().warn("No goal — use RViz '2D Goal Pose' or Nav2 Goal")
            return

        start_grid = self._world_to_grid(self.start_world)
        goal_grid = self._world_to_grid(self.goal_world)

        if not self._in_bounds(start_grid):
            self.get_logger().error(f"Start {start_grid} out of map bounds!")
            return
        if not self._in_bounds(goal_grid):
            self.get_logger().error(f"Goal {goal_grid} out of map bounds!")
            return

        self.get_logger().info(f"Planning: grid {start_grid} -> {goal_grid}")
        path = self.planner.solve(self.grid, self.clearance, start_grid, goal_grid)

        if not path:
            self.get_logger().warn("No path found!")
            return

        self.get_logger().info(f"Path found: {len(path)} waypoints")
        self._publish_path(path)

    def _world_to_grid(self, world_pos):
        """Convert world (m) -> grid (pixel) coordinates."""
        wx, wy = world_pos
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        res = self.map_info.resolution
        gx = int((wx - origin_x) / res)
        gy = int((wy - origin_y) / res)
        return (gx, gy)

    def _in_bounds(self, grid_pos):
        gx, gy = grid_pos
        h = len(self.grid)
        w = len(self.grid[0]) if h > 0 else 0
        return 0 <= gx < w and 0 <= gy < h

    def _publish_path(self, path):
        """Convert grid path -> ROS Path and publish."""
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        res = self.map_info.resolution

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = gx * res + origin_x
            pose.pose.position.y = gy * res + origin_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info("Path published to /vis_path")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()