import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PathFollowerNode(Node):
    """
    Subscribe: /vis_path (nav_msgs/Path), /odom (nav_msgs/Odometry)
    Publish: /cmd_vel (geometry_msgs/Twist)
    """
    def __init__(self):
        super().__init__("path_follower_node")

        # param
        self.lookahead = 0.35 # m
        self.linear_speed = 0.15 # m/s
        self.goal_tol = 0.35 # m
        self.angle_tol = 0.1 # rad
        self.max_angular = 1.2 # rad/s

        # state
        self.path = []            
        self.wp_idx = 0           
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.active = False

        # Subscriber
        self.create_subscription(Path, "/vis_path", self.path_cb,  10)
        self.create_subscription(Odometry, "/odom", self.odom_cb,  10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("PathFollower started (Pure Pursuit)")

    def path_cb(self, msg: Path):
        """Receive new path from planner"""
        if not msg.poses:
            self.get_logger().warn("Received empty path")
            self.active = False
            return

        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.wp_idx = 0
        self.active = True
        self.get_logger().info(f"New path received: {len(self.path)} waypoints")

    def odom_cb(self, msg: Odometry):
        """Update robot pose from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.active or not self.path:
            return

        # check if goal reached
        gx, gy = self.path[-1]
        dist_goal = math.hypot(gx - self.robot_x, gy - self.robot_y)
        if dist_goal < self.goal_tol:
            self.get_logger().info("Goal reached! Stopping.")
            self.publish_stop()
            self.active = False
            return

        while self.wp_idx < len(self.path) - 1:
            wx, wy = self.path[self.wp_idx]
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist < self.lookahead:
                self.wp_idx += 1
            else:
                break

        target_x, target_y = self.path[self.wp_idx]

        # heading error
        angle_to_target = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
        heading_err = self._normalize_angle(angle_to_target - self.robot_yaw)
        dist_to_wp = math.hypot(target_x - self.robot_x, target_y - self.robot_y)

        if abs(heading_err) > 0.4: 
            linear = 0.0
        else:
            linear = self.linear_speed * (1.0 - 0.6 * abs(heading_err) / math.pi)
            linear = max(0.05, linear)

        angular = max(-self.max_angular, min(self.max_angular, 1.8 * heading_err))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()