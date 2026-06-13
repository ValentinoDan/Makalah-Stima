import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class OdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')

        self.declare_parameter('publish_rate', 20.0) # Hz
        self.declare_parameter('initial_x',   0.0) # m
        self.declare_parameter('initial_y',   0.0) # m
        self.declare_parameter('initial_yaw', 0.0) # rad

        rate = self.get_parameter('publish_rate').value
        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value
        self.dt = 1.0 / rate
        self.vx = 0.0
        self.wz = 0.0

        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._initialpose_callback, 10)

        self.timer = self.create_timer(self.dt, self._update)
        self.get_logger().info(
            f'OdomNode started | rate={rate:.1f} Hz | '
            f'init=({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.3f} rad)'
        )
        self.get_logger().info(
            'Collision detection active — robot stops at obstacle boundaries.'
        )


    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = list(msg.data)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().info(
            f'Map received: {self.map_width}x{self.map_height} '
            f'@ {self.map_resolution:.3f} m/px — collision detection ready.'
        )

    def _cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def _initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """Reset posisi robot saat klik '2D Pose Estimate'"""
        self.x  = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        self.vx = 0.0
        self.wz = 0.0

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(
            f'Pose reset: ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.3f} rad)'
        )

    ROBOT_RADIUS = 0.20

    def _check_cell(self, wx: float, wy: float) -> bool:
        gx = int((wx - self.map_origin_x) / self.map_resolution)
        gy = int((wy - self.map_origin_y) / self.map_resolution)
        if gx < 0 or gx >= self.map_width or gy < 0 or gy >= self.map_height:
            return True
        return self.map_data[gy * self.map_width + gx] > 50

    def _is_obstacle(self, wx: float, wy: float) -> bool:
        if self.map_data is None:
            return False

        # Titik tengah
        if self._check_cell(wx, wy):
            return True

        # 16 titik di sekeliling radius robot
        r = self.ROBOT_RADIUS
        for i in range(16):
            angle = i * (2 * math.pi / 16)
            px = wx + r * math.cos(angle)
            py = wy + r * math.sin(angle)
            if self._check_cell(px, py):
                return True

        return False

    def _update(self) -> None:
        now = self.get_clock().now()

        delta_yaw = self.wz * self.dt
        mid_yaw = self.yaw + delta_yaw / 2.0
        new_x = self.x + self.vx * math.cos(mid_yaw) * self.dt
        new_y = self.y + self.vx * math.sin(mid_yaw) * self.dt
        new_yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))

        if self._is_obstacle(new_x, new_y):
            # stop translasi, tapi ttp bisa rotasi
            self.vx = 0.0
            new_x = self.x
            new_y = self.y
        
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw

        q = euler_to_quaternion(self.yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()