import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateNode(Node):
    # Publish joint states for the robots wheel joints

    JOINT_NAMES = ['left_wheel_joint', 'right_wheel_joint']

    def __init__(self):
        super().__init__('joint_state_node')
        self.declare_parameter('publish_rate', 30.0)
        rate = self.get_parameter('publish_rate').value
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'JointStateNode started – publishing {self.JOINT_NAMES} at {rate:.0f} Hz')

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = [0.0] * len(self.JOINT_NAMES)
        msg.velocity = [0.0] * len(self.JOINT_NAMES)
        msg.effort = [0.0] * len(self.JOINT_NAMES)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()