import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')

        # Subscriber to /cmd_vel (Twist)
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        # Publisher to /tricycle_controller/cmd_vel (TwistStamped)
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            '/tricycle_controller/cmd_vel',
            10
        )

    def twist_callback(self, msg):
        """ Convert Twist to TwistStamped """
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg  # Copy twist values

        # Publish the transformed message
        self.twist_stamped_pub.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
