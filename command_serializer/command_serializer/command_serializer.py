import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CommandSerializer(Node):
    def __init__(self):
        super().__init__('command_serializer')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Configure the serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f"Received cmd_vel - Linear: {linear_x}, Angular: {angular_z}")

        # Format and send data over serial
        if self.serial_port:
            command = f"{linear_x},{angular_z}\n"
            try:
                self.serial_port.write(command.encode())
                self.get_logger().info(f"Sent to serial: {command.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandSerializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if node.serial_port:
            node.serial_port.close()

if __name__ == '__main__':
    main()
