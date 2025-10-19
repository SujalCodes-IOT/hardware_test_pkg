import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class HardwareTestNode(Node):
    def __init__(self):
        super().__init__('hardware_test_node')
        self.publisher_ = self.create_publisher(String, 'hardware_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.get_logger().info("Hardware test node started...")

    def publish_data(self):
        temperature = 43
        humidity = 75
        motor_speed = 175
        torque = 70

        msg = String()
        msg.data = (
            f"Temperature: {temperature} °C | "
            f"Humidity: {humidity}% | "
            f"Motor Speed: {motor_speed} rpm | "
            f"Torque: {torque} Nm"
        )
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = HardwareTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
