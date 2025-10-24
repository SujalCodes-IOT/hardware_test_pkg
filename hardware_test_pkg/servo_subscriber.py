import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ServoControl(Node):
    def __init__(self):
        super().__init__('servo_control')
        self.publisher_ = self.create_publisher(Int32, 'servo_angle', 10)
        self.get_logger().info("Use keyboard input to control the servo (0–180).")

        while rclpy.ok():
            try:
                angle = int(input("Enter servo angle (0-180): "))
                msg = Int32()
                msg.data = angle
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published angle: {angle}°")
            except ValueError:
                print("Please enter a valid integer.")
            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
