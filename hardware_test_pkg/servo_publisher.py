import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import time

SERVO_PIN = 18  # BCM pin

chip = gpiod.Chip('gpiochip0')
line = chip.get_line(SERVO_PIN)
line.request(consumer="servo_driver", type=gpiod.LINE_REQ_DIR_OUT)

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        self.subscription = self.create_subscription(Int32, 'servo_angle', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Servo driver ready, waiting for commands...")

    def listener_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f"Received angle: {angle}°")
        # Run ~1 second of PWM to move servo to target
        for _ in range(50):
            self.set_servo_angle(angle)

    def set_servo_angle(self, angle):
        period = 0.02  # 20ms
        high_time = 0.001 + (angle / 180.0) * 0.001
        line.set_value(1)
        time.sleep(high_time)
        line.set_value(0)
        time.sleep(period - high_time)

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        line.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
