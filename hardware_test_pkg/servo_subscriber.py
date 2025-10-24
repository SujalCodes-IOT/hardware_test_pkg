import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import time


SERVO_PIN = 18  # BCM numbering
chip = gpiod.Chip('gpiochip0')
line = chip.get_line(SERVO_PIN)
line.request(consumer="servo_subscriber", type=gpiod.LINE_REQ_DIR_OUT)


class ServoSubscriber(Node):
    def __init__(self):
        super().__init__('servo_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Servo subscriber started — waiting for angle commands...")

    def listener_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f"Received angle: {angle}°")
        self.move_servo(angle)

    def move_servo(self, angle):
        period = 0.02  # 20 ms period (50 Hz)
        high_time = 0.001 + (angle / 180.0) * 0.001  # 1–2 ms pulse width
        for _ in range(50):  # Run ~1 s of 50 Hz PWM
            line.set_value(1)
            time.sleep(high_time)
            line.set_value(0)
            time.sleep(period - high_time)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSubscriber()
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
