import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import time

SERVO_PIN = 18  # BCM pin number for servo

# Open GPIO chip
chip = gpiod.Chip('gpiochip0')
line = chip.get_line(SERVO_PIN)
line.request(consumer="servo_test", type=gpiod.LINE_REQ_DIR_OUT)

class ServoPublisher(Node):
    def __init__(self):
        super().__init__('servo_publisher')
        self.publisher_ = self.create_publisher(Int32, 'servo_angle', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.angles = [0, 90, 180, 90]  # Sequence
        self.index = 0

    def timer_callback(self):
        angle = self.angles[self.index]
        self.index = (self.index + 1) % len(self.angles)

        # Publish ROS 2 message
        msg = Int32()
        msg.data = angle
        self.publisher_.publish(msg)
        self.get_logger().info(f"Rotating servo to: {angle}°")

        # Generate PWM pulse for servo
        for _ in range(50):  # Run ~1 second of 50 Hz PWM
            self.set_servo_angle(angle)

    def set_servo_angle(self, angle):
        period = 0.02  # 20 ms period (50 Hz)
        high_time = 0.001 + (angle / 180.0) * 0.001  # 1–2 ms pulse width
        line.set_value(1)
        time.sleep(high_time)
        line.set_value(0)
        time.sleep(period - high_time)

def main(args=None):
    rclpy.init(args=args)
    node = ServoPublisher()
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
