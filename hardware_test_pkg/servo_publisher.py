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
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.angle = 0
        self.direction = 1

    def timer_callback(self):
        # Sweep servo back and forth
        self.angle += 10 * self.direction
        if self.angle >= 180:
            self.angle = 180
            self.direction = -1
        elif self.angle <= 0:
            self.angle = 0
            self.direction = 1

        # Publish ROS 2 message
        msg = Int32()
        msg.data = self.angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing angle: {self.angle}')

        # Software PWM for servo using gpiod
        period = 0.02  # 20 ms period (50 Hz)
        high_time = 0.001 + (self.angle / 180.0) * 0.002  # 1–3 ms pulse
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
        # Release GPIO line and shutdown
        line.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
