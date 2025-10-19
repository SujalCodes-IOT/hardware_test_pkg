import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

SERVO_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz
pwm.start(0)

class ServoPublisher(Node):
    def __init__(self):
        super().__init__('servo_publisher')
        self.publisher_ = self.create_publisher(Int32, 'servo_angle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.angle = 0
        self.direction = 1

    def timer_callback(self):
        # Sweep servo
        self.angle += 10 * self.direction
        if self.angle >= 180:
            self.angle = 180
            self.direction = -1
        elif self.angle <= 0:
            self.angle = 0
            self.direction = 1

        msg = Int32()
        msg.data = self.angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing angle: {self.angle}')

        # Set servo PWM
        duty = 2 + (self.angle / 18)
        pwm.ChangeDutyCycle(duty)

def main(args=None):
    rclpy.init(args=args)
    node = ServoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
