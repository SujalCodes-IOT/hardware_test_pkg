import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import time
import threading

SERVO_PIN = 18  # BCM pin for servo
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
        self.subscription
        self.target_angle = 90  # default starting angle
        self.running = True
        self.get_logger().info("Servo subscriber ready (real-time control).")

        # Start PWM loop in separate thread
        pwm_thread = threading.Thread(target=self.pwm_loop)
        pwm_thread.daemon = True
        pwm_thread.start()

    def listener_callback(self, msg):
        self.target_angle = msg.data  # update instantly
        self.get_logger().info(f"Updated target angle: {self.target_angle}°")

    def pwm_loop(self):
        period = 0.02  # 20ms period (50Hz)
        while self.running:
            angle = self.target_angle
            high_time = 0.001 + (angle / 180.0) * 0.001  # 1–2ms pulse
            line.set_value(1)
            time.sleep(high_time)
            line.set_value(0)
            time.sleep(period - high_time)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


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
