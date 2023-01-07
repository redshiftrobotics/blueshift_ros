import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import board
import busio
import adafruit_pca9685

from blueshift_interfaces.msg import Motors


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Motors,
            'motor_speeds',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = 60

        self.motor1 = self.pca.channels[0]

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.top_front_left)

        self.motor1.duty_cycle = int((((msg.top_front_left)+1)/2)*0xffff)




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()