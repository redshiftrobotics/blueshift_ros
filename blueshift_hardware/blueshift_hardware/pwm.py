import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import board
import busio
import adafruit_pca9685

from blueshift_interfaces.msg import Motors

from rcl_interfaces.msg import ParameterDescriptor


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        motorIdsParamDescriptor = ParameterDescriptor(description="Motor IDs for PWM Gen, U=up D=down F=front B=back L=left R=right")
        self.declare_parameter('motorIds', [0,1,2,3,4,5,6,7,8], motorIdsParamDescriptor)

        motorLowestPWMParamDescriptor = ParameterDescriptor(description="The lowest PWM singal value for the motors")
        self.declare_parameter('motorLowestPWMSignal', 0, motorLowestPWMParamDescriptor)

        motorHighestPWMParamDescriptor = ParameterDescriptor(description="The highest PWM signal value for the motors")
        self.declare_parameter('motorHighestPWMSignal', 0xffff, motorHighestPWMParamDescriptor)

        PWMGenFrequencyParamDescriptor = ParameterDescriptor(description="The frequency for the PWM generator")
        self.declare_parameter('PWMGenFrequency', 60, PWMGenFrequencyParamDescriptor)


        self.subscription = self.create_subscription(
            Motors,
            'motor_speeds',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.PWMGen = adafruit_pca9685.PCA9685(self.i2c)
        self.PWMGen.frequency = self.get_parameter('PWMGenFrequency').get_parameter_value()

        self.motor1 = self.PWMGen.channels[self.get_parameter_value('motorIds').get_parameter_value()[0]]

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.top_front_left)

        motorMinPWMSignal = self.get_parameter('motorLowestPWMSignal').get_parameter_value()
        motorMaxPWMSignal = self.get_parameter('motorHighestPWMSignal').get_parameter_value()

        self.motor1.duty_cycle = int((((msg.top_front_left)+1)/2) * (motorMaxPWMSignal - motorMinPWMSignal) + motorMinPWMSignal)




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