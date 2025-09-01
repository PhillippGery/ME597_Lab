
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Phillipp_subscriber')
        self.subscription = self.create_subscription(
            Float64, 'my_first_topic', self.listener_callback, 10)
        self.get_logger().info('Phillipp_subscriber node has been started.')


    def listener_callback(self, msg):
        original_value = msg.data
        doubled_value = 2 * original_value
        self.get_logger().info(f'Orgninal time recived: {original_value:.2f},sek --> Doubled: {doubled_value:.2f}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
