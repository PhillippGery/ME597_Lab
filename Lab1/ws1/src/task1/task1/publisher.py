

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Phillipp_publisher')
        self.publisher_ = self.create_publisher(Float64, 'my_first_topic', 10)
        timer_period = 0.5  # sek == 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Phillipp_publisher node has been started.')

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
        msg = Float64()
        msg.data = elapsed_time
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Phillipp_publisher node is running for: "{msg.data:.2f}" seconds')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
