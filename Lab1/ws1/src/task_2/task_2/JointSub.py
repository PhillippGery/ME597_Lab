import rclpy
from rclpy.node import Node
from task_2_interfaces.msg import JointData

class JointDataSubscriber(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            JointData,
            'joint_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: center=({msg.center.x}, {msg.center.y}, {msg.center.z}), vel={msg.vel}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = JointDataSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()