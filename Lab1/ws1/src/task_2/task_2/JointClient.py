import sys
import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointStateClient(Node):

    def __init__(self):
        super().__init__('joint_state_client')
        self.client = self.create_client(JointState, 'joint_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = JointState.Request()

    def send_request(self, x, y, z):
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.z = float(z)
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage: ros2 run task_2 client <x> <y> <z>")
        return

    client = JointStateClient()
    response = client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
    
    if response:
        client.get_logger().info(
            f'Request sent: x={sys.argv[1]}, y={sys.argv[2]}, z={sys.argv[3]}\n'
            f'Response received: valid={response.valid}')
    else:
        client.get_logger().error('Exception while calling service')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()