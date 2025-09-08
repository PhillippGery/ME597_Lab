import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState

class JointStateService(Node):

    def __init__(self):
        super().__init__('joint_state_service')
        self.srv = self.create_service(JointState, 'joint_service', self.joint_state_callback)

    def joint_state_callback(self, request, response):

        sum_of_coords = request.x + request.y + request.z
        
        if sum_of_coords >= 0:
            response.valid = True
            self.get_logger().info(f'Incoming request\nx: {request.x}, y: {request.y}, z: {request.z}\nSum is {sum_of_coords}, sending response: {response.valid}')
        else:
            response.valid = False
            self.get_logger().info(f'Incoming request\nx: {request.x}, y: {request.y}, z: {request.z}\nSum is {sum_of_coords}, sending response: {response.valid}')
            
        return response

def main(args=None):
    rclpy.init(args=args)
    service = JointStateService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()