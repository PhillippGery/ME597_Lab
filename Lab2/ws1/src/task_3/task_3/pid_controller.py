
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
import math 
import time 

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        #PID vales
        self.Kp = 1.0  
        self.Ki = 0.0
        self.Kd = 0. 

        # Target
        self.target_distance = 0.35 

        #Init vales
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.previous_time = time.time()
        self.latest_scan = None

        # Vel lims
        self.MAX_LINEAR_VEL = 0.15
        self.MIN_LINEAR_VEL = -0.15
   
        # Sub /scan
        self.scan_subscriber = self.create_subscription( LaserScan, '/scan', self.scan_callback, 10) 
            
        # Publ /cmd_vel top
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
    
        # Timer loop at 10Hz
        timer_period = 0.1 # seconds (1 / 10Hz)
        self.timer = self.create_timer(timer_period, self.controller_loop)

    def scan_callback(self, msg):
        self.latest_scan = msg


    def controller_loop(self):

        if self.latest_scan is None:
            # debug if no msg comes in 
            self.get_logger().info('Waiting for laser scan data...')
            return

        forward_distance = self.latest_scan.ranges[0]
        left_distance = self.latest_scan.ranges[90]
        back_distance = self.latest_scan.ranges[180]
        rigth_distance = self.latest_scan.ranges[270]

        # # Debug if dis ==(inf or nan)
        # if math.isinf(forward_distance) or math.isnan(forward_distance):
        #     self.get_logger().warn('Invalid laser data, stopping the robot.')
        #     twist_msg.linear.x = 0.0 # Command the robot to stop
        #     self.velocity_publisher.publish(twist_msg)
        #     return

        # PID Calc
        current_time = time.time()
        dt = current_time - self.previous_time

        error = self.target_distance - forward_distance

        p_term = self.Kp * error

        self.integral_error += error * dt
        # # Anti-windup with constant
        # self.integral_error = max(min(self.integral_error, 1.0), -1.0) 
        i_term = self.Ki * self.integral_error

        if dt > 0:
            derivative_error = (error - self.previous_error) / dt
        else:
            derivative_error = 0.0
        d_term = self.Kd * derivative_error

        pid_output = p_term + i_term + d_term

        #save stae
        self.previous_error = error
        self.previous_time = current_time

        #whatch boundurys
        linear_velocity = max(self.MIN_LINEAR_VEL, min(-pid_output, self.MAX_LINEAR_VEL))

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = 0.0

        self.velocity_publisher.publish(twist_msg)

        # Log the values for tuning and debugging
        self.get_logger().info(
            f'Dist: {forward_distance:.2f}m | '
            f'Error: {error:.2f} | '
            f'Cmd_Vel: {linear_velocity:.2f} m/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()