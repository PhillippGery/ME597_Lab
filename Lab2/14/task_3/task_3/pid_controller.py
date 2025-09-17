
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
import math 
import time 
import matplotlib.pyplot as plt
from collections import deque

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_speed_controller')

        #PID vales
        self.Kp = 0.30#*0.6
        self.Ki = 0.01#.5*2
        self.Kd = 0.08#.12*2

        self.Kp = 1#*0.6
        self.Ki = 0.02#.5*2
        self.Kd = 0.1#.12*2

        # Target
        self.target_distance = 0.35

        #Init vales
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.previous_time = time.time()
        self.latest_scan = None
        self.start_time = time.time() # For plotting time axis

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

        #  Plot Initialization 
        self.plot_update_freq = 0.1 # Update plot every 0.1 seconds
        self.last_plot_time = time.time()
        
        # Set up plot
        plt.ion() # Turn on interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
        self.fig.suptitle('Live PID Controller Tuning')

        # Create deques to store data for plotting (fixed size)
        self.max_data_points = 100 
        self.time_data = deque(maxlen=self.max_data_points)
        self.desired_dist_data = deque(maxlen=self.max_data_points)
        self.actual_dist_data = deque(maxlen=self.max_data_points)
        self.velocity_data = deque(maxlen=self.max_data_points)
        self.pid_output_data = deque(maxlen=self.max_data_points)

        # Initialize empty plot lines
        self.line_desired, = self.ax1.plot([], [], 'r--', label='Desired Distance')
        self.line_actual, = self.ax1.plot([], [], 'b-', label='Actual Distance')
        self.line_pid_output, = self.ax2.plot([], [], 'm:', label='Unbounded Command Velocity') 
        self.line_velocity, = self.ax2.plot([], [], 'g-', label='Command Velocity')

        # Configure distance subplot (ax1)
        self.ax1.set_ylabel('Distance (m)')
        self.ax1.set_xlabel('Time (s)') 
        self.ax1.legend()
        self.ax1.grid(True, which='major', linestyle='-', linewidth='0.5', color='gray')
        self.ax1.grid(True, which='minor', linestyle=':', linewidth='0.5', color='lightgray')
        self.ax1.minorticks_on()

        # Configure velocity subplot (ax2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Velocity (m/s)')
        self.ax2.legend()
        self.ax2.set_ylim(self.MIN_LINEAR_VEL - 0.05, self.MAX_LINEAR_VEL + 0.05) # Fixed Y-limits for velocity
        self.ax2.grid(True, which='major', linestyle='-', linewidth='0.5', color='gray')
        self.ax2.grid(True, which='minor', linestyle=':', linewidth='0.5', color='lightgray')
        self.ax2.minorticks_on()


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
        i_term = self.Ki * self.integral_error
        # Anti-Windup

        max_iterm = 0.5#self.MAX_LINEAR_VEL
        min_iterm = -0.5#self.MIN_LINEAR_VEL
        if i_term > max_iterm:
            i_term = max_iterm
        elif i_term < min_iterm:
            i_term = min_iterm

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

        # Log results
        self.get_logger().info(
            f'Dist: {forward_distance:.2f}m | '
            f'Error: {error:.2f} | '
            f'Cmd_Vel: {linear_velocity:.2f} m/s'
        )
        #  Data Recording and Plot Update 
        elapsed_time = current_time - self.start_time

        # Append new data to the deques
        self.time_data.append(elapsed_time)
        self.desired_dist_data.append(self.target_distance)
        self.actual_dist_data.append(forward_distance)
        self.velocity_data.append(linear_velocity)
        self.pid_output_data.append(-pid_output)

        # Update plot data
        self.line_desired.set_data(self.time_data, self.desired_dist_data)
        self.line_actual.set_data(self.time_data, self.actual_dist_data)
        self.line_velocity.set_data(self.time_data, self.velocity_data)
        self.line_pid_output.set_data(self.time_data, self.pid_output_data)
        
        # Rescale axes
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        # Redraw the canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # A short pause to allow the plot to update
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # Keep the final plot visible until manually closed
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()