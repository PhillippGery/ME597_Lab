#!/usr/bin/env python3

import math
import sys
import os
import numpy as np

import rclpy
from rclpy.node import Node as RosNode
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from .AStar import Queue, Node, Tree, AStar, MapProcessor, Map


class Navigation(RosNode):
    """! Navigation node class.
    This class should serve as a template to implement the path planning and
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        super().__init__(node_name)

        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.start_time = 0.0
        self.get_logger().info("Graph built successfully.")

        pkg_share_path = get_package_share_directory('task_4')
        map_yaml_path = os.path.join(pkg_share_path, 'maps', 'classroom_map.yaml')
        
        inflation_kernel_size = 5 
        
        self.kp_angular = 1.5
        self.kp_linear = 0.3
        self.lookahead_dist = 0.5
        self.speed_max = 0.15
        self.goal_tolerance = 0.2
        self.align_threshold = 0.4

        self.get_logger().info(f"Loading map from '{map_yaml_path}' and building graph...")
        self.map_processor = MapProcessor(map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Graph built successfully.")

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10)

        self.rate = 10.0
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        self.get_logger().info(
            'goal_pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped
        self.get_logger().info(
            'ttbot_pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))

    def a_star_path_planner(self, start_pose, end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        path = Path()
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9 #Do not edit this line (required for autograder)


        start_world = (start_pose.pose.position.x, start_pose.pose.position.y)
        end_world = (end_pose.pose.position.x, end_pose.pose.position.y)
        
        start_grid = self._world_to_grid(self.map_processor.map, start_world)
        end_grid = self._world_to_grid(self.map_processor.map, end_world)

        start_name = f"{start_grid[0]},{start_grid[1]}"
        end_name = f"{end_grid[0]},{end_grid[1]}"
        
        # Validate that start/end are valid nodes in our graph
        if start_name not in self.map_processor.map_graph.g or end_name not in self.map_processor.map_graph.g:
            self.get_logger().error("Start or end pose is inside an obstacle or out of bounds.")
            # Publish empty path and return
            self.astarTime = Float32()
            self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
            self.calc_time_pub.publish(self.astarTime)
            return Path()
            
        start_node = self.map_processor.map_graph.g[start_name]
        end_node = self.map_processor.map_graph.g[end_name]
        
        # 2. Run the A* solver
        astar_solver = AStar(self.map_processor.map_graph)
        path_names, path_dist = astar_solver.solve(start_node, end_node)
        
        # 3. Convert the grid path back to a world path (Path message)
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        if path_names:
            self.get_logger().info(f"A* found a path of length {len(path_names)}")
            for name in path_names:
                grid_coords = tuple(map(int, name.split(',')))
                world_coords = self._grid_to_world(self.map_processor.map, grid_coords)
                
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = world_coords[0]
                pose.pose.position.y = world_coords[1]
                pose.pose.orientation.w = 1.0 
                path.poses.append(pose)
        else:
            self.get_logger().warn("A* failed to find a path.")
            # The template's dummy path is removed, as we return a proper path here.


        # Do not edit below (required for autograder)
        self.astarTime = Float32()
        self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
        self.calc_time_pub.publish(self.astarTime)
        
        return path

    def get_path_idx(self, path, vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position in the path pointing to the next goal pose to follow.
        """
        min_dist = float('inf')
        closest_idx = 0
        for i, pose in enumerate(path.poses):
            dx = pose.pose.position.x - vehicle_pose.pose.position.x
            dy = pose.pose.position.y - vehicle_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Start searching from the closest point and find the first point
        # that is beyond the lookahead distance
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(path.poses)):
            dx = path.poses[i].pose.position.x - vehicle_pose.pose.position.x
            dy = path.poses[i].pose.position.y - vehicle_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist > self.lookahead_dist:
                lookahead_idx = i
                return lookahead_idx
        
        # If no point is far enough, target the last point
        return len(path.poses) - 1


    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        # Calculate distance and angle to the target waypoint
        k_rho = 0.3
        k_alpha = 0.8
        k_beta = -0.3 # Must be negative for stability

        # 1. Get robot's current state (x_R, y_R, theta_R)
        robot_x = vehicle_pose.pose.position.x
        robot_y = vehicle_pose.pose.position.y
        quat = vehicle_pose.pose.orientation
        
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_theta = math.atan2(siny_cosp, cosy_cosp)

        # 2. Get goal position (x_G, y_G)
        goal_x = current_goal_pose.pose.position.x
        goal_y = current_goal_pose.pose.position.y

        # 3. Calculate Cartesian error
        delta_x = goal_x - robot_x
        delta_y = goal_y - robot_y

        # 4. Transform error to Polar Coordinates (rho, alpha, beta)
        rho = math.sqrt(delta_x**2 + delta_y**2)
        alpha = -robot_theta + math.atan2(delta_y, delta_x)
        
        # Normalize alpha to be between -pi and pi
        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi
        
        beta = -robot_theta - alpha

        # If we are very close to the goal, stop the robot to prevent overshoot.
        if rho < 0.05: # 5 cm tolerance
            return 0.0, 0.0

        # 5. Apply the Control Law 
        # v = k_rho * rho
        # omega = k_alpha * alpha + k_beta * beta
        
        # We cap the linear speed to the max speed defined in __init__
        speed = min(k_rho * rho, self.speed_max) 
        heading = k_alpha * alpha + k_beta * beta
            
        return speed, heading

    def move_ttbot(self, speed, heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def run_loop(self):
        """! Main loop of the node, called by a timer. """
        # Wait until we have a robot pose and a path to follow
        if self.ttbot_pose is None or not self.path.poses or self.goal_pose is None:
            return
            
        # Check if we have reached the final goal
        final_goal_pose = self.path.poses[-1]
        dx = final_goal_pose.pose.position.x - self.ttbot_pose.pose.position.x
        dy = final_goal_pose.pose.position.y - self.ttbot_pose.pose.position.y
        dist_to_final_goal = math.sqrt(dx**2 + dy**2)
        
        if dist_to_final_goal < self.goal_tolerance:
            self.get_logger().info("Goal reached!")
            self.move_ttbot(0.0, 0.0) # Stop the robot
            self.path = Path()       # Clear the path
            self.goal_pose = None      # Clear the goal
            return
        
        # Find the next waypoint on the path to follow
        idx = self.get_path_idx(self.path, self.ttbot_pose)
        current_goal = self.path.poses[idx]
        
        # Calculate and publish robot commands
        speed, heading = self.path_follower(self.ttbot_pose, current_goal)
        self.move_ttbot(speed, heading)

    def _world_to_grid(self, map_info, world_pos):
        """ Converts world coordinates (meters) to grid cell coordinates. """
        origin_x = map_info.origin[0]
        origin_y = map_info.origin[1]
        resolution = map_info.resolution
        grid_j = int((world_pos[0] - origin_x) / resolution)
        grid_i = int((world_pos[1] - origin_y) / resolution)
        return (grid_i, grid_j)

    def _grid_to_world(self, map_info, grid_pos):
        """ Converts grid cell coordinates back to world coordinates (meters). """
        origin_x = map_info.origin[0]
        origin_y = map_info.origin[1]
        resolution = map_info.resolution
        world_x = (grid_pos[1] * resolution) + origin_x + resolution / 2.0
        world_y = (grid_pos[0] * resolution) + origin_y + resolution / 2.0
        return (world_x, world_y)


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')

    try:
        rclpy.spin(nav) 
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
