import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')
        
        # raw image topic

        self.subscription = self.create_subscription( Image, '/camera/image_raw', self.listener_callback, 10)

        # Does not work because depth image is not avalivle in lab config
        # self.subscription = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        # self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw')
        # self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription], 10, 0.1)      
        # # synchronized callback with two msges
        # self.ts.registerCallback(self.listener_callback)
        
        # Publish bounding box data
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            
        self.bridge = CvBridge()

        #Init prms
        self.pid_p_angular = 0.4
        self.pid_i_angular = 0.1
        self.pid_d_angular = 0.4
        self.integral_angular = 0.0
        self.previous_error_angular = 0.0
        self.p_linear = 0.3

        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0

        self.last_time = None
        self.last_detection_time = self.get_clock().now()

        # define initial pose in origion
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_timer = self.create_timer(2.0, self.publish_initial_pose)
        

        self.get_logger().info("Red Ball follower  started.")

    def listener_callback(self, msg):

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return
        

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Avoid division by zero if dt is too small
        if dt == 0:
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([0, 130, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red = np.array([170, 130, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

        # Combine the two masks
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        twist_msg = Twist()
        potential_balls = []

        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                
                # Avoid division by zero
                if perimeter == 0:
                    continue
                    
                # check if circle bacause detacting red briks ....
                circularity = (4 * np.pi * area) / (perimeter * perimeter)
                
                circularity_threshold = 0.8
                min_area = 100 # pixels
                
                if circularity > circularity_threshold and area > min_area:
                    potential_balls.append(contour)
            
        if potential_balls:

            self.last_detection_time = self.get_clock().now()
            # Find the largest contour *among the circular ones*
            largest_contour = max(potential_balls, key=cv2.contourArea)
            

            #  box coordinates
            x, y, w, h = cv2.boundingRect(largest_contour)

            # center
            centroid_x = x + w // 2
            centroid_y = y + h // 2

            image_center_x = cv_image.shape[1] // 2

            #create bounding box as done in previous task_5
            self.get_logger().info(f"Object Centroid: (x={centroid_x}, y={centroid_y}), Size: (w={w}, h={h})")
            bbox_msg = BoundingBox2D()                
            bbox_msg.center.position.x = float(centroid_x)
            bbox_msg.center.position.y = float(centroid_y)
            bbox_msg.center.theta = 0.0
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)                
            self.bbox_publisher.publish(bbox_msg)                
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # error detection 
            #distance_to_object = depth_image[int(y), int(x)]
            #self.get_logger().info(f"Distance to Object: {distance_to_object} meters")
            distance_to_object = (100-w)/w
            self.get_logger().info(f"errro Distance: {distance_to_object}")
            error_x = centroid_x - image_center_x
            error_x = error_x / image_center_x  # Normalize error
            self.get_logger().info(f"Error X: {error_x}")


            #Controller
            self.integral_angular += error_x
            derivative_angular = error_x - self.previous_error_angular

            p_term = self.pid_p_angular * error_x
            i_term = (self.pid_i_angular * self.integral_angular)*dt
            d_term = (self.pid_d_angular * derivative_angular)/dt

            #windup for integral term
            i_term = np.clip(i_term, -0.5, 0.5)

            if abs(distance_to_object) > 0.1:
                linear_velocity = self.p_linear * distance_to_object
            else:
                linear_velocity = 0.0
            
            if abs(error_x) < 0.05:
                angular_velocity = 0.0
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0
                self.get_logger().info("Object centered.")
            else:
                angular_velocity = p_term + i_term + d_term
                self.previous_error_angular = error_x

            twist_msg.linear.x = np.clip(linear_velocity, -self.max_linear_speed, self.max_linear_speed)
            twist_msg.angular.z = np.clip(-angular_velocity, -self.max_angular_speed, self.max_angular_speed)
            
            self.publisher_.publish(twist_msg)

        else: 

            # When no object wait 3 sec then turn to search
            # when object in room and not covered by obstacle robot wil find it          
            time_since_last_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            self.get_logger().info(f"Time since last detection: {time_since_last_detection} seconds")   
            if time_since_last_detection > 3.0:
                self.get_logger().info("Searching for object...")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.3
                self.publisher_.publish(twist_msg)
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0

            else:
                self.get_logger().info("No object detected.")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher_.publish(twist_msg)
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0

                
        
        # Display vieo
        scale = 0.7

        # Calculate the new dimensions
        width = int(cv_image.shape[1] * scale)
        height = int(cv_image.shape[0] * scale)
        dim = (width, height)

        # Resize the image
        resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        # Display the RESIZED image
        cv2.imshow("Object Detector", resized_image)
        cv2.waitKey(1)




    def publish_initial_pose(self):
        """
        Publishes the initial pose to AMCL to set the robot's starting position
        on the map and then cancels the timer.
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set the position to the map's origin
        pose_msg.pose.pose.position.x = -5.4
        pose_msg.pose.pose.position.y = -6.18
        pose_msg.pose.pose.position.z = 0.0

        # Set the orientation (0 degrees yaw)
        pose_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Publishing initial pose to AMCL-topic")
        self.initial_pose_pub.publish(pose_msg)

        self.initial_pose_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    
    object_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()