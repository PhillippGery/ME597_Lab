import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

# ros2 interface show vision_msgs/msg/BoundingBox2D
# # A 2D bounding box that can be rotated about its center.
# # All dimensions are in pixels, but represented using floating-point
# #   values to allow sub-pixel precision. If an exact pixel crop is required
# #   for a rotated bounding box, it can be calculated using Bresenham's line
# #   algorithm.

# # The 2D position (in pixels) and orientation of the bounding box center.
# vision_msgs/Pose2D center
#         vision_msgs/Point2D position
#                 float64 x
#                 float64 y
#         float64 theta

# # The total size (in pixels) of the bounding box surrounding the object relative
# #   to the pose of its center.
# float64 size_x
# float64 size_y

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # raw image topic
        self.subscription = self.create_subscription( Image, '/video_data', self.listener_callback, 10)
        
        # Publish bounding box data
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
            
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # --- Object Detection Logic ---
        # Convert BGR to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the range for a bright color (e.g., yellow) in HSV
        # These values might need tuning for your specific object color
        lower_bound = np.array([0, 35, 0])
        upper_bound = np.array([16, 155, 254])

        # # The most important lines for tuning!
        # lower_bound = np.array([H_min, S_min, V_min])
        # upper_bound = np.array([H_max, S_max, V_max])
        
        # Create a mask to isolate the color
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour if any are found
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the bounding box coordinates
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate the centroid
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            
            # --- Print and Publish Data ---
            self.get_logger().info(f"Object Centroid: (x={centroid_x}, y={centroid_y}), Size: (w={w}, h={h})")
            
            # Create a BoundingBox2D message
            bbox_msg = BoundingBox2D()
            
            bbox_msg.center.position.x = float(centroid_x)
            bbox_msg.center.position.y = float(centroid_y)
            bbox_msg.center.theta = 0.0

            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            
            # Publish the bounding box
            self.bbox_publisher.publish(bbox_msg)
            
            # Draw the bounding box on the original image for visualization
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Display the processed image in a window
        cv2.imshow("Object Detector", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    
    object_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()