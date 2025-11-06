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

#color mask for red Object copyed by
#https://github.com/zainasir/BallFollower/blob/main/ball_follower/scripts/finder.py

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
        
        # # for tuning use hsv_tuner.py
        # lower_bound = np.array([H_min, S_min, V_min])
        # upper_bound = np.array([H_max, S_max, V_max])
        
        # mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red = np.array([170, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

        # Combine the two masks
        mask = cv2.bitwise_or(mask1, mask2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # only one contour -- largest
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
            

            
                #  box coordinates
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # center
                centroid_x = x + w // 2
                centroid_y = y + h // 2

                self.get_logger().info(f"Object Centroid: (x={centroid_x}, y={centroid_y}), Size: (w={w}, h={h})")
                
                bbox_msg = BoundingBox2D()
                
                bbox_msg.center.position.x = float(centroid_x)
                bbox_msg.center.position.y = float(centroid_y)
                bbox_msg.center.theta = 0.0

                bbox_msg.size_x = float(w)
                bbox_msg.size_y = float(h)
                
                self.bbox_publisher.publish(bbox_msg)
                
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Display vieo
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