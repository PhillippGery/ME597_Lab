import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')
        
        # raw image topic
        self.subscription = self.create_subscription( Image, '/camera/image_raw', self.listener_callback, 10)
        
        # Publish bounding box data
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            
        self.bridge = CvBridge()

        self.get_logger().info("Red Ball follower  started.")

    def listener_callback(self, msg):

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
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
            
            #  box coordinates
            x, y, w, h = cv2.boundingRect(largest_contour)

            if w < 10:
                self.get_logger().info("Object too small, ignoring --> Noicy misstake.")
                return
            else:
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

                # Control logic
                distance_to_object = 

                error_x = centroid_x - image_center_x
                error_z = 
        
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