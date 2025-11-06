import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import both BoundingBox2D and BoundingBox2DArray
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray, Pose2D

from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.subscription = self.create_subscription(
            Image,
            '/video_data',
            self.listener_callback,
            10)
        
        # --- CHANGE 1: Update the publisher to use BoundingBox2DArray ---
        self.bbox_publisher = self.create_publisher(BoundingBox2DArray, '/bounding_boxes', 10) # New topic name
            
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLOv8n model loaded successfully.")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)

        # --- CHANGE 2: Create a BoundingBox2DArray message to hold all detections ---
        bbox_array_msg = BoundingBox2DArray()
        # It's good practice to set the header
        bbox_array_msg.header.stamp = self.get_clock().now().to_msg()
        bbox_array_msg.header.frame_id = "camera_frame" # Or the appropriate frame

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get the class ID and confidence score
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls]
                
                
                if conf > 0.2:
                    # Get bounding box coordinates in (x_center, y_center, width, height) format
                    x_center, y_center, w, h = box.xywh[0].tolist()
                    
                    # Create a single BoundingBox2D message for this object
                    bbox_msg = BoundingBox2D()
                    bbox_msg.center.position.x = x_center
                    bbox_msg.center.position.y = y_center
                    bbox_msg.size_x = w
                    bbox_msg.size_y = h
                    
                    # --- CHANGE 4: Append this single box to the array message ---
                    bbox_array_msg.boxes.append(bbox_msg)
                    
                    # Draw the bounding box and label on the image for visualization
                    x1, y1, x2, y2 = box.xyxy[0].tolist() # Get corner points
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'{class_name} {conf:.2f}', (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- CHANGE 5: Publish the entire array of boxes once, after the loop ---
        if bbox_array_msg.boxes: # Only publish if at least one object was detected
            self.bbox_publisher.publish(bbox_array_msg)

        # Display the processed image
        cv2.imshow("YOLO Object Detector", cv_image)
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