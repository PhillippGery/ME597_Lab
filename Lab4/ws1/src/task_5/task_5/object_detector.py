import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import BoundingBox2D, Pose2D

# Import the YOLO model from the ultralytics library
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.subscription = self.create_subscription(
            Image,
            '/video_data',
            self.listener_callback,
            10)
        
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
        self.bridge = CvBridge()

        # --- YOLO MODEL INITIALIZATION ---
        # Load a pre-trained YOLOv8n model. 
        # The model is downloaded automatically the first time you run this.
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLOv8n model loaded successfully.")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # --- OBJECT DETECTION WITH YOLO ---
        # Run inference on the frame
        results = self.model(cv_image)

        # The 'results' object contains all detections. We loop through them.
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get bounding box coordinates in (x, y, width, height) format
                x, y, w, h = box.xywh[0].tolist()
                
                # Get the class ID and confidence score
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls]
                
                # --- For this lab, let's track a specific object, e.g., 'sports ball' ---
                # You can change 'sports ball' to 'person', 'cup', 'bottle', etc.
                # Check the COCO dataset for all 80 classes YOLOv8 is trained on.
                if class_name == 'sports ball' and conf > 0.5:
                    centroid_x = x
                    centroid_y = y
                    
                    self.get_logger().info(
                        f"Detected '{class_name}' with confidence {conf:.2f} "
                        f"at (x={centroid_x:.0f}, y={centroid_y:.0f}), "
                        f"Size: (w={w:.0f}, h={h:.0f})"
                    )

                    # Create and publish the BoundingBox2D message
                    bbox_msg = BoundingBox2D()
                    bbox_msg.center.position.x = centroid_x
                    bbox_msg.center.position.y = centroid_y
                    bbox_msg.size_x = w
                    bbox_msg.size_y = h
                    self.bbox_publisher.publish(bbox_msg)
                    
                    # Draw the bounding box and label on the image for visualization
                    x1, y1, x2, y2 = box.xyxy[0].tolist() # Get corner points
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'{class_name} {conf:.2f}', (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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