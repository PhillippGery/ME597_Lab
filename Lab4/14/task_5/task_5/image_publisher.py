import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from ament_index_python.packages import get_package_share_directory

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        
        # (20 FPS) for start dont know the actual frames
        timer_period = 1/25.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        pkg_path = get_package_share_directory('task_5')
        video_path = os.path.join(pkg_path, 'resource', 'lab3_video.avi')
        
        # Open the video file
        self.cap = cv2.VideoCapture(video_path)
        #DEBUG
        if not self.cap.isOpened():
            self.get_logger().error(f"Error opening video file: {video_path}")
            rclpy.shutdown()
            
        # CvBridge 
        self.bridge = CvBridge()

    def timer_callback(self):

        ret, frame = self.cap.read()
        
        if ret:
            # convert  to a ROS
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            self.publisher_.publish(img_msg)
        else:
            # If the video ends, log it and stop the node
            self.get_logger().info('End of video file.')
            self.cap.release()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()