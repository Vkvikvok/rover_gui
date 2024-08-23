import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        try:
            super().__init__('camera_publisher')
            self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
            self.bridge = CvBridge()
            self.cap = cv2.VideoCapture(0)  # Kamera kaynağı

            self.timer = self.create_timer(0.1, self.timer_callback)  # 10 fps
            self.get_logger().info("Camera Publisher Node has been started")
        except Exception as e:
            print("There is an error in publishing of camera data:{e}")
        

    def timer_callback(self):
        try: 
            ret, frame = self.cap.read()
            if ret:
                # OpenCV görüntüsünü ROS mesajına dönüştür
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(ros_image)
        except Exception as e:
            print(f"There is an error in timer of publishing camera data:{e}")
            
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
