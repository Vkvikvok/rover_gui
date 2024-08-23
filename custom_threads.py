from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Kamera bilgilerinin publisher dan alınıp arayüze aktarılmasını sağlayan iş parçacığı sınıfı
class CameraSubscriberThread(QThread):
    image_update = pyqtSignal(np.ndarray)

    def __init__(self, context):
        super().__init__()
        self.bridge = CvBridge() # Kamera verisinin dönüştürülmesini sağlayan köprü
        self._running = False
        self.context = context
        self.node = None

    # İş parçacığı çalıştırıldığında parçacığın yapması gerekenlerin tanımlandığı fonksiyon
    def run(self):
        try:
            self._running = True
            if self.node is None:
                self.node = Node('qt_ros_node', context=self.context)
                #self.node = rclpy.create_node('camera_subscriber_thread')
                self.subscription = self.node.create_subscription(
                    Image,
                    'camera/image_raw',
                    self.listener_callback,
                    10
                )

            # Subscriber düğümünün çalıştırılmasını sağlar
            while self._running:
                rclpy.spin_once(self.node)
                self.msleep(10)
                
        except Exception as e:
            print(f"There is an error in running thread:{e}")
            #self.get_logger().info(f"There is an error in running thread:{e}")

    # Görüntülerin işlenmesi ve arayüze çekilmesini sağlayan callback
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Burada istenilen görüntü işleme yapılabilir

            self.image_update.emit(rgb_image)
            
        except Exception as e:
            #self.get_logger().info(f"There is an error in camera listener:{e}")   
            print(f"There is an error in camera listener:{e}")          

    # Kamera arayüz içerisinde kapatıldığında düğümü kapatmak yerine beklemeye alır
    def stop(self):
        try:
            self._running = False # Burada callback içerisindeki while döngüsü durdurulur
            
        except Exception as e:
            #self.get_logger().info(f"Kamera işlemcisinin durdurulmasında sıkıntı çıktı:{e}")
            print(f"Kamera işlemcisinin durdurulmasında sıkıntı çıktı:{e}")
            

    # Arayüz kapatıldığında düğümün çalışmasını da kapatır
    def shutdown(self):
        if self.node:
            self.node.destroy_node()
            self.node = None
            rclpy.shutdown()
