import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt5.QtCore import Qt
from rover_window import RoverWindow
from rclpy.node import Node
import rclpy
from custom_element_files.custom_threads import CameraSubscriberThread

# Import other tabs similarly

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        try:
            self.setWindowTitle("Robot Projesi Arayüzü")

            # ROS2 bağlamını çek
            rclpy.init()
            self.context = rclpy.get_global_executor().context

            # Gerekli iş parçacıklarını tanımlıyoruz
            self.camera_thread = CameraSubscriberThread(self.context)


            # Tab Widget oluşturma
            self.tabs = QTabWidget()
            self.setCentralWidget(self.tabs)

            # Her bir sekme için widget oluşturma
            self.rover_tab = RoverWindow(self.context, self.camera_thread)
        
        

            # Sekmeleri tab widget'a eklemek
            self.tabs.addTab(self.rover_tab, "Rover")
        
            # Tanımlanmış iş parçacıklarının bağlantılarını kurup başlatıyoruz
            self.camera_thread.image_update.connect(self.rover_tab.update_image)  # Veri sinyali bağlama
            

        except Exception as e:
            print(f"There is an error in main gui:{e}")

    def closeEvent(self, event):
        try:
            if self.camera_thread.isRunning():
                self.camera_thread.stop()
            self.camera_thread.shutdown()
            rclpy.shutdown()
            event.accept()
        except Exception as e:
            print(f"There is an error in closing camera thread:{e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    print("Program has been started")
    sys.exit(app.exec_())