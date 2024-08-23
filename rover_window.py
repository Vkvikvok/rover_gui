import sys
from PyQt5.QtWidgets import QApplication, QWidget, QGraphicsScene, QGraphicsView, QLineEdit, QHeaderView, QTableView
from PyQt5.QtGui import QDoubleValidator, QKeySequence, QImage, QPixmap
from PyQt5.QtCore import Qt
# Özelleştirilmiş elemanların içeri aktarılması
from custom_element_files.custom_widgets import CustomGraphicsView, MyTableModel, CustomTableView
from rover_tab_ui import Ui_Rover  # output.py dosyasından oluşturulan sınıfı içe aktarın
import rclpy

class RoverWindow(QWidget, Ui_Rover):
    def __init__(self, context,camera_thread):
        super(RoverWindow, self).__init__()
        try:
            self.setupUi(self)  # Arayüzü ayarla

            # ROS2 bağlamını çekme
            self.context = context
            self.camera_thread = camera_thread

            # Yarışma alanının haritasını gösteren parça
            try:
                self.scene = QGraphicsScene()
                self.customCompetitionMap = CustomGraphicsView(self.scene)

                # Layout'taki eski QGraphicsView'i kaldır ve yeni CustomGraphicsView'i ekle
                layout = self.competitionMap.parentWidget().layout()
                layout.replaceWidget(self.competitionMap, self.customCompetitionMap)
                self.competitionMap.deleteLater()  # Orijinal QGraphicsView'i temizle

                # Görseli yükle
                self.customCompetitionMap.load_image('/home/volki/meturover_24/src/metu_gui/metu_gui/images/competition_map.jpg')
        
            except Exception as e:
                print(f"There is an error in loading competition map: {e}")



            # Roverın gideceği konumların listesinin ayarlanması
            try:
                self.model = MyTableModel()
                self.locationList = CustomTableView()
                self.locationList.setModel(self.model) # Özelleştirilmiş "QTableView" elemanını çağırıyoruz
                
                # Layout'taki eski QTableView elemanını kaldır ve yeni CustomTableView elemanını ekle
                layout = self.location_widgets_layout
                layout.replaceWidget(self.location_list, self.locationList)
                self.location_list.deleteLater()  # Orijinal QGraphicsView'i temizle

                # Tablo sütunlarının arayüze daha uygun şekilde boyutlanmasını sağlar
                self.locationList.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

                # Tablodan bir hücre seçildiği zaman tüm satırın seçilmesini sağlar
                self.locationList.setSelectionMode(QTableView.SingleSelection)
                self.locationList.setSelectionBehavior(QTableView.SelectRows)

                
           
            except Exception as e:
                print(f"There is an error in location list: {e}")
            

            # x ve y verilerini alacağımız kutuların ayarlanması
            validator = QDoubleValidator() # Sadece sayısal değerlerin girilmesini sağlar(e hariç)
            self.send_x.setValidator(validator)
            self.send_y.setValidator(validator)
            
            # "Send Location" butonunun düzenlenmesi
            self.send_location_button.setShortcut(QKeySequence("Enter")) # Enter kısayol olarak ayarlandı
            self.send_location_button.clicked.connect(self.sendLocation) # Gerekli fonksiyon atandı
            
            # "Go/Cancel" butonunun düzenlenmesi
            self.go_cancel_button.clicked.connect(self.toggle_row_highlight)
            self.highlight = False

            # "Camera On/Off" butonunun düzenlenmesi
            self.camera_on_off_button.clicked.connect(self.toggle_camera)
            self.camera_running = False

        except Exception as e:
            print(f"Error in MainWindow initialization: {e}")

    # "Go/Cancel" butonunun tetiklediği fonksiyon
    # İlk tıklandığında rover ilk satırdaki konuma gitmeye başlar ve bu sırada gittiği konum tabloda yeşil renk ile gözükür
    # İkinci kere tıklandığında ise bu eylem iptal edilir
    def toggle_row_highlight(self):
        if self.highlight:
            self.model.set_highlight_row(-1)
            self.go_cancel_button.setText("Go")
        else:
            self.model.set_highlight_row(0)  # 0, birinci satırı belirtir
            self.go_cancel_button.setText('Cancel')
        self.highlight = not self.highlight

    # Kameranın açılıp kapanmasını sağlayan butonun tetiklediği fonksiyon
    def toggle_camera(self):
        try:
            if self.camera_running:
                self.camera_thread.stop()
                self.camera_on_off_button.setText("Start Camera")

            else:
                if not self.camera_thread.isRunning():
                    self.camera_thread.start()
                self.camera_on_off_button.setText("Stop Camera")
            self.camera_running = not self.camera_running
        
        except Exception as e:
            print(f"There is an error at toggle camera:{e}")


    # Kameradan gelen verinin güncellenmesini ve boyutlandırılmasını sağlar
    def update_image(self, frame):
        h, w, ch = frame.shape
        qt_image = QImage(frame.data, w, h, w * ch, QImage.Format_RGB888) # CV verisini Qt'ye uygun bir formata dönüştürüyoruz
        # Görseli Pixmap formatına dönüştürüp arayüze göre ölçeklendirilmesini sağlıyoruz.
        camera_pixmap = QPixmap.fromImage(qt_image).scaled(self.camera_data_label.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.camera_data_label.setPixmap(camera_pixmap)   

    def sendLocation(self): 
        try:
            # x ve y verileri çekilip satır formatına getirildi
            send_x_data = float(self.send_x.text())
            send_y_data = float(self.send_y.text())
            
            new_row = [send_x_data, send_y_data]

            # Kutucukların boş olup olmadığı kontrol edilir ve verinin gönderildiği durumda kutucuklar temizlenir
            if send_x_data != None and send_y_data != None:
                self.model.addData(new_row)
                self.send_x.clear()
                self.send_y.clear()

            else:
                return None
        
        except Exception as e:
            print(f"There is an error in sending the location: {e}")

    # Haritadan bir nokta seçildiğinde koordinatlar gerekli kutucuklara otomatik olarak girilir
    def update_coordinates(self, x, y):
        try:
            self.send_x.setText(f"{x:.4f}")
            self.send_y.setText(f"{y:.4f}")
        
        except Exception as e:
            print(f"There is an error in update_coordinates:{e}")

    # "Delete" tuşuna basıldığında tabloda seçilmiş satır silinir
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete:
            self.delete_selected_rows()

    def delete_selected_rows(self):
        try:
            selection_model = self.locationList.selectionModel()
            selected_indexes = selection_model.selectedRows()
            row = selected_indexes[0].row()
            self.model.removeRow(row)
            self.locationList.selectionModel().clearSelection()
    
        except Exception as e:
            print(f"There is an error in delete_selected_rows: {e}")

            
    # Tablo dışında bir yere dokunulduğunda tablodaki seçim iptal edilir
    def mousePressEvent(self, event):
        if self.locationList.rect().contains(event.pos()):
            # Tıklama tablo üzerinde ise, herhangi bir işlem yapılmaz
            super().mousePressEvent(event)
        else:
            # Tıklama tablo dışında ise, tablo seçimini temizleyin
            self.locationList.selectionModel().clearSelection()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoverWindow()
    window.show()
    sys.exit(app.exec_())
