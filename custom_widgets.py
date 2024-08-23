# Bu dosyada bazı arayüz elemanlarına ek özellikler tanımlanmaktadır ve sonrasında değiştirilmiş
# elemanlar ui dosyasındaki asıl eleman sınfıyla değiştirilmiştir.

from PyQt5.QtWidgets import QGraphicsView, QGraphicsPixmapItem, QTableView
from PyQt5.QtCore import Qt, QRectF, QAbstractTableModel, QModelIndex, QVariant, QMimeData, QDataStream, QByteArray
from PyQt5.QtGui import QPainter, QPixmap, QDrag, QBrush, QColor

# İçerisinde harita öğelerini taşıyacak özelleştirilmiş eleman
class CustomGraphicsView(QGraphicsView):
    
    def __init__(self, scene, parent=None):
        super(CustomGraphicsView, self).__init__(scene, parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setSceneRect(self.scene().itemsBoundingRect())
        self.setDragMode(QGraphicsView.NoDrag)
        self._isPanning = False

    #Panning 
    def mousePressEvent(self, event):
        try:
            if event.button() == Qt.MiddleButton:
                self._isPanning = True
                self.setCursor(Qt.ClosedHandCursor)
                self._lastPos = event.pos()
            if event.button() == Qt.LeftButton:
                scene_pos = self.mapToScene(event.pos())
                self.parent().update_coordinates(scene_pos.x(), scene_pos.y())
            super(CustomGraphicsView, self).mousePressEvent(event)

        except Exception as e:
            print(f"Error in mousePressEvent: {e}")

    def mouseMoveEvent(self, event):
        try:
            if self._isPanning:
                delta = event.pos() - self._lastPos
                self.translateView(delta)
                self._lastPos = event.pos()
            super(CustomGraphicsView, self).mouseMoveEvent(event)

        except Exception as e:
            print(f"Error in mouseMoveEvent: {e}")

    def mouseReleaseEvent(self, event):
        try:
            if event.button() == Qt.MiddleButton:
                self._isPanning = False
                self.setCursor(Qt.ArrowCursor)
            super(CustomGraphicsView, self).mouseReleaseEvent(event)

        except Exception as e:
            print(f"Error in mouseReleaseEvent: {e}")

    def translateView(self, delta):
        try:
            self.setTransformationAnchor(QGraphicsView.NoAnchor)
            self.setResizeAnchor(QGraphicsView.NoAnchor)
            self.translate(delta.x(), delta.y())

        except Exception as e:
            print(f"Error in translateView: {e}")

    #Zooming
    def wheelEvent(self, event):
        try:
            factor = 1.2
            if event.angleDelta().y() < 0:
                factor = 1.0 / factor
            self.scale(factor, factor)

        except Exception as e:
            print(f"Error in wheelEvent: {e}")

    #Harita görselini yükleme
    def load_image(self, image_path):
        try:
            pixmap = QPixmap(image_path)
            scaled_pixmap = pixmap.scaled(self.viewport().size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.scene().clear()
            self.scene().addPixmap(scaled_pixmap)
            self.setSceneRect(QRectF(pixmap.rect()))

        except Exception as e:
            print(f"Error in load_image: {e}")



# İçerisinde roverın gitmesi gereken konumları içeren tablonun modeli
class MyTableModel(QAbstractTableModel):
    def __init__(self, data=None, parent=None):
        super(MyTableModel, self).__init__(parent)
        self._data = data if data is not None else []
        self.headers = ["x", "y"]
        self.highlight_row = -1 # "Go" komudu verildiğinde doğru satırın etkilenmesini sağlayacak değişken
        
    def rowCount(self, parent=QModelIndex()):
        return len(self._data)
    
    def columnCount(self, parent=QModelIndex()):
        return len(self.headers)
    
    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return QVariant()
        
        if role == Qt.DisplayRole:
            value = self._data[index.row()][index.column()]
            return QVariant(value)
        
        if role == Qt.BackgroundRole:
            # Satırın arka plan rengini belirler
            if index.row() == self.highlight_row:
                return QBrush(QColor('#20bd0f'))
        return QVariant()
    
    # İlk satırın renk değişiminde rol oynar
    def set_highlight_row(self, row):
        self.highlight_row = row
        self.dataChanged.emit(QModelIndex(), QModelIndex())
    
    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                if 0 <= section < len(self.headers):
                    return QVariant(self.headers[section])
            
            if orientation == Qt.Vertical:
                return str(section + 1)
        return QVariant()
    
    # Verilerin değiştiği durumlarda tablonun yenilenmesini sağlayan kısım
    def setData(self, index, value, role):
        if role == Qt.EditRole:
            self._data[index.row()][index.column()] = value
            self.dataChanged.emit(index, index, [Qt.DisplayRole])
            return True
        return False
    
    def swapRows(self, row1, row2):
        self.beginMoveRows(QModelIndex(), row1, row1, QModelIndex(), row2 + 1 if row1 < row2 else row2)
        self._data[row1], self._data[row2] = self._data[row2], self._data[row1]
        self.endMoveRows()
        self.dataChanged.emit(self.index(0, 0), self.index(self.rowCount(None) - 1, self.columnCount(None) - 1), [Qt.DisplayRole])

    def clearSelection(self):
        self.selected_row = None
        self.dataChanged.emit(self.index(0, 0), self.index(self.rowCount(None) - 1, self.columnCount(None) - 1), [Qt.BackgroundRole])

    def addData(self, new_row):
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._data.append(new_row)
        self.endInsertRows()

    def removeRow(self, row):
        self.beginRemoveRows(QModelIndex(), row, row)
        self._data.pop(row)
        self.endRemoveRows()

# Gidilecek konumların listesini içeren tablonun ekstra özellikleri tanımlandı
class CustomTableView(QTableView):
    def __init__(self, parent=None):
        super(CustomTableView, self).__init__(parent)
        self.double_clicked_row = None

    # Bir satıra çift tıklama durumunda başka bir satırla yer değiştirme için tetiklenir
    def mouseDoubleClickEvent(self, event):
        index = self.indexAt(event.pos())
        if index.isValid():
            self.double_clicked_row = index.row()
            self.model().selected_row = self.double_clicked_row
            self.model().dataChanged.emit(self.model().index(0, 0), self.model().index(self.model().rowCount(None) - 1, self.model().columnCount(None) - 1), [Qt.BackgroundRole])
        super(CustomTableView, self).mouseDoubleClickEvent(event)

    # Çift tıklamanın ardından başka bir hücreye sol click ile tıklandığında satırlar yer değiştirir,
    # sağ click durumunda ise seçim iptal edilir 
    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton and self.double_clicked_row is not None:
            self.model().clearSelection()
            self.double_clicked_row = None
        elif event.button() == Qt.LeftButton and self.double_clicked_row is not None:
            index = self.indexAt(event.pos())
            if index.isValid() and index.row() != self.double_clicked_row:
                self.model().swapRows(self.double_clicked_row, index.row())
                self.model().clearSelection()
                self.double_clicked_row = None
        super(CustomTableView, self).mousePressEvent(event)

