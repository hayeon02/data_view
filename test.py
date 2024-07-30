import sys
import threading
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtWidgets import *
class data_view(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('view')
        self.move(500, 100)
        self.resize(1000, 800)
        self.vehicle()
    def vehicle(self):
        self.Table = None
        self.Table = QTableWidget(self)
        table_coloumn = ["RPM", "Angle", "Status"]
        self.Table.setFixedSize(350, 60)
        self.Table.setColumnCount(3)
        self.Table.setRowCount(1)
        self.Table.setHorizontalHeaderLabels(table_coloumn)
        self.show()

    def keyReleaseEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.close()


if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = data_view()
   sys.exit(app.exec_())