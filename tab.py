import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from gps_pan import MainWindow
from sens import MainWindow1

class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        tab1 = QWidget()
        tab1_layout = QVBoxLayout()
        tab1_layout.addWidget(MainWindow())
        tab1.setLayout(tab1_layout)

        tab2 = QWidget()
        tab2_layout = QVBoxLayout()
        tab2_layout.addWidget(MainWindow1())
        tab2.setLayout(tab2_layout)

        tabs = QTabWidget()
        tabs.addTab(tab1, 'GPS')
        tabs.addTab(tab2, 'Sensors')

        # 레이아웃 크기 정책 설정
        tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        vbox = QVBoxLayout()
        vbox.addWidget(tabs)
        self.setLayout(vbox)

        self.setWindowTitle('View')
        self.setMinimumSize(1400, 800)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

if __name__ == '__main__':
    rospy.init_node('vehicle_node', anonymous=True)
    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    sys.exit(app.exec_())
