import sys
import math
from PySide6.QtWidgets import *
from PySide6.QtMultimedia import *
from PySide6.QtMultimediaWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
import rospy
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy
class StatusWindow(QTableWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(320, 60)

        self.rpm = 0
        self.steer = 512
        self.mode = 'M'

        rospy.init_node('status_node')

        self.rpm_sub = rospy.Subscriber('/current_motor_RPM', UInt32, self.update_rpm)
        self.steer_sub = rospy.Subscriber('/current_steer', UInt32, self.update_steer)
        self.mode_sub = rospy.Subscriber('/manual_status', Joy, self.update_mode)

        self.init_ui()

    def init_ui(self):
        self.setColumnCount(3)
        self.setHorizontalHeaderLabels(['RPM', 'Steer', 'Mode'])
        self.setRowCount(1)
        self.setItem(0, 0, QTableWidgetItem(str(self.rpm)))
        self.setItem(0, 1, QTableWidgetItem(str(self.steer)))
        self.setItem(0, 2, QTableWidgetItem(self.mode))

    def update_rpm(self, msg: UInt32):
        self.rpm = msg.data
        self.setItem(0, 0, QTableWidgetItem(str(self.rpm)))

    def update_steer(self, msg: UInt32):
        g_steer = msg.data
        self.setItem(0, 1, QTableWidgetItem(str(g_steer)))

    def update_mode(self, msg: Joy):
        if msg.buttons[4] == 0:
            self.mode = 'A'
        else:
            self.mode = 'M'
        self.setItem(0, 2, QTableWidgetItem(self.mode))

class CarView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(300, 300)

        self.x = 0.00
        self.y = 0.00

    def update_line(self):
        self.line = float((self.steer / 20.00 ) - 25.00)
        ra = math.radians(self.line)
        a = math.tan(ra)
        # print(ra)

        if a == 0:
            self.x = 0.00
            self.y = 0.00
        else:
            self.y = (((80*a)**2) / ((a**2)+1)) ** 0.5
            self.x = self.y / a
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        car = QRect(120, 70, 80, 150)
        wheel1 = QRect(100, 80, 10, 30)
        wheel2 = QRect(100, 180, 10, 30)
        wheel3 = QRect(210, 80, 10, 30)
        wheel4 = QRect(210, 180, 10, 30)

        painter.setBrush(QColor(0, 0, 0))
        painter.drawRect(car)
        painter.drawRect(wheel1)
        painter.drawRect(wheel2)
        painter.drawRect(wheel3)
        painter.drawRect(wheel4)

        painter.setPen(QColor(255, 0, 0))

        painter.drawLine(QPoint(self.x, self.y), QPoint(105, 80))
        painter.drawLine(QPoint((self.x+100), self.y), QPoint(215, 80))

class CameraView(QWidget):
    def __init__(self):
        super().__init__()
        self.ui()
        # self.cameras()

    def ui(self):
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)

    # def cameras(self):
    #     available_cameras = QMediaDevices.videoInputs()
    #
    #     # 카메라 및 세션 초기화
    #     self.video_widgets = []
    #     self.cameras = []
    #     self.capture_sessions = []
    #
    #     for i in range(2):
    #         video_widget = QVideoWidget(self)
    #         camera = QCamera(available_cameras[i])
    #         capture_session = QMediaCaptureSession()
    #         capture_session.setCamera(camera)
    #         capture_session.setVideoOutput(video_widget)
    #         camera.start()
    #
    #         self.video_widgets.append(video_widget)
    #         self.cameras.append(camera)
    #         self.capture_sessions.append(capture_session)

class MainView(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)
        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(100)

    def init_ui(self):
        central_widget = QWidget(self)
        status_view = StatusWindow()
        car_view = CarView()
        camea_view = CameraView()

        layout = QVBoxLayout()
        layout.addWidget(status_view)
        layout.addWidget(car_view)

        layout1 = QVBoxLayout()
        # layout1.addWidget(camea_view.video_widgets[0])
        # layout1.addWidget(camea_view.video_widgets[1])

        layout2 = QHBoxLayout()
        layout2.addLayout(layout)
        layout2.addLayout(layout1)

        central_widget.setLayout(layout2)
        self.setCentralWidget(central_widget)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

    def spin_once(self):
        rospy.rostime.wallsleep(0.05)

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ex = MainView()
        ex.show()
        sys.exit(app.exec())
    except Exception:
        print("An error occurred.")