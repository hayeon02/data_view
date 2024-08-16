import sys
import rospy
import math
from PySide6.QtWidgets import *
from PySide6.QtMultimedia import *
from PySide6.QtMultimediaWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

class vehicle_data(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(320, 60)
        self.vehicle_data()

        self.rpm = 0
        self.gear = 0
        self.status = 0

    def vehicle_data(self):
        self.vehicle_table = QTableWidget(self)
        self.vehicle_table.setFixedSize(320, 60)
        self.vehicle_table.setColumnCount(3)
        self.vehicle_table.setRowCount(1)
        self.vehicle_table.setHorizontalHeaderLabels(["RPM", "Gear", "Status"])

        self.rpm = QLabel('rpm', self)
        self.gear = QLabel('gear', self)
        self.status = QLabel('status', self)

        self.update_rpm(self.rpm)
        self.update_gear(self.gear)
        self.update_status(self.status)

        self.vehicle_table.setCellWidget(0, 0, self.rpm)
        self.vehicle_table.setCellWidget(0, 1, self.gear)
        self.vehicle_table.setCellWidget(0, 2, self.status)

    def load_data(self):
        rospy.init_node('listener_vehicle_data')
        rospy.Subscriber("/current_motor_RPM", UInt32, self.update_rpm)
        rospy.Subscriber("/manual_status", Joy, self.update_status)
        rospy.spin()

    def update_rpm(self, data): #바퀴 둘레 길이 * rpm
        self.rpm = data.data
        self.rpm.setText(str(self.rpm))

    def update_gear(self, data):
        if self.gear == '':
            self.gear.setText("P") #주차
        elif self.gear == '':
            self.gear.setText("R") #후진
        elif self.gear == '':
            self.gear.setText("N") #중립
        elif self.gear == '':
            self.gear.setText("D") #주행

    def update_status(self, data):
        if data.buttons[1]:
            self.status = data.buttons[1]
            self.status.setText("Manual")
        else:
            self.status.setText("Auto")

class vehicle_view(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(300, 300)
        self.angle = 0.0
        self.expected_value = 0.0

    def load_data(self):
        rospy.init_node('listener_vehicle_angle_data')
        rospy.Subscriber("/steering_angle", Float32, self.vehicle_line)
        rospy.spin()

    def vehicle_line(self, data):
        self.angle = data.data
        degree = self.angle * (180/3.14)
        self.expected_value = 110 / math.cos(degree)


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

        painter.drawLine(QPoint(105, self.expected_value), QPoint(105, 80))
        painter.drawLine(QPoint(215, self.expected_value), QPoint(215,80))

class data_view(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui()
        self.cameras()
        self.setupLayout()

    def ui(self):
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)

    def cameras(self):
        available_cameras = QMediaDevices.videoInputs()

        # 카메라 및 세션 초기화
        self.video_widgets = []
        self.cameras = []
        self.capture_sessions = []

        for i in range(4):
            video_widget = QVideoWidget(self)
            camera = QCamera(available_cameras[i])
            capture_session = QMediaCaptureSession()
            capture_session.setCamera(camera)
            capture_session.setVideoOutput(video_widget)
            camera.start()

            self.video_widgets.append(video_widget)
            self.cameras.append(camera)
            self.capture_sessions.append(capture_session)

    def setupLayout(self):
        central_widget = QWidget(self)
        vehicle_data_widget = vehicle_data(self)
        vehicle_view_widget = vehicle_view(self)

        layout1 = QVBoxLayout()
        layout1.addWidget(self.video_widgets[0])
        layout1.addWidget(self.video_widgets[2])

        layout2 = QVBoxLayout()
        layout2.addWidget(self.video_widgets[1])
        layout2.addWidget(self.video_widgets[3])

        layout3 = QVBoxLayout()
        layout3.addWidget(vehicle_data_widget)
        layout3.addWidget(vehicle_view_widget)

        layout4 = QHBoxLayout()
        layout4.addLayout(layout3)
        layout4.addLayout(layout1)
        layout4.addLayout(layout2)

        central_widget.setLayout(layout4)
        self.setCentralWidget(central_widget)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = data_view()
    ex.show()
    sys.exit(app.exec())
