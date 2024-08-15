import sys
import rospy
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
        self.angle = 0
        self.status = 0

    def vehicle_data(self):
        self.vehicle_table = QTableWidget(self)
        self.vehicle_table.setFixedSize(320, 60)
        self.vehicle_table.setColumnCount(3)
        self.vehicle_table.setRowCount(1)
        self.vehicle_table.setHorizontalHeaderLabels(["RPM", "Angle", "Status"])

        self.rpm = QLabel('rpm', self)
        self.angle = QLabel('angle', self)
        self.status = QLabel('status', self)

        self.updateRPM(self.rpm)
        self.updateAngle(self.angle)
        self.updateStatus(self.status)

        self.vehicle_table.setCellWidget(0, 0, self.rpm)
        self.vehicle_table.setCellWidget(0, 1, self.angle)
        self.vehicle_table.setCellWidget(0, 2, self.status)

    def load_data(self):
        rospy.init_node('listener_vehicle_data')
        rospy.Subscriber("/current_motor_RPM", UInt32, self.updateRPM)
        rospy.Subscriber("/steering_angle", Float32, self.updateAngle)
        rospy.Subscriber("/manual_status", Joy, self.updateStatus)
        rospy.spin()

    def updateRPM(self, data): #바퀴 둘레 길이 * rpm
        self.rpm = data.data
        wheel_size = 3.14 * (바퀴 지름)
        speed_m = wheel_size * (self.rpm / 60) * (기어비)
        #기어비: 드라이브 기어 이빨 수(구동 기어) / 드리븐 기어 이빨 수(수동 기어)
        speed_km = speed_m * 3.6
        self.rpm.setText(str(speed_km))

    def updateAngle(self, data):
        #라디안(57.2958) -> 도 = 라디안 * (180/3.14)
        self.angle = data.data
        degree = self.angle * (180 / 3.14)
        if degree == 0:
            angle_status = 0
        elif degree > 0:
            angle_status = 1
        else:
            angle_status = -1

        # if self.angle == '':
        #     self.angle.setText("P") #주차
        # elif self.angle == '':
        #     self.angle.setText("R") #후진
        # elif self.angle == '':
        #     self.angle.setText("N") #중립
        # elif self.angle == '':
        #     self.angle.setText("D") #주행

    def updateStatus(self, data):
        if data.buttons[1]:
            self.status = data.buttons[1]
            self.status.setText("Manual")
        else:
            self.status.setText("Auto")

class vehicle_view(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(300, 300)

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
        painter.drawLine(QPoint(105, 0), QPoint(105, 80))
        painter.drawLine(QPoint(215, 0), QPoint(215,80))

def vehicle_line(angle_status):
    # 차선 방향 변경

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
