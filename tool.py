import sys
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QLabel, QWidget,
    QVBoxLayout, QHBoxLayout, QTableWidgetItem, QTableWidget,
)
from PySide6.QtMultimedia import QMediaDevices, QCamera, QMediaCaptureSession
from PySide6.QtMultimediaWidgets import QVideoWidget

# 초기 차량 데이터 (예시)
rpm = 0
angle = 0
status = 0

class DataView(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initVehicleData()
        self.initCameras()
        self.setupLayout()

    def initUI(self):
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)

    def initVehicleData(self):
        self.vehicle_table = QTableWidget(self)
        self.vehicle_table.setFixedSize(320, 60)
        self.vehicle_table.setColumnCount(3)
        self.vehicle_table.setRowCount(1)
        self.vehicle_table.setHorizontalHeaderLabels(["RPM", "Angle", "Status"])

        # 초기 값 설정 (추후 업데이트 가능)
        self.rpm_label = QLabel('rpm', self)
        self.angle_label = QLabel('angle', self)
        self.status_label = QLabel('status', self)

        self.vehicle_table.setCellWidget(0, 0, self.rpm_label)
        self.vehicle_table.setCellWidget(0, 1, self.angle_label)
        self.vehicle_table.setCellWidget(0, 2, self.status_label)

    def updateRPM(self, rpm):
        self.rpm_label.setText(str(rpm))

    def updateAngle(self, angle):
        self.angle_label.setText(str(angle))

    def updateStatus(self, status):
        self.status_label.setText(str(status))

    def initCameras(self):
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

        layout1 = QVBoxLayout()
        layout1.addWidget(self.video_widgets[0])
        layout1.addWidget(self.video_widgets[2])

        layout2 = QVBoxLayout()
        layout2.addWidget(self.video_widgets[1])
        layout2.addWidget(self.video_widgets[3])

        layout3 = QHBoxLayout()
        layout3.addWidget(self.vehicle_table)
        layout3.addLayout(layout1)
        layout3.addLayout(layout2)

        central_widget.setLayout(layout3)
        self.setCentralWidget(central_widget)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = DataView()
    ex.show()
    sys.exit(app.exec())
