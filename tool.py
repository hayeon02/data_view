import sys
from PySide6.QtCore import Qt
from PySide6.QtWidgets import *
from PySide6.QtMultimedia import QMediaDevices, QCamera, QMediaCaptureSession
from PySide6.QtMultimediaWidgets import QVideoWidget

rpm = 0
angle = 0
status = 0
class data_view(QMainWindow):
    def __init__(self):
        super().__init__()
        self.view_IU()
        self.vehicle_data()
        self.camera()
        self.seat()

    def view_IU(self):
        self.setWindowTitle('view')
        self.move(500, 100)
        self.resize(1000, 800)

    def vehicle_data(self):
        self.vehicle_table = QTableWidget(self)
        vehicle_table_coloumn = ["RPM", "Angle", "Status"]
        self.vehicle_table.setFixedSize(320, 60)
        self.vehicle_table.setColumnCount(3)
        self.vehicle_table.setRowCount(1)
        self.vehicle_table.setHorizontalHeaderLabels(vehicle_table_coloumn)

        self.rpm_label = QLabel('rpm', self)
        self.angle_label = QLabel('angle', self)
        self.status_label = QLabel('status', self)

        self.vehicle_table.setCellWidget(0,0, self.rpm_label)
        self.vehicle_table.setCellWidget(0, 1, self.angle_label)
        self.vehicle_table.setCellWidget(0, 2, self.status_label)

    def rpm_value(self):
        self.rpm_label.setText()
        # /current_motor_RPM
    def angle_value(self):
        self.angle_label.setText()
        # /steering_angle
    def status_value(self):
        self.status_label.setText()
        # /manual_status



    def camera(self):
        # self.camera_table = QTableWidget(self)
        # self.camera_table.setFixedSize(800, 300)
        # self.camera_table.setColumnCount(1)
        # self.camera_table.setRowCount(1)

        self.resize(800, 300)
        self.video_widget = QVideoWidget(self)  # 비디오 표시

        available_cameras = QMediaDevices.videoInputs()  # QMediaDevices: 사용 가능한 멀티미디어 입력 및 출력 장치에 대한 정보 제공
        if available_cameras:
            self.camera = QCamera(available_cameras[0])  # QCamera: 카메라 인터페이스 제공
            self.capture_session = QMediaCaptureSession()  # QMediaCaptureSession: 오디오 및 비디오 콘텐츠 캡처 허용
            self.capture_session.setCamera(self.camera)  # 여러 카메라 간 전환 가능
            self.capture_session.setVideoOutput(self.video_widget)  # 이미 비디오 출력이 연결되어 있는 경우 새 비디오 출력으로 교체
            self.camera.start()

    def seat(self):
        central_widget = QWidget(self)
        layout = QHBoxLayout(central_widget)
        layout.addWidget(self.vehicle_table)
        layout.addWidget(self.video_widget)
        self.setCentralWidget(central_widget)

    def keyReleaseEvent(self, e):
            if e.key() == Qt.Key_Escape:
                self.close()

if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = data_view()
   ex.show()
   sys.exit(app.exec_())