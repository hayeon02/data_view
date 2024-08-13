import sys
from PySide6.QtCore import Qt
from PySide6.QtWidgets import *
from PySide6.QtMultimedia import QMediaDevices, QCamera, QMediaCaptureSession
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtGui import QPainter,QColor, QPen

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
        self.resize(1200, 800)

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
        # rospy.Subscriber('/current_motor_RPM')
        self.rpm_label.setText("rpm")
    def angle_value(self):
        # rospy.Subscriber('/steering_angle')
        self.angle_label.setText()
    # def status_value(self):
        #status_value = rospy.Subscriber('/manual_status')
        # if status_value == 0:
        #     self.status_label.setText("0")
        # elif status_value = 1:
        #     self.status_label.setText("0")
        #



    def camera(self):
        self.resize(1200, 500)

        available_cameras = QMediaDevices.videoInputs()  # QMediaDevices: 사용 가능한 멀티미디어 입력 및 출력 장치에 대한 정보 제공
        self.video_widget1 = QVideoWidget(self) #객체가 생성한 비디오를 표시하는 위젯
        self.video_widget2 = QVideoWidget(self)
        self.video_widget3 = QVideoWidget(self)
        self.video_widget4 = QVideoWidget(self)

        self.camera1 = QCamera(available_cameras[0])  # QCamera: 카메라 인터페이스 제공
        self.capture_session1 = QMediaCaptureSession()  # QMediaCaptureSession: 오디오 및 비디오 콘텐츠 캡처 허용
        self.capture_session1.setCamera(self.camera1)  # 여러 카메라 간 전환 가능
        self.capture_session1.setVideoOutput(self.video_widget1)  # 이미 비디오 출력이 연결되어 있는 경우 새 비디오 출력으로 교체
        self.camera1.start()

        self.camera2 = QCamera(available_cameras[1])
        self.capture_session2 = QMediaCaptureSession()
        self.capture_session2.setCamera(self.camera2)
        self.capture_session2.setVideoOutput(self.video_widget2)
        self.camera2.start()

        self.camera3 = QCamera(available_cameras[2])
        self.capture_session3 = QMediaCaptureSession()
        self.capture_session3.setCamera(self.camera3)
        self.capture_session3.setVideoOutput(self.video_widget3)
        self.camera3.start()

        self.camera4 = QCamera(available_cameras[3])
        self.capture_session4 = QMediaCaptureSession()
        self.capture_session4.setCamera(self.camera4)
        self.capture_session4.setVideoOutput(self.video_widget4)
        self.camera4.start()

    def seat(self):
        central_widget = QWidget(self)

        layout1 = QVBoxLayout() #세로
        layout1.addWidget(self.video_widget1)
        layout1.addWidget(self.video_widget3)

        layout2 = QHBoxLayout() #가로
        layout2.addWidget(self.vehicle_table)
        layout2.addLayout(layout1)
        layout2.addWidget(self.video_widget2)
        layout2.addWidget(self.video_widget4)

        central_widget.setLayout(layout2)
        self.setCentralWidget(central_widget)

    def keyReleaseEvent(self, e):
            if e.key() == Qt.Key_Escape:
                self.close()

if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = data_view()
   ex.show()
   sys.exit(app.exec_())