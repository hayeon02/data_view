#차량 데이터 + 카메라
import sys
import math
import rospy
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VehicleView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(400, 300)

        #rpm, steer, mode 값 초기화
        self.rpm = 0
        self.steer = 512
        self.mode = 'M'

        #노드 생성
        rospy.init_node('vehicle_node')

        self.rpm_sub = rospy.Subscriber('/current_motor_RPM', UInt32, self.update_rpm)
        self.steer_sub = rospy.Subscriber('/current_steer', UInt32, self.update_steer)
        self.mode_sub = rospy.Subscriber('/manual_status', Joy, self.update_mode)

        self.init_ui()

        #100ms마다 vehicle view 갱신
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui) #일정 시간이 지나면 timeout 신호 발생, 발생할때마다 update_ui 메서드 호출
        self.timer.start(100) #단위: 밀리초(ms) 100ms = 0.1s

    # 차량 정보(표형-index)
    def init_ui(self):
        self.table = QTableWidget(1, 3, self)
        self.table.setHorizontalHeaderLabels(['RPM', 'Steer', 'Mode'])
        self.table.setFixedSize(320, 60)
        self.table.setItem(0, 0, QTableWidgetItem(str(self.rpm)))
        self.table.setItem(0, 1, QTableWidgetItem(str(self.steer)))
        self.table.setItem(0, 2, QTableWidgetItem(self.mode))

        self.car_widget = CarWidget(self)

        layout = QVBoxLayout()
        layout.addWidget(self.table)
        layout.addWidget(self.car_widget)
        self.setLayout(layout)

    # 차량 정보(표형-값(변하는 값))
    def update_ui(self):
        self.table.setItem(0, 0, QTableWidgetItem(str(self.rpm)))
        self.table.setItem(0, 1, QTableWidgetItem(str(self.steer)))
        self.table.setItem(0, 2, QTableWidgetItem(self.mode))
        self.car_widget.update_line()  # Update CarWidget when data changes

    def update_rpm(self, msg: UInt32):
        self.rpm = msg.data

    def update_steer(self, msg: UInt32):
        self.steer = msg.data

    def update_mode(self, msg: Joy):
        self.mode = 'A' if msg.buttons[4] == 0 else 'M'


class CarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(300, 300)

    #차량 이동 선 업데이트
    def update_line(self):
        if isinstance(self.parent(), VehicleView):
            self.steer = self.parent().steer

        s_de = (self.steer - 512) * (25 / 512)
        s_ra = math.radians(s_de)

        if s_ra < 0:
            t_ra = 1.57 + s_ra
        elif s_ra > 0:
            t_ra = -1.57 + s_ra
        else:
            self.a = 0
            self.b = 100
            return

        m = math.tan(t_ra)
        self.a = 100 / ((m**2) + 1)
        self.b = self.a * m

        self.update()

    #차량 이미지
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
        painter.drawLine(QPoint(self.a + 105, self.b), QPoint(105, 80))
        painter.drawLine(QPoint((self.a + 215), self.b), QPoint(215, 80))

        painter.setPen(QColor(0, 0, 0))
        painter.drawLine(QPoint(123, -38), QPoint(105, 80))
        painter.drawLine(QPoint(233, 0), QPoint(215, 80))

        print(self.a)
        print(self.b)

class CameraView(QWidget):
    def __init__(self):
        super().__init__()
        self.ui()

        self.bridge = CvBridge()
        self.camera_sub1 = rospy.Subscriber('/usb_cam_1/image_raw', Image, self.update_camera1)
        self.camera_sub2 = rospy.Subscriber('/usb_cam_2/image_raw', Image, self.update_camera2)

    def ui(self):
        self.setGeometry(100, 100, 300, 200)

        self.label1 = QLabel(self)
        self.label1.setAlignment(Qt.AlignCenter)

        self.label2 = QLabel(self)
        self.label2.setAlignment(Qt.AlignCenter)

        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.label1)
        self.layout.addWidget(self.label2)

    def update_camera1(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(500, 400, Qt.KeepAspectRatio)

        self.label1.setPixmap(pixmap)

    def update_camera2(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(500, 400, Qt.KeepAspectRatio)

        self.label2.setPixmap(pixmap)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 800, 500)
        self.init_ui()

    #배치
    def init_ui(self):
        central_widget = QWidget(self)

        camera_widget = CameraView()
        vehicle_widget = VehicleView(self)

        layout = QHBoxLayout()
        layout.addWidget(vehicle_widget)
        layout.addWidget(camera_widget)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

def main():
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()