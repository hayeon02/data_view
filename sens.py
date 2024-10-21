import sys
import math
import cv2
import numpy as np
import rospy
import threading
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class VehicleView(QWidget):
    rpm_signal = pyqtSignal(int)
    steer_signal = pyqtSignal(int)
    mode_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(400, 300)

        # rpm, steer, mode 값 초기화
        self.rpm = 0
        self.steer = 512
        self.mode = 'M'

        # 노드 생성
        self.rpm_sub = rospy.Subscriber('/current_motor_RPM', UInt32, self.update_rpm)
        self.steer_sub = rospy.Subscriber('/current_steer', UInt32, self.update_steer)
        self.mode_sub = rospy.Subscriber('/manual_status', Joy, self.update_mode)

        self.init_ui()

        self.rpm_signal.connect(self.update_rpm_ui)
        self.steer_signal.connect(self.update_steer_ui)
        self.mode_signal.connect(self.update_mode_ui)

        # 100ms마다 vehicle view 갱신
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

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

    def update_ui(self):
        self.table.setItem(0, 0, QTableWidgetItem(str(self.rpm)))
        self.table.setItem(0, 1, QTableWidgetItem(str(self.steer)))
        self.table.setItem(0, 2, QTableWidgetItem(str(self.mode)))
        self.car_widget.update_line()

    def update_rpm(self, msg: UInt32):
        self.rpm_signal.emit(msg.data)

    def update_steer(self, msg: UInt32):
        self.steer_signal.emit(msg.data)

    def update_mode(self, msg: Joy):
        mode = 'A' if msg.buttons[4] == 0 else 'M'
        self.mode_signal.emit(mode)

    def update_rpm_ui(self, rpm):
        self.rpm = rpm

    def update_steer_ui(self, steer):
        self.steer = steer

    def update_mode_ui(self, mode):
        self.mode = mode

class CarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(300, 300)
        self.a = 0
        self.b = 100

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
        self.a = int(100 / ((m ** 2) + 1))
        self.b = int(self.a * m)

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

        if hasattr(self, 'a') and hasattr(self, 'b'):
            painter.drawLine(QPoint(int(self.a + 105), int(self.b)), QPoint(105, 80))
            painter.drawLine(QPoint(int(self.a + 215), int(self.b)), QPoint(215, 80))

class CameraView(QWidget):
    def __init__(self):
        super().__init__()
        self.ui()

        self.bridge = CvBridge()
        self.camera_sub1 = rospy.Subscriber('/zed_node/left/image_rect_color', Image, self.update_camera1)
        self.camera_sub2 = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.update_camera2)

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
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            q_image = self.convert_cv_to_qt(cv_image)
            self.label1.setPixmap(QPixmap.fromImage(q_image))
        except Exception as e:
            print(f"Error updating camera1: {e}")

    def update_camera2(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            q_image = self.convert_cv_to_qt(cv_image)
            self.label2.setPixmap(QPixmap.fromImage(q_image))
        except Exception as e:
            print(f"Error updating camera2: {e}")

    def convert_cv_to_qt(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        return QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

class MainWindow1(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 800, 500)
        self.init_ui()

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

def ros_spin():
    rospy.spin()

def main():
    # rospy.init_node('vehicle_node', anonymous=True)
    app = QApplication(sys.argv)
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    main_win = MainWindow1()
    main_win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()