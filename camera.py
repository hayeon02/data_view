import sys
import rospy
import cv2
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, Qt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraView(QWidget):
    def __init__(self):
        super().__init__()
        self.ui()

        self.bridge = CvBridge()
        rospy.init_node('camera_node', anonymous=True)

        # 이미지 토픽 구독
        self.camera_sub3 = rospy.Subscriber('/zed2i/zed_node/left/image_rect_color', Image, self.update_camera2)

        # ROS 스핀을 위한 타이머 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(10)  # 10밀리초마다 ROS 스핀

    def ui(self):
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)

        # 이미지 표시를 위한 QLabel
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.label)

    def ros_spin(self):
        # rospy.spin() 대신 수동적으로 콜백 처리
        QApplication.processEvents()
        rospy.rostime.wallsleep(0.01)  # 10ms 동안 슬립하여 이벤트 처리

    def update_camera2(self, msg: Image):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # OpenCV 이미지를 QImage로 변환
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        # QImage로 QLabel 업데이트
        self.label.setPixmap(QPixmap.fromImage(q_image))

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)
        self.init_ui()

    def init_ui(self):
        central_widget = CameraView()

        layout = QHBoxLayout()
        layout.addWidget(central_widget)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

def main():
    try:
        app = QApplication(sys.argv)
        main_win = MainWindow()
        main_win.show()
        sys.exit(app.exec())
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == '__main__':
    main()
