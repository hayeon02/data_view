import sys
import rospy
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, Qt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraView(QWidget):
    def __init__(self):
        super().__init__()
        self.ui()

        self.bridge = CvBridge()
        rospy.init_node('camera_node', anonymous=True)
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
        pixmap = pixmap.scaled(400, 300, Qt.KeepAspectRatio)

        self.label1.setPixmap(pixmap)

    def update_camera2(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(400, 300, Qt.KeepAspectRatio)

        self.label2.setPixmap(pixmap)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Camera')
        self.setGeometry(100, 100, 300, 200)
        self.init_ui()

    def init_ui(self):
        central_widget = QWidget(self)

        camera_widget = CameraView()

        layout = QHBoxLayout()
        layout.addWidget(camera_widget)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

def main():
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
