import sys
import rospy
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy, CompressedImage, CameraInfo, NavSatFix
from cv_bridge import CvBridge
from threading import Thread

class RosTopicViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        rospy.init_node('ros_topic_viewer')
        self.update_topic_list()

    def initUI(self):
        layout = QVBoxLayout()

        self.refresh_btn = QPushButton('Refresh Topic List', self)
        self.refresh_btn.clicked.connect(self.update_topic_list)
        layout.addWidget(self.refresh_btn)

        self.topic_list_widget = QListWidget(self)
        self.topic_list_widget.setSelectionMode(QListWidget.MultiSelection)  # 다중 선택 가능

        self.start_btn = QPushButton('Start', self)
        self.start_btn.clicked.connect(self.start_mode)

        layout.addWidget(self.topic_list_widget)
        layout.addWidget(self.start_btn)

        self.setLayout(layout)
        self.setWindowTitle('ROS Topic List')
        self.setGeometry(500, 100, 400, 300)

    def update_topic_list(self):
        #현재 활성화된 토픽 리스트를 가져옴
        topics = rospy.get_published_topics()

        self.topic_list_widget.clear()
        for topic in topics:
            self.topic_list_widget.addItem(topic[0])

    def select_topic(self):
        selected_items = self.topic_list_widget.selectedItems()
        selected_topic = [item.text() for item in selected_items]
        return selected_topic

    def start_mode(self):
        selected_topics = self.select_topic()  # 선택된 토픽 리스트를 가져옴
        self.hide()
        self.MainView = MainView(selected_topics)
        self.MainView.exec()

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

class MainView(QDialog):
    def __init__(self, selected_topics):
        super().__init__()
        self.setGeometry(500, 100, 400, 300)
        self.bridge = CvBridge()
        self.selected_topics = selected_topics
        self.ui()
        self.TopicToNode()

    def ui(self):
        self.label1 = QLabel(self)
        self.label1.setAlignment(Qt.AlignCenter)
        self.label1.setFixedSize(640, 480)

        self.label2 = QLabel(self)
        self.label2.setAlignment(Qt.AlignCenter)
        self.label2.setFixedSize(640, 480)

        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.label1)
        self.layout.addWidget(self.label2)

    def TopicToNode(self):
        for topic in self.selected_topics:
            if topic == '/current_motor_RPM':
                self.rpm = 0
                self.rpm_sub = rospy.Subscriber('/current_motor_RPM', UInt32, self.update_rpm)

            elif topic== '/current_steer':
                self.steer = 512
                self.steer_sub = rospy.Subscriber('/current_steer', UInt32, self.update_steer)

            elif topic == '/manual_status':
                self.mode = 'M'
                self.mode_sub = rospy.Subscriber('/manual_status', Joy, self.update_mode)

            elif topic == '/usb_cam/image_raw/compressed':
                self.camera_sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.update_camera)



            elif topic == '/zed_node/left/camera_info':
                self.camera_sub_zed = rospy.Subscriber('/zed_node/left/camera_info', CameraInfo, self.update_camera_zed)

            elif topic == '/ublox_gps/fix':
                self.gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.update_gps)

    def update_rpm(self, msg: UInt32):
        self.rpm = msg.data

    def update_steer(self, msg: UInt32):
        self.steer = msg.data

    def update_mode(self, msg: Joy):
        self.mode = 'A' if msg.buttons[4] == 0 else 'M'

    def update_camera(self, msg: CompressedImage):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(500, 400, Qt.KeepAspectRatio)

        self.label1.setPixmap(pixmap)

    def update_camera_zed(self, msg: CameraInfo):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(500, 400, Qt.KeepAspectRatio)

        self.label2.setPixmap(pixmap)

    def update_gps(self, msg: NavSatFix):
        pass

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = RosTopicViewer()
    viewer.show()

    def ros_spin():
        rospy.spin()

    ros_thread = Thread(target=ros_spin)
    ros_thread.start()

    sys.exit(app.exec())