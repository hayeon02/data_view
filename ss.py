import sys
import rospy
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class RosTopicViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        rospy.init_node('ros_topic_viewer')

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
        # 현재 활성화된 토픽 리스트를 가져옴
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
        self.setGeometry(500, 100, 600, 800)  # 크기를 좀 더 키움

        self.bridge = CvBridge()

        # QLabel 저장을 위한 리스트
        self.camera_labels = []

        # 레이아웃 설정
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.TopicToNode(selected_topics)

    def TopicToNode(self, selected_topics):
        self.selected_topics = selected_topics

        # 선택한 카메라 토픽에 따라 QLabel 생성
        for topic in self.selected_topics:
            if topic == '/usb_cam_1/image_raw' or topic == '/usb_cam_2/image_raw':
                label = QLabel(self)  # 카메라 이미지를 표시할 QLabel 생성
                self.camera_labels.append(label)  # 생성된 QLabel을 리스트에 저장
                self.layout.addWidget(label)  # 레이아웃에 QLabel 추가

                # 토픽에 따라 ROS 구독 설정
                self.camera_sub = rospy.Subscriber(topic, Image, self.update_camera)

    def update_camera(self, msg: Image):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # OpenCV 이미지 -> QImage 변환
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        # 토픽에 따라 이미지를 각 QLabel에 표시
        for i, label in enumerate(self.camera_labels):
            pixmap = QPixmap.fromImage(q_image)
            pixmap = pixmap.scaled(500, 400, Qt.KeepAspectRatio)
            label.setPixmap(pixmap)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = RosTopicViewer()
    viewer.show()
    sys.exit(app.exec())
