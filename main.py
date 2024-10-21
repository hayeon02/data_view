import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy, CompressedImage, CameraInfo, NavSatFix
from cv_bridge import CvBridge
from threading import Thread
from tab import MyApp

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
        self.setGeometry(100, 100, 400, 300)

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
        self.setGeometry(100, 100, 400, 300)
        self.selected_topics = selected_topics
        self.setWindowTitle('View')
        self.ui()

    def ui(self):
        self.tab_widget = MyApp()
        layout = QVBoxLayout()
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)

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

    sys.exit(app.exec_())