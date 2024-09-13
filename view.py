import sys
from ros_topic import RosTopicViewer
from PySide6.QtWidgets import *

class MainView(QWidget):
    def __init__(self):
        super().__init__()
        self.setGeometry(500, 100, 400, 300)
        self.initialize()
        self.show()

    def initialize(self):
        selected_topic = RosTopicViewer.select_topic
        print(selected_topic)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainView()
    window.show()
    sys.exit(app.exec_())