import sys
from PyQt5.QtWidgets import *
from gps_new import MainWindow2
from no_ros_gps import MainWindow3

class Bar(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setGeometry(100, 100, 1400, 900)

        self.waypoint_select1 = MainWindow3()
        self.waypoint_select2 = MainWindow2()
        # self.gps_line = MainWindow()

        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.waypoint_select2)

        self.setCentralWidget(self.stacked_widget)

        self.create_menu_bar()

    def create_menu_bar(self):
        menu_bar = self.menuBar()

        # page1_action.triggered.connect(): 페이지 이동
        no_ros_gps = QAction('No ROS GPS', self)
        no_ros_gps.triggered.connect(lambda: self.stacked_widget.setCurrentWidget(self.waypoint_select1))

        index5_ation = QAction("index5", self)
        index5_ation.triggered.connect(lambda: self.stacked_widget.setCurrentWidget(self.waypoint_select2))

        # gps_line_ation = QAction("gps_line", self)
        # gps_line_ation.triggered.coonect(lambda: self.stacked_widget.setCurrentWidget(self.gps_line))

        page_menu1 = menu_bar.addMenu("Select")
        page_menu1.addAction(no_ros_gps)
        page_menu1.addAction(index5_ation)

        page_menu2 = menu_bar.addMenu("gps_line")
        # page_menu2.addAction(gps_line_ation)

        page_menu3 = menu_bar.addMenu("Sensor")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = Bar()
    main.show()
    app.exec_()