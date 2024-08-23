import sys
import math
import rospy
from PySide6.QtWidgets import QApplication, QWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QMainWindow
from PySide6.QtGui import QPainter, QColor
from PySide6.QtCore import QRect, Qt, QPoint, QTimer
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy


class VehicleView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(620, 300)  # Adjust size to fit both views

        # Initialize ROS node
        rospy.init_node('vehicle_node')

        # Initialize attributes
        self.rpm = 0
        self.steer = 512
        self.mode = 'M'

        # Set up ROS subscribers
        self.rpm_sub = rospy.Subscriber('/current_motor_RPM', UInt32, self.update_rpm)
        self.steer_sub = rospy.Subscriber('/current_steer', UInt32, self.update_steer)
        self.mode_sub = rospy.Subscriber('/manual_status', Joy, self.update_mode)

        # Initialize UI components
        self.init_ui()

        # Timer to periodically update UI from ROS callbacks
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # Update every 100 ms

    def init_ui(self):
        # Set up the table view for status
        self.table = QTableWidget(1, 3, self)
        self.table.setHorizontalHeaderLabels(['RPM', 'Steer', 'Mode'])
        self.table.setFixedSize(320, 60)
        self.table.setItem(0, 0, QTableWidgetItem(str(self.rpm)))
        self.table.setItem(0, 1, QTableWidgetItem(str(self.steer)))
        self.table.setItem(0, 2, QTableWidgetItem(self.mode))

        self.car_widget = CarWidget(self)  # Create CarWidget instance

        layout = QVBoxLayout()
        layout.addWidget(self.table)
        layout.addWidget(self.car_widget)
        self.setLayout(layout)

    def update_ui(self):
        # Ensure UI is updated with latest ROS data
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
        self.x = 0.00
        self.y = 0.00
        self.steer = 512  # Default value, will be updated by VehicleView

    def update_line(self):
        # Access steer from the parent widget (VehicleView)
        if isinstance(self.parent(), VehicleView):
            self.steer = self.parent().steer

        self.line = float((self.steer / 20.00) - 25.00)
        ra = math.radians(self.line)
        a = math.tan(ra)

        if a == 0:
            self.x = 0.00
            self.y = 0.00
        else:
            self.y = (((80 * a) ** 2) / ((a ** 2) + 1)) ** 0.5
            self.x = self.y / a
        self.update()  # Request a repaint to reflect the changes

    def paintEvent(self, event):
        painter = QPainter(self)

        # Drawing the car
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
        painter.drawLine(QPoint(self.x, self.y), QPoint(105, 80))
        painter.drawLine(QPoint((self.x + 100), self.y), QPoint(215, 80))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vehicle Data View')
        self.setGeometry(500, 100, 1200, 800)
        self.init_ui()

    def init_ui(self):
        central_widget = VehicleView(self)
        self.setCentralWidget(central_widget)


def main():
    try:
        app = QApplication(sys.argv)
        main_win = MainWindow()
        main_win.show()
        sys.exit(app.exec())
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == '__main__':
    main()
